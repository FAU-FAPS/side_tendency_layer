// Copyright 2025 Matthias Kalenberg, Song Li
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "side_tendency_layer/side_tendency_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

using rcl_interfaces::msg::ParameterType;

namespace side_tendency_layer
{

void SideTendencyLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("scan_topic", rclcpp::ParameterValue(scan_topic_));
  declareParameter("perception_range", rclcpp::ParameterValue(perception_range_));
  declareParameter("corridor_min_width", rclcpp::ParameterValue(corridor_min_width_));
  declareParameter("corridor_max_width", rclcpp::ParameterValue(corridor_max_width_));
  declareParameter("sample_step", rclcpp::ParameterValue(sample_step_));
  declareParameter("max_cost", rclcpp::ParameterValue(max_cost_));
  declareParameter("comfort_a", rclcpp::ParameterValue(comfort_a_));
  declareParameter("comfort_b", rclcpp::ParameterValue(comfort_b_));
  declareParameter("comfort_c", rclcpp::ParameterValue(comfort_c_));
  declareParameter("comfort_eta", rclcpp::ParameterValue(comfort_eta_));
  declareParameter("score_alpha", rclcpp::ParameterValue(score_alpha_));
  declareParameter("score_lambda", rclcpp::ParameterValue(score_lambda_));
  declareParameter("min_wall_points", rclcpp::ParameterValue(min_wall_points_));
  declareParameter("min_wall_length", rclcpp::ParameterValue(min_wall_length_));
  declareParameter("max_wall_mae", rclcpp::ParameterValue(max_wall_mae_));
  declareParameter("min_slope_product", rclcpp::ParameterValue(min_slope_product_));

  const std::string p = name_ + ".";
  node->get_parameter(p + "enabled", enabled_);
  node->get_parameter(p + "scan_topic", scan_topic_);
  node->get_parameter(p + "perception_range", perception_range_);
  node->get_parameter(p + "corridor_min_width", corridor_min_width_);
  node->get_parameter(p + "corridor_max_width", corridor_max_width_);
  node->get_parameter(p + "sample_step", sample_step_);
  node->get_parameter(p + "max_cost", max_cost_);
  node->get_parameter(p + "comfort_a", comfort_a_);
  node->get_parameter(p + "comfort_b", comfort_b_);
  node->get_parameter(p + "comfort_c", comfort_c_);
  node->get_parameter(p + "comfort_eta", comfort_eta_);
  node->get_parameter(p + "score_alpha", score_alpha_);
  node->get_parameter(p + "score_lambda", score_lambda_);
  node->get_parameter(p + "min_wall_points", min_wall_points_);
  node->get_parameter(p + "min_wall_length", min_wall_length_);
  node->get_parameter(p + "max_wall_mae", max_wall_mae_);
  node->get_parameter(p + "min_slope_product", min_slope_product_);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SideTendencyLayer::dynamicParametersCallback, this, std::placeholders::_1));

  // Costs are written into the frame of the costmap this layer belongs to.
  global_frame_ = layered_costmap_->getGlobalFrameID();

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SideTendencyLayer::laserCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Side tendency layer %s: scan %s, costmap frame %s, preferred side %s (comfort_c %.2f)",
    name_.c_str(), scan_topic_.c_str(), global_frame_.c_str(),
    comfort_c_ > 0.5 ? "right" : "left", comfort_c_);

  current_ = true;
}

rcl_interfaces::msg::SetParametersResult SideTendencyLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  const std::map<std::string, double *> doubles{
    {"perception_range", &perception_range_}, {"corridor_min_width", &corridor_min_width_},
    {"corridor_max_width", &corridor_max_width_}, {"sample_step", &sample_step_},
    {"comfort_a", &comfort_a_}, {"comfort_b", &comfort_b_}, {"comfort_c", &comfort_c_},
    {"comfort_eta", &comfort_eta_}, {"score_alpha", &score_alpha_},
    {"score_lambda", &score_lambda_}, {"min_wall_length", &min_wall_length_},
    {"max_wall_mae", &max_wall_mae_}, {"min_slope_product", &min_slope_product_}};
  const std::map<std::string, int *> integers{
    {"max_cost", &max_cost_}, {"min_wall_points", &min_wall_points_}};
  const std::string prefix = name_ + ".";

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Values that would break the layer are rejected as a whole.
  for (const auto & parameter : parameters) {
    const std::string & full_name = parameter.get_name();
    if (full_name.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string name = full_name.substr(prefix.size());
    const bool positive = name == "perception_range" || name == "sample_step" ||
      name == "comfort_b" || name == "comfort_eta";
    if (positive && parameter.get_type() == ParameterType::PARAMETER_DOUBLE &&
      parameter.as_double() <= 0.0)
    {
      result.successful = false;
      result.reason = name + " must be positive";
      return result;
    }
    if (name == "max_cost" && parameter.get_type() == ParameterType::PARAMETER_INTEGER &&
      (parameter.as_int() < 0 || parameter.as_int() > nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
    {
      result.successful = false;
      result.reason = "max_cost must be between 0 and 252";
      return result;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & parameter : parameters) {
    const std::string & full_name = parameter.get_name();
    if (full_name.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string name = full_name.substr(prefix.size());
    const auto type = parameter.get_type();
    if (type == ParameterType::PARAMETER_DOUBLE && doubles.count(name)) {
      *doubles.at(name) = parameter.as_double();
    } else if (type == ParameterType::PARAMETER_INTEGER && integers.count(name)) {
      *integers.at(name) = static_cast<int>(parameter.as_int());
    } else if (type == ParameterType::PARAMETER_BOOL && name == "enabled") {
      enabled_ = parameter.as_bool();
    }
  }
  return result;
}

void SideTendencyLayer::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    cost_points_.clear();
    return;
  }

  // A line through all scan points gives the rough corridor direction and separates the points
  // within perception_range into left and right (Algorithm 1 and Fig. 3 of the paper).
  const std::vector<Point2D> points = scanPoints(*msg);
  const LineFit corridor = fitLine(points);
  if (!corridor.valid) {
    RCLCPP_DEBUG(logger_, "No corridor: not enough scan points");
    cost_points_.clear();
    return;
  }
  std::vector<Point2D> left, right;
  for (const auto & point : points) {
    if (point.x > 0.0 && point.x <= perception_range_) {
      (point.y > corridor.slope * point.x + corridor.intercept ? left : right).push_back(point);
    }
  }

  // Validity check of the corridor (Section III-A of the paper).
  const LineFit left_fit = fitLine(left);
  const LineFit right_fit = fitLine(right);
  const char * invalid = nullptr;
  if (!left_fit.valid || !right_fit.valid) {
    invalid = "not enough points on one side";
  } else if (wallLength(left) < min_wall_length_ || wallLength(right) < min_wall_length_) {
    invalid = "one wall is too short";
  } else if (left_fit.r_squared < 0.0 && right_fit.r_squared < 0.0) {
    invalid = "no wall fits a line (R^2 < 0)";
  } else if (left_fit.mae > max_wall_mae_ && right_fit.mae > max_wall_mae_) {
    invalid = "MAE of both walls too large";
  } else if (left_fit.slope * right_fit.slope < min_slope_product_) {
    invalid = "walls not parallel";
  }
  if (invalid) {
    RCLCPP_DEBUG(logger_, "No corridor: %s", invalid);
    cost_points_.clear();
    return;
  }

  // The wall with the higher confidence score gives the corridor direction (Eq. 7);
  // the intercept of the other wall follows from least absolute error (Eq. 8).
  double slope, left_intercept, width_along_y;
  if (wallScore(left_fit) > wallScore(right_fit)) {
    slope = left_fit.slope;
    left_intercept = left_fit.intercept;
    width_along_y = std::abs(medianIntercept(slope, right)) + std::abs(left_fit.intercept);
  } else {
    slope = right_fit.slope;
    left_intercept = medianIntercept(slope, left);
    width_along_y = std::abs(right_fit.intercept) + std::abs(left_intercept);
  }

  // Width perpendicular to the walls.
  const double corridor_width = width_along_y / std::sqrt(1.0 + slope * slope);
  if (corridor_width <= corridor_min_width_ || corridor_width >= corridor_max_width_) {
    RCLCPP_DEBUG(
      logger_, "No corridor: width %.2f m outside (%.2f, %.2f)", corridor_width,
      corridor_min_width_, corridor_max_width_);
    cost_points_.clear();
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(global_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(logger_, "No transform to %s: %s", global_frame_.c_str(), ex.what());
    return;
  }
  const auto & q = transform.transform.rotation;
  const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
  const tf2::Vector3 translation(
    transform.transform.translation.x, transform.transform.translation.y, 0.0);

  // Sample the parallelogram between the walls (Section III-B): rows along the corridor from
  // the scanner to perception_range, columns from the left wall to the right one.
  const int rows = static_cast<int>(std::ceil(perception_range_ / sample_step_));
  const int cols = static_cast<int>(std::ceil(width_along_y / sample_step_));
  std::vector<unsigned char> column_cost(cols);
  for (int j = 0; j < cols; ++j) {
    column_cost[j] = comfortCost(static_cast<double>(j) / cols);
  }

  cost_points_.clear();
  cost_points_.reserve(static_cast<size_t>(rows) * cols);
  for (int i = 0; i < rows; ++i) {
    const double x = i * sample_step_;
    const double left_wall_y = slope * x + left_intercept;
    for (int j = 0; j < cols; ++j) {
      const tf2::Vector3 point =
        rotation * tf2::Vector3(x, left_wall_y - j * sample_step_, 0.0) + translation;
      cost_points_.push_back({point.x(), point.y(), column_cost[j]});
    }
  }
}

void SideTendencyLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // The area written in the last update is included, so that it is cleared.
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }
  has_last_bounds_ = enabled_ && !cost_points_.empty();
  if (!has_last_bounds_) {
    return;
  }

  last_min_x_ = last_min_y_ = std::numeric_limits<double>::max();
  last_max_x_ = last_max_y_ = std::numeric_limits<double>::lowest();
  for (const auto & point : cost_points_) {
    last_min_x_ = std::min(last_min_x_, point.x);
    last_min_y_ = std::min(last_min_y_, point.y);
    last_max_x_ = std::max(last_max_x_, point.x);
    last_max_y_ = std::max(last_max_y_, point.y);
  }
  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void SideTendencyLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & point : cost_points_) {
    unsigned int mx, my;
    if (!master_grid.worldToMap(point.x, point.y, mx, my)) {
      continue;
    }
    const int i = static_cast<int>(mx), j = static_cast<int>(my);
    if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
      continue;
    }
    master_grid.setCost(mx, my, std::max(point.cost, master_grid.getCost(mx, my)));
  }
}

void SideTendencyLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cost_points_.clear();
}

std::vector<Point2D> SideTendencyLayer::scanPoints(const sensor_msgs::msg::LaserScan & msg)
{
  std::vector<Point2D> points;
  points.reserve(msg.ranges.size());
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const double range = msg.ranges[i];
    if (!std::isfinite(range) || range < msg.range_min || range > msg.range_max) {
      continue;
    }
    const double angle = msg.angle_min + i * msg.angle_increment;
    points.push_back({range * std::cos(angle), range * std::sin(angle)});
  }
  return points;
}

LineFit SideTendencyLayer::fitLine(const std::vector<Point2D> & points) const
{
  const int n = static_cast<int>(points.size());
  if (n < min_wall_points_ || n < 2) {
    return {false, 0.0, 0.0, 0.0, 0.0};
  }

  // Least squares fit (Eq. 4).
  Eigen::MatrixXd A(n, 2);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; ++i) {
    A(i, 0) = points[i].x;
    A(i, 1) = 1.0;
    b(i) = points[i].y;
  }
  const Eigen::VectorXd line = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  const Eigen::VectorXd residuals = b - A * line;

  const double ss_total = (b.array() - b.mean()).matrix().squaredNorm();
  const double r_squared = 1.0 - residuals.squaredNorm() / ss_total;
  const double mae = residuals.array().abs().mean();
  return {true, line(0), line(1), r_squared, mae};
}

double SideTendencyLayer::medianIntercept(double slope, const std::vector<Point2D> & points)
{
  if (points.empty()) {
    return 0.0;
  }
  std::vector<double> residuals;
  residuals.reserve(points.size());
  for (const auto & p : points) {
    residuals.push_back(p.y - slope * p.x);
  }
  std::sort(residuals.begin(), residuals.end());
  const size_t n = residuals.size();
  return n % 2 == 0 ? (residuals[n / 2 - 1] + residuals[n / 2]) / 2.0 : residuals[n / 2];
}

double SideTendencyLayer::wallLength(const std::vector<Point2D> & points)
{
  if (points.empty()) {
    return 0.0;
  }
  auto distance = [](const Point2D & p) {return p.x * p.x + p.y * p.y;};
  const auto nearest_farthest = std::minmax_element(
    points.begin(), points.end(),
    [&](const Point2D & a, const Point2D & b) {return distance(a) < distance(b);});
  const Point2D & a = *nearest_farthest.first;
  const Point2D & b = *nearest_farthest.second;
  return std::hypot(a.x - b.x, a.y - b.y);
}

double SideTendencyLayer::wallScore(const LineFit & fit) const
{
  return score_alpha_ * fit.r_squared + (1.0 - score_alpha_) * std::exp(-score_lambda_ * fit.mae);
}

unsigned char SideTendencyLayer::comfortCost(double y) const
{
  // Passenger comfort p(y) of Morales et al. (Eq. 1 and 2); the cost is the discomfort,
  // scaled to max_cost.
  constexpr double epsilon = 1e-6;
  y = std::clamp(y, epsilon, 1.0 - epsilon);
  const double u = comfort_a_ / y + comfort_a_ / (1.0 - y) +
    std::pow((y - comfort_c_) / comfort_b_, 2);
  const double comfort = std::clamp(std::exp(-u) / comfort_eta_, 0.0, 1.0);
  return static_cast<unsigned char>(max_cost_ - static_cast<int>(comfort * max_cost_));
}

}  // namespace side_tendency_layer

PLUGINLIB_EXPORT_CLASS(side_tendency_layer::SideTendencyLayer, nav2_costmap_2d::Layer)
