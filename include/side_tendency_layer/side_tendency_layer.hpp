// Copyright 2025 Matthias Kalenberg, Song Li
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef SIDE_TENDENCY_LAYER__SIDE_TENDENCY_LAYER_HPP_
#define SIDE_TENDENCY_LAYER__SIDE_TENDENCY_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace side_tendency_layer
{

/// A 2D point in the frame of the laser scanner.
struct Point2D
{
  double x;
  double y;
};

/// Result of a line fit y = slope * x + intercept.
struct LineFit
{
  bool valid;        ///< false if there were fewer than min_wall_points points
  double slope;
  double intercept;
  double r_squared;  ///< coefficient of determination, Eq. 5 of the paper
  double mae;        ///< mean absolute error, Eq. 6 of the paper
};

/// A costmap cell to be written, in the costmap's global frame.
struct CostPoint
{
  double x;
  double y;
  unsigned char cost;
};

/**
 * @class SideTendencyLayer
 * @brief Costmap layer that adds lateral costs inside corridors so that the robot keeps to
 *        one side (Kalenberg et al., "Side Tendency Layer", ECMR 2025).
 *
 * The walls left and right of the robot are fitted into the laser scan. If they form a valid
 * corridor, the comfort function of Morales et al. is laid between them as costs, from the
 * robot to perception_range ahead. Otherwise the layer adds nothing and the rest of the
 * costmap acts as the fallback.
 */
class SideTendencyLayer : public nav2_costmap_2d::Layer
{
public:
  SideTendencyLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /// All valid scan points, in the scanner frame.
  static std::vector<Point2D> scanPoints(const sensor_msgs::msg::LaserScan & msg);
  LineFit fitLine(const std::vector<Point2D> & points) const;
  /// Intercept of a line with known slope by least absolute error (median of residuals, Eq. 8).
  static double medianIntercept(double slope, const std::vector<Point2D> & points);
  /// Distance between the nearest and the farthest point from the scanner.
  static double wallLength(const std::vector<Point2D> & points);
  double wallScore(const LineFit & fit) const;
  /// Cost of a position 0 <= y < 1 across the corridor (0 = left wall), Eq. 1 and 2.
  unsigned char comfortCost(double y) const;

  // Parameters, see README.md. Defaults are the values of the paper's experiments.
  std::string scan_topic_{"/scan"};
  double perception_range_{1.0};
  double corridor_min_width_{2.0};
  double corridor_max_width_{6.0};
  double sample_step_{0.03};
  int max_cost_{127};
  double comfort_a_{0.1};
  double comfort_b_{0.3};
  double comfort_c_{0.99};
  double comfort_eta_{0.604};
  double score_alpha_{0.5};
  double score_lambda_{10.0};
  int min_wall_points_{5};
  double min_wall_length_{0.2};
  double max_wall_mae_{2.5};
  double min_slope_product_{-0.3};

  std::string global_frame_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::mutex mutex_;                   ///< guards the parameters and the cost points
  std::vector<CostPoint> cost_points_;  ///< costs of the last valid corridor
  bool has_last_bounds_{false};        ///< area written in the last update, to clear it
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};
};

}  // namespace side_tendency_layer

#endif  // SIDE_TENDENCY_LAYER__SIDE_TENDENCY_LAYER_HPP_
