# Side Tendency Layer: Enhancing Social Robot Navigation for Passenger and Pedestrian Comfort Using Local Sensor Data

<p align="center">
  <img src="docs/SideTendency-GraphicalAbstract.png" alt="Graphical Abstract" width="600">
</p>

Implementation of our paper **"Side Tendency Layer: Enhancing Social Robot Navigation for
Passenger and Pedestrian Comfort Using Local Sensor Data"**, ECMR 2025,
doi: [10.1109/ECMR65884.2025.11163026](https://doi.org/10.1109/ECMR65884.2025.11163026).

The side tendency layer is a [Nav2](https://nav2.org) costmap layer that makes a robot keep to one
side of a corridor, like pedestrians do. It detects the walls left and right of the robot in the
laser scan and lays a comfort function across the corridor as costs. Wherever no valid corridor is
detected, the layer adds nothing and the rest of the costmap works as usual. The repository also
contains the simulation scenarios of the paper, so the results can be reproduced with the Nav2
tutorial TurtleBot3.

## How it works

For every laser scan, in the frame of the scanner:

1. A line fitted through all scan points gives the rough corridor direction. It splits the
   scan points ahead of the robot, up to `perception_range`, into a left and a right set
   (Algorithm 1 of the paper).
2. A line is fitted into each set by least squares. The corridor is valid if both walls have
   enough points and length, fit a line, are roughly parallel and the corridor width lies between
   `corridor_min_width` and `corridor_max_width`.
3. The wall with the higher confidence score (Eq. 7 of the paper) gives the corridor direction.
   The other wall's position follows by least absolute error (Eq. 8).
4. The parallelogram between the walls is sampled. Each sample gets the cost of the comfort
   function of Morales et al. (Eq. 1 and 2) at its position across the corridor.
5. The samples are written into the costmap's global frame, taking the maximum with the existing
   costs.

The layer is meant for the global costmap. The local planner follows the global path closely, so
the side tendency acts through the global planner.

## Installation

Tested with ROS 2 Humble on Ubuntu 22.04.

```bash
mkdir -p ~/side_tendency_ws/src && cd ~/side_tendency_ws/src
git clone https://github.com/FAU-FAPS/side_tendency_layer.git
cd ~/side_tendency_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Usage

Add the layer to the plugins of the global costmap, after the inflation layer:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "side_tendency_layer"]
      side_tendency_layer:
        plugin: "side_tendency_layer/SideTendencyLayer"
        scan_topic: /scan
        comfort_c: 0.99          # keep right; values below 0.5 keep left
```

The scanner may use any angle convention, from 0 to 2π or from −π to π. The scan frame must be
connected to the costmap's global frame by TF. `config/nav2_tb3_side_tendency.yaml` is a complete
example with every parameter.

### Parameters

All parameters are read from the layer's namespace and can be changed at runtime with
`ros2 param set`, except `scan_topic`. Defaults are the values of the paper's experiments.

| Parameter | Paper | Default | Description |
|---|---|---|---|
| `enabled` | | `true` | Whether the layer adds costs |
| `scan_topic` | | `/scan` | `sensor_msgs/LaserScan` topic |
| `perception_range` | r | `1.0` | Scan points this far ahead [m] are used, and costs are drawn this far ahead. A short range delays the side shift. |
| `corridor_min_width` | | `2.0` | Narrower corridors [m] get no side tendency |
| `corridor_max_width` | | `6.0` | Wider areas [m] are not treated as corridor |
| `min_wall_points` | | `5` | Scan points needed on each side |
| `min_wall_length` | | `0.2` | Shortest wall [m] that counts |
| `max_wall_mae` | MAE limit | `2.5` | No corridor if the mean absolute error of both walls is larger [m] |
| `min_slope_product` | parallel check | `-0.3` | Walls count as not parallel if the product of their slopes is lower |
| `score_alpha` | α | `0.5` | Weight of R² against the MAE term in the wall confidence score |
| `score_lambda` | λ | `10.0` | Sensitivity of the confidence score to the MAE |
| `comfort_a` | a | `0.1` | Discomfort close to the walls |
| `comfort_b` | b | `0.3` | Sensitivity to leaving the preferred position |
| `comfort_c` | c | `0.99` | Preferred position across the corridor, 0 = left wall, 1 = right wall |
| `comfort_eta` | η | `0.604` | Normalises the maximal comfort to 1 |
| `max_cost` | | `127` | Cost of the least comfortable position, at most 252 |
| `sample_step` | | `0.03` | Spacing of the sampled cost points [m] |

## Simulation

Each scenario has its own launch file. It starts the Nav2 tutorial simulation
(`nav2_bringup/tb3_simulation_launch.py`: Gazebo, TurtleBot3 waffle, AMCL, Nav2 with DWB, RViz)
with `config/nav2_tb3_side_tendency.yaml` and the scenario's world, map and poses. Worlds, maps
and models all come with this package. As in the paper's simulation, the layer runs in the global
costmap only. DWB and the layer use the tuning of the paper's simulation config, and the
inflation layer keeps the Nav2 default.

<p align="center">
  <img src="docs/laboratory_paths.png" alt="Paths of a TurtleBot3 in the laboratory simulation with and without the layer" width="800">
</p>

*TurtleBot3 in the laboratory scenario, driving east. Red: plain Nav2. Blue: with the side
tendency layer, the robot keeps to the right.*

| Launch file | Scenario | Demo route in the map frame |
|---|---|---|
| `sim_office.launch.py` | Simulation of Figure 6a in the paper: a loop around an inner block with corridors of different widths, doors, a crossing, pedestrians and cluttered areas | Through the four corners of the loop, taken from the global plan of the paper's run |
| `sim_laboratory.launch.py` | Long corridor of a laboratory, 2.9 m wide | `7 -3.45 0`, east end of the corridor |
| `sim_straight_corridor.launch.py` | Straight 20 m corridor, 2.85 m wide | `17 0 0`, end of the corridor |
| `sim_doorway.launch.py` | Straight corridor with a doorway | `17 0 0`, end of the corridor |
| `sim_intersection.launch.py` | Crossing of two corridors | `17 0 0`, through the crossing |
| `sim_corridor_turns.launch.py` | Corridor with two turns, about 1.7 to 1.9 m wide | `12.3 -6.1 0`, right arm of the cross-shaped room |

```bash
ros2 launch side_tendency_layer sim_office.launch.py
ros2 launch side_tendency_layer sim_office.launch.py side_tendency:=false   # baseline
```

Once Nav2 is up, `scripts/demo_goal.py` drives the demo route and logs the remaining distance.
All launch files take the same arguments, and the scenario's values can be overridden:

| Argument | Effect |
|---|---|
| `side_tendency:=false` | Baseline: the same parameters without the layer |
| `demo_goal:=false` | No demo route, e.g. for `corridor_trial.py` or goals from RViz |
| `goals:="x y yaw; x y yaw"` | Demo route in the map frame |
| `headless:=True use_rviz:=False` | No windows |
| `x_pose:=`, `y_pose:=`, `yaw:=` | Spawn pose in the Gazebo world |
| `initial_x:=`, `initial_y:=`, `initial_yaw:=` | AMCL initial pose in the map frame, by default the spawn pose |
| `params_file:=` | Other Nav2 parameters |

The maps of `straight_corridor`, `doorway`, `intersection` and `corridor_turns` were recorded with
SLAM from the spawn pose. Their map frame is therefore the spawn pose: AMCL starts at the map
origin and the robot faces along the map's x-axis. The other maps are in the world frame.

The scenario files include `simulation.launch.py`, which also runs your own world and map:

```bash
ros2 launch side_tendency_layer simulation.launch.py world:=/path/to/my.world \
  map:=/path/to/my.yaml x_pose:=1.0 y_pose:=2.0 goals:="10 2 0"
```

### Trials

`scripts/corridor_trial.py` drives from start to goal, records the path and prints the lateral
offset. Launch with `demo_goal:=false`, so that two goals do not compete. Start and goal are in
the map frame:

| Scenario | Start | Goal |
|---|---|---|
| `laboratory` | `-17 -3.45 0` | `7 -3.45 0` |
| `straight_corridor`, `doorway`, `intersection` | `0 0 0` | `17 0 0` |
| `corridor_turns` | `0 0 0` | `6.5 0 0` |

```bash
ros2 launch side_tendency_layer sim_doorway.launch.py demo_goal:=false
ros2 run side_tendency_layer corridor_trial.py --start 0 0 0 --goal 17 0 0 --out st.csv
```

Lateral offset over the middle 80 % of the way, negative = right of the travel direction:

| Scenario | Corridor width | Without layer | With layer |
|---|---|---|---|
| `laboratory` | 2.9 m | -0.05 m | -0.85 m |
| `straight_corridor` | 2.85 m | -0.10 m | -0.82 m |
| `doorway` | 2.85 m | -0.07 m | -0.68 m |
| `intersection` | 2.85 m | -0.12 m | -0.87 m |
| `corridor_turns`, first leg | 1.73 m | -0.04 m | -0.36 m |

The TurtleBot config sets `corridor_min_width` to 1.5 m, so that the layer also acts in the
narrow legs of `corridor_turns`. With `perception_range` 1.0 m and the Nav2 default DWB settings,
the robot needed about 11 m of driving to complete the side shift in the laboratory. With 1.5 m it
needs under 2 m.

### Troubleshooting

If `spawn_entity.py` fails with `No module named 'lxml'`, the `python3` first on the `PATH`, for
example one from pyenv, lacks lxml. Start with the system Python first, `PATH=/usr/bin:$PATH ros2
launch ...`, or install lxml into that Python.

## Citation

If you use this work, please cite our paper:

```bibtex
@inproceedings{kalenberg2025sidetendency,
  author    = {Kalenberg, Matthias and Li, Song and Engelhardt, Helmut and Walter, Jonas and Franke, J{\"o}rg},
  title     = {Side Tendency Layer: Enhancing Social Robot Navigation for Passenger and Pedestrian
               Comfort Using Local Sensor Data},
  booktitle = {2025 European Conference on Mobile Robots (ECMR)},
  year      = {2025},
  doi       = {10.1109/ECMR65884.2025.11163026}
}
```

## License and acknowledgement

MIT License, see [LICENSE](LICENSE). The Gazebo models in `models/` are licensed under
CC BY 3.0, see [models/README.md](models/README.md).

This work was supported by the Federal Ministry of Education and Research (BMBF) under funding
code 16SV9306.
