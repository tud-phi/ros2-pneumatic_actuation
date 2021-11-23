# ROS2 Pneumatic Actuation

ROS2 package for the pneumatic actuation of continuum soft robot arms.

## Commands

### Build
```bash
colcon build --packages-select pneumatic_actuation_demos
```

### Run pressure trajectory node
```bash
ros2 run pneumatic_actuation_demos pressure_trajectory_node
```

### Run with launch file

```bash
ros2 launch pneumatic_actuation_demos pressure_trajectory.launch.py
```
