# ROS2 Pneumatic Actuation

ROS2 package for the pneumatic actuation of continuum soft robot arms.

## Usage

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

## Conventions
We are using a chamber numbering convention analogue to the parametrized lengths L_i referred to in [[1]](#1).
This means that for a four chamber segment:
1. **Chamber 1:** points along the positive local x-axis
2. **Chamber 2:** points along the negative local x-axis, e.g. opposite of chamber 2.
3. **Chamber 3:** points along the positive local y-axis
4. **Chamber 4:** points along the negative local y-axis, e.g. opposite of chamber 3.
5. The positive z-axis is in a right-handed coordinate system to the x- and y-axis and points along the distal end of the robotic arm.

![Chamber numbering convention used in this repository for the four-chamber per segment case. The figure is adapted from [[1]](#1).](figures/convention_four_chambers.png)

## Citations
<a id="1">[1]</a> Della Santina, C., Bicchi, A., & Rus, D. (2019, November). Dynamic control of soft robots with internal constraints in the presence of obstacles. In 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 6622-6629). IEEE.