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
ros2 launch pneumatic_actuation_demos default.launch.py
```

## Conventions
### Chambers
We are using a chamber numbering convention analogue to the parametrized lengths L_i referred to in [[1]](#1).
This means that for a four chamber segment:
1. **Chamber 1:** points along the positive local x-axis
2. **Chamber 2:** points along the negative local x-axis, e.g. opposite of chamber 2.
3. **Chamber 3:** points along the positive local y-axis
4. **Chamber 4:** points along the negative local y-axis, e.g. opposite of chamber 3.
5. The positive z-axis is in a right-handed coordinate system to the x- and y-axis and points along the distal end of the robotic arm.

![Chamber numbering convention used in this repository for the four-chamber per segment case. The figure is adapted from [[1]](#1).](figures/convention_four_chambers.png)
### Pressure torque
We define pressure torques as wrenches produced by perpendicular forces acting on the tip plane of the segment at the Center of Pressure (CoP), which is displaced at a distance of `r_p` from the center-line representing the lever arm of the torque.
For a four chamber segment, we remind ourselves of the following convention:
1. When we apply a positive pressure torque in x-direction, the robot should be bending with a positive angle around the x-axis. This means, that we have to increase the pressure in chamber 3 and decrease the pressure in chamber 4. The robot will have negative local y-position coordinates.
2. When we apply a positive pressure torque in y-direction, the robot should be bending with a positive angle around the y-axis. This means, that we have to increase the pressure in chamber 2 and decrease the pressure in chamber 1. The robot will have positive local x-position coordinates.

![Convention for pressure torques acting on the segment. `n` represents the x-axis and `e` the y-axis.](figures/torques_on_segment.png)

## Citations
<a id="1">[1]</a> Della Santina, C., Bicchi, A., & Rus, D. (2019, November). Dynamic control of soft robots with internal constraints in the presence of obstacles. In 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 6622-6629). IEEE.