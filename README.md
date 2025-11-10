## Setup

The Python examples assume you've cloned the [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) repo with the URDF files.

Instead of using the 6DoF of the phone to control the gripper tip, in this project we use the phone xyz position to control the tip of the lower arm and the phone pitch/roll to control the wrist flex/roll. Add the following frame to your SO101 URDF file:

```
<!-- Lower arm frame (dummy link + fixed joint) -->
<link name="lower_arm_frame_link">
  <origin xyz="0 0 0" rpy="0 -0 0"/>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1e-9"/>
    <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
  </inertial>
</link>

<joint name="lower_arm_frame_joint" type="fixed">
  <origin xyz="-0.1349 0.0052 0.015" rpy="1.5708 0 -1.5708"/>
  <parent link="lower_arm_link"/>
  <child link="lower_arm_frame_link"/>
  <axis xyz="0 0 0"/>
</joint>
```
