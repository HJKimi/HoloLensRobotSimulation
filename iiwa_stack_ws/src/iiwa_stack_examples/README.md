# iiwa_tool

- **iiwa_tool_description** : a URDF for a KUKA LBR IIWA robot with a tool attached and a rigid base.
- **iiwa_tool_moveit** : a MoveIt! package to work with the robot description just defined.
- **iiwa_tool_examples** : small ROS nodes to show basic usage of the robot described above.

In **iiwa_tool_description**/urdf/iiwa_tool.urdf.xacro,
The coordinate frame of the base of the tool is changed like below.

    <joint name="tool_joint" type="fixed">
      <parent link="iiwa_link_ee" />
      <child link = "tool_link" />
      <!-- <origin xyz="${tool_joint_offset}" rpy="0 ${PI/2.0} 0" />  -->
      <origin xyz="${tool_joint_offset}" rpy="0 0 0" />
    </joint>

