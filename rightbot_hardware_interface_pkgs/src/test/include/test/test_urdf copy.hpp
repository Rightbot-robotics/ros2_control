<ros2_control name="${name}" type="system">
      <hardware>
        <plugin>ros2_control_demo_example_1/RRBotSystemPositionOnlyHardware</plugin>
        <param name="example_param_hw_start_duration_sec">0</param>
        <param name="example_param_hw_stop_duration_sec">3.0</param>
        <param name="example_param_hw_slowdown">100</param>
      </hardware>

      <joint name="${prefix}joint1">
        <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
      </joint>
      <joint name="${prefix}joint2">
        <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
      </joint>
    </ros2_control>
