from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='pix_hooke_driver',
      executable='pix_hooke_driver_control_command_node',
      name='pix_hooke_driver_command_node',
      emulate_tty=True
    ),
    Node(
      package='pix_hooke_driver',
      executable='pix_hooke_driver_report_parser_node',
      name='pix_hooke_driver_report_node',
      emulate_tty=True
    )
  ])