#include <pix_hooke_driver/control_converter.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pix_hooke_driver::control_converter::ControlConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}