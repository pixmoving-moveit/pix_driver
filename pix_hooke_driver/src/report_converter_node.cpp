#include <pix_hooke_driver/report_converter.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pix_hooke_driver::report_converter::ReportConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}