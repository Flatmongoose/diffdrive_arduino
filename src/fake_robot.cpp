#include "diffdrive_arduino/fake_robot.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

hardware_interface::CallbackReturn FakeRobot::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("FakeRobot"), "Configuring...");

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];


  // Set up the wheels
  // Note: It doesn't matter that we haven't set encoder counts per rev
  // since the fake robot bypasses the encoder code completely

  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(rclcpp::get_logger("FakeRobot"), "Finished Configuration");

  //status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn FakeRobot::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeRobot"), "Starting Controller...");
  //status_ = hardware_interface::status::STARTED;
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobot::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeRobot"), "Stopping Controller...");
  //status_ = hardware_interface::status::STOPPED;
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FakeRobot::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Force the wheel position
  l_wheel_.pos = l_wheel_.pos + l_wheel_.vel * period.seconds();
  r_wheel_.pos = r_wheel_.pos + r_wheel_.vel * period.seconds();

  return hardware_interface::return_type::OK;

  
}

hardware_interface::return_type FakeRobot::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // Set the wheel velocities to directly match what is commanded

  l_wheel_.vel = l_wheel_.cmd;
  r_wheel_.vel = r_wheel_.cmd;


  return hardware_interface::return_type::OK;  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)