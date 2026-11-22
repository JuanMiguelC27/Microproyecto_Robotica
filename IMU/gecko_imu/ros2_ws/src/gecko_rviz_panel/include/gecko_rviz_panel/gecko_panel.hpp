#ifndef GECKO_RVIZ_PANEL__GECKO_PANEL_HPP_
#define GECKO_RVIZ_PANEL__GECKO_PANEL_HPP_

#include <QDial>
#include <QLabel>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>

#include "gecko_rviz_panel/joystick_widget.hpp"

class QGroupBox;

namespace gecko_rviz_panel
{

class GeckoPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GeckoPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onJoystick(double x, double y);
  void publishDrive();
  void onServo(int degrees);
  void onTurbineF(int percent);
  void onTurbineR(int percent);

private:
  QGroupBox * makeKnob(const QString & title, QDial *& dial, QLabel *& label);
  void applyInitialControls(double servo_rad, double turbine_f, double turbine_r);
  void showServo(int degrees);
  void publishFloat(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher, double value);

  JoystickWidget * joystick_;
  QLabel * drive_label_;
  QSlider * servo_slider_;
  QLabel * servo_label_;
  QDial * turbine_f_dial_;
  QLabel * turbine_f_label_;
  QDial * turbine_r_dial_;
  QLabel * turbine_r_label_;
  QTimer * drive_timer_;

  int drive_{0};
  bool controls_synced_{false};

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turbine_f_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turbine_r_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr controls_sub_;
};

}  // namespace gecko_rviz_panel

#endif  // GECKO_RVIZ_PANEL__GECKO_PANEL_HPP_
