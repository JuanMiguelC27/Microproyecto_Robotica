#include "gecko_rviz_panel/gecko_panel.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QMetaObject>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "gecko_rviz_panel/drive_mapping.hpp"

namespace gecko_rviz_panel
{

namespace
{
constexpr int SERVO_LIMIT_DEG = 45;
constexpr int TURBINE_LIMIT_PERCENT = 100;
constexpr int DRIVE_PERIOD_MS = 100;
constexpr double DEG_TO_RAD = M_PI / 180.0;

QString driveName(int drive)
{
  switch (drive) {
    case DRIVE_FORWARD: return QStringLiteral("Adelante");
    case DRIVE_BACK: return QStringLiteral("Atrás");
    case DRIVE_RIGHT: return QStringLiteral("Derecha");
    case DRIVE_LEFT: return QStringLiteral("Izquierda");
    default: return QStringLiteral("Detenido");
  }
}

QString percentText(int percent)
{
  return QStringLiteral("%1%2 %").arg(percent > 0 ? "+" : "").arg(percent);
}
}  // namespace

GeckoPanel::GeckoPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * joystick_box = new QGroupBox(QStringLiteral("Ruedas"));
  auto * joystick_layout = new QVBoxLayout(joystick_box);
  joystick_ = new JoystickWidget;
  drive_label_ = new QLabel(driveName(DRIVE_STOP));
  drive_label_->setAlignment(Qt::AlignCenter);
  joystick_layout->addWidget(joystick_, 1);
  joystick_layout->addWidget(drive_label_);

  auto * servo_box = new QGroupBox(QStringLiteral("Servos"));
  auto * servo_layout = new QVBoxLayout(servo_box);
  servo_slider_ = new QSlider(Qt::Vertical);
  servo_slider_->setRange(-SERVO_LIMIT_DEG, SERVO_LIMIT_DEG);
  servo_slider_->setTickPosition(QSlider::TicksBothSides);
  servo_slider_->setTickInterval(15);
  servo_slider_->setPageStep(5);
  servo_slider_->setMinimumHeight(120);
  servo_label_ = new QLabel;
  servo_label_->setAlignment(Qt::AlignCenter);
  auto * servo_zero = new QPushButton(QStringLiteral("0°"));
  servo_layout->addWidget(servo_slider_, 1, Qt::AlignHCenter);
  servo_layout->addWidget(servo_label_);
  servo_layout->addWidget(servo_zero);
  showServo(0);

  // Two rows keep the panel narrow enough for RViz's side dock.
  auto * layout = new QGridLayout(this);
  layout->addWidget(joystick_box, 0, 0);
  layout->addWidget(servo_box, 0, 1);
  layout->addWidget(makeKnob(QStringLiteral("Turbina F"), turbine_f_dial_, turbine_f_label_), 1, 0);
  layout->addWidget(makeKnob(QStringLiteral("Turbina R"), turbine_r_dial_, turbine_r_label_), 1, 1);
  layout->setColumnStretch(0, 3);
  layout->setColumnStretch(1, 2);
  layout->setRowStretch(0, 3);
  layout->setRowStretch(1, 2);

  drive_timer_ = new QTimer(this);

  connect(joystick_, &JoystickWidget::vectorChanged, this, &GeckoPanel::onJoystick);
  connect(drive_timer_, &QTimer::timeout, this, &GeckoPanel::publishDrive);
  connect(servo_slider_, &QSlider::valueChanged, this, &GeckoPanel::onServo);
  connect(servo_zero, &QPushButton::clicked, this, [this]() {servo_slider_->setValue(0);});
  connect(turbine_f_dial_, &QDial::valueChanged, this, &GeckoPanel::onTurbineF);
  connect(turbine_r_dial_, &QDial::valueChanged, this, &GeckoPanel::onTurbineR);
}

QGroupBox * GeckoPanel::makeKnob(const QString & title, QDial *& dial, QLabel *& label)
{
  auto * box = new QGroupBox(title);
  auto * box_layout = new QVBoxLayout(box);
  dial = new QDial;
  dial->setRange(-TURBINE_LIMIT_PERCENT, TURBINE_LIMIT_PERCENT);
  dial->setNotchesVisible(true);
  dial->setNotchTarget(10.0);
  dial->setPageStep(5);
  dial->setMinimumSize(90, 90);
  label = new QLabel(percentText(0));
  label->setAlignment(Qt::AlignCenter);
  auto * zero = new QPushButton(QStringLiteral("0"));
  box_layout->addWidget(dial, 1);
  box_layout->addWidget(label);
  box_layout->addWidget(zero);

  QDial * target = dial;
  connect(zero, &QPushButton::clicked, this, [target]() {target->setValue(0);});
  return box;
}

void GeckoPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  drive_pub_ = node->create_publisher<std_msgs::msg::Int8>("/gecko/cmd/drive", 10);
  servo_pub_ = node->create_publisher<std_msgs::msg::Float64>("/gecko/cmd/servo_angle", 10);
  turbine_f_pub_ = node->create_publisher<std_msgs::msg::Float64>("/gecko/cmd/turbine_f", 10);
  turbine_r_pub_ = node->create_publisher<std_msgs::msg::Float64>("/gecko/cmd/turbine_r", 10);

  // The twin keeps servo and turbine values while RViz is closed.
  controls_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/gecko/state/controls", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
      if (msg->data.size() != 3) {
        return;
      }
      const double servo = msg->data[0];
      const double turbine_f = msg->data[1];
      const double turbine_r = msg->data[2];
      QMetaObject::invokeMethod(
        this, [this, servo, turbine_f, turbine_r]() {
          applyInitialControls(servo, turbine_f, turbine_r);
        }, Qt::QueuedConnection);
    });

  drive_timer_->start(DRIVE_PERIOD_MS);
}

void GeckoPanel::applyInitialControls(double servo_rad, double turbine_f, double turbine_r)
{
  // Only the first state is applied, so the twin's echo never fights the user.
  if (controls_synced_) {
    return;
  }
  controls_synced_ = true;

  const int degrees = static_cast<int>(std::lround(servo_rad / DEG_TO_RAD));
  const int percent_f = static_cast<int>(std::lround(turbine_f * TURBINE_LIMIT_PERCENT));
  const int percent_r = static_cast<int>(std::lround(turbine_r * TURBINE_LIMIT_PERCENT));

  const QSignalBlocker block_servo(servo_slider_);
  const QSignalBlocker block_f(turbine_f_dial_);
  const QSignalBlocker block_r(turbine_r_dial_);
  servo_slider_->setValue(degrees);
  turbine_f_dial_->setValue(percent_f);
  turbine_r_dial_->setValue(percent_r);
  showServo(degrees);
  turbine_f_label_->setText(percentText(percent_f));
  turbine_r_label_->setText(percentText(percent_r));
}

void GeckoPanel::onJoystick(double x, double y)
{
  drive_ = driveFromVector(x, y);
  drive_label_->setText(driveName(drive_));
  publishDrive();
}

void GeckoPanel::publishDrive()
{
  if (drive_pub_) {
    std_msgs::msg::Int8 msg;
    msg.data = static_cast<int8_t>(drive_);
    drive_pub_->publish(msg);
  }
}

void GeckoPanel::showServo(int degrees)
{
  servo_label_->setText(QStringLiteral("%1%2°").arg(degrees > 0 ? "+" : "").arg(degrees));
}

void GeckoPanel::onServo(int degrees)
{
  controls_synced_ = true;
  showServo(degrees);
  publishFloat(servo_pub_, degrees * DEG_TO_RAD);
}

void GeckoPanel::onTurbineF(int percent)
{
  controls_synced_ = true;
  turbine_f_label_->setText(percentText(percent));
  publishFloat(turbine_f_pub_, static_cast<double>(percent) / TURBINE_LIMIT_PERCENT);
}

void GeckoPanel::onTurbineR(int percent)
{
  controls_synced_ = true;
  turbine_r_label_->setText(percentText(percent));
  publishFloat(turbine_r_pub_, static_cast<double>(percent) / TURBINE_LIMIT_PERCENT);
}

void GeckoPanel::publishFloat(
  const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher, double value)
{
  if (publisher) {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    publisher->publish(msg);
  }
}

}  // namespace gecko_rviz_panel

PLUGINLIB_EXPORT_CLASS(gecko_rviz_panel::GeckoPanel, rviz_common::Panel)
