#ifndef GECKO_RVIZ_PANEL__JOYSTICK_WIDGET_HPP_
#define GECKO_RVIZ_PANEL__JOYSTICK_WIDGET_HPP_

#include <QPointF>
#include <QWidget>

namespace gecko_rviz_panel
{

// Spring-return joystick: it reports to the centre as soon as it is released.
class JoystickWidget : public QWidget
{
  Q_OBJECT

public:
  explicit JoystickWidget(QWidget * parent = nullptr);

Q_SIGNALS:
  void vectorChanged(double x, double y);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void hideEvent(QHideEvent * event) override;

private:
  double radius() const;
  void moveStick(const QPointF & position);
  void recenter();

  QPointF stick_{0.0, 0.0};  // Normalized; y grows upwards.
};

}  // namespace gecko_rviz_panel

#endif  // GECKO_RVIZ_PANEL__JOYSTICK_WIDGET_HPP_
