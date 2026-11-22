#include "gecko_rviz_panel/joystick_widget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

#include "gecko_rviz_panel/drive_mapping.hpp"

namespace gecko_rviz_panel
{

namespace
{
constexpr double KNOB_RATIO = 0.32;

void drawArrow(
  QPainter & painter, const QPalette & palette, double angle, double distance, double size,
  bool active)
{
  painter.save();
  painter.rotate(angle);
  QPainterPath arrow;
  arrow.moveTo(0.0, -distance - size);
  arrow.lineTo(size, -distance);
  arrow.lineTo(-size, -distance);
  arrow.closeSubpath();
  painter.fillPath(arrow, active ? palette.highlight() : palette.mid());
  painter.restore();
}
}  // namespace

JoystickWidget::JoystickWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(150, 150);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setCursor(Qt::OpenHandCursor);
}

double JoystickWidget::radius() const
{
  return std::min(width(), height()) / 2.0 - 4.0;
}

void JoystickWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.translate(width() / 2.0, height() / 2.0);

  const double r = radius();
  const double travel = r * (1.0 - KNOB_RATIO);
  const int drive = driveFromVector(stick_.x(), stick_.y());

  painter.setPen(QPen(palette().mid(), 2.0));
  painter.setBrush(palette().base());
  painter.drawEllipse(QPointF(0.0, 0.0), r, r);

  painter.setPen(Qt::NoPen);
  const double arrow = r * 0.09;
  const double distance = r * 0.72;
  drawArrow(painter, palette(), 0.0, distance, arrow, drive == DRIVE_FORWARD);
  drawArrow(painter, palette(), 90.0, distance, arrow, drive == DRIVE_RIGHT);
  drawArrow(painter, palette(), 180.0, distance, arrow, drive == DRIVE_BACK);
  drawArrow(painter, palette(), 270.0, distance, arrow, drive == DRIVE_LEFT);

  const QPointF knob(stick_.x() * travel, -stick_.y() * travel);
  painter.setPen(QPen(palette().dark(), 1.5));
  painter.setBrush(drive == DRIVE_STOP ? palette().button() : palette().highlight());
  painter.drawEllipse(knob, r * KNOB_RATIO, r * KNOB_RATIO);
}

void JoystickWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    setCursor(Qt::ClosedHandCursor);
    moveStick(event->localPos());
  }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (event->buttons() & Qt::LeftButton) {
    moveStick(event->localPos());
  }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    setCursor(Qt::OpenHandCursor);
    recenter();
  }
}

void JoystickWidget::hideEvent(QHideEvent * event)
{
  recenter();
  QWidget::hideEvent(event);
}

void JoystickWidget::moveStick(const QPointF & position)
{
  const double travel = radius() * (1.0 - KNOB_RATIO);
  double x = (position.x() - width() / 2.0) / travel;
  double y = (height() / 2.0 - position.y()) / travel;
  const double magnitude = std::hypot(x, y);
  if (magnitude > 1.0) {
    x /= magnitude;
    y /= magnitude;
  }
  stick_ = QPointF(x, y);
  update();
  Q_EMIT vectorChanged(x, y);
}

void JoystickWidget::recenter()
{
  stick_ = QPointF(0.0, 0.0);
  update();
  Q_EMIT vectorChanged(0.0, 0.0);
}

}  // namespace gecko_rviz_panel
