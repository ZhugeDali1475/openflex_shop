#include "openarmx_battery_monitor/battery_overlay_display.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <QColor>
#include <QEvent>
#include <QFont>
#include <QFontMetrics>
#include <QMetaObject>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QRect>
#include <QWidget>

#include <pluginlib/class_list_macros.hpp>

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"

namespace openarmx_battery_monitor
{

namespace
{

constexpr int kWidgetPaddingPx = 2;
constexpr int kLabelGapPx = 4;

std::uint16_t modbusCrc16(const std::vector<std::uint8_t> & payload)
{
  std::uint16_t crc = 0xFFFF;
  for (const std::uint8_t item : payload) {
    crc ^= item;
    for (int i = 0; i < 8; ++i) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<std::uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

std::uint16_t modbusCrc16(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<std::uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

speed_t baudToTermios(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      throw std::runtime_error("Unsupported baudrate");
  }
}

std::vector<std::string> splitCsv(const std::string & text)
{
  std::vector<std::string> items;
  std::string current;
  for (const char ch : text) {
    if (ch == ',') {
      if (!current.empty()) {
        items.push_back(current);
      }
      current.clear();
    } else if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r') {
      current.push_back(ch);
    }
  }
  if (!current.empty()) {
    items.push_back(current);
  }
  return items;
}

}  // namespace

class BatteryOverlayDisplay::SerialModbusRtuClient
{
public:
  SerialModbusRtuClient(std::string port, int baudrate, float timeout_sec)
  : port_(std::move(port)), baudrate_(baudrate), timeout_sec_(timeout_sec)
  {
  }

  ~SerialModbusRtuClient()
  {
    close();
  }

  const std::string & port() const
  {
    return port_;
  }

  void setPort(const std::string & port)
  {
    if (port == port_) {
      return;
    }
    close();
    port_ = port;
  }

  void open()
  {
    if (fd_ >= 0) {
      return;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(std::string("Failed to open serial port: ") + std::strerror(errno));
    }

    termios attrs{};
    if (tcgetattr(fd_, &attrs) != 0) {
      close();
      throw std::runtime_error(std::string("tcgetattr failed: ") + std::strerror(errno));
    }

    attrs.c_iflag = 0;
    attrs.c_oflag = 0;
    attrs.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
    attrs.c_cflag |= CS8 | CLOCAL | CREAD;
    attrs.c_lflag = 0;
    const speed_t baud = baudToTermios(baudrate_);
    cfsetispeed(&attrs, baud);
    cfsetospeed(&attrs, baud);
    attrs.c_cc[VMIN] = 0;
    attrs.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &attrs) != 0) {
      close();
      throw std::runtime_error(std::string("tcsetattr failed: ") + std::strerror(errno));
    }
    tcflush(fd_, TCIOFLUSH);
  }

  void close()
  {
    if (fd_ < 0) {
      return;
    }
    ::close(fd_);
    fd_ = -1;
  }

  std::vector<std::uint16_t> readHoldingRegisters(int slave_id, int start_addr, int register_count)
  {
    open();
    tcflush(fd_, TCIFLUSH);

    std::vector<std::uint8_t> request{
      static_cast<std::uint8_t>(slave_id),
      0x03,
      static_cast<std::uint8_t>((start_addr >> 8) & 0xFF),
      static_cast<std::uint8_t>(start_addr & 0xFF),
      static_cast<std::uint8_t>((register_count >> 8) & 0xFF),
      static_cast<std::uint8_t>(register_count & 0xFF)};
    const std::uint16_t crc = modbusCrc16(request);
    request.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    request.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));

    writeAll(request);
    const std::vector<std::uint8_t> frame = readResponseFrame(slave_id, register_count * 2);
    std::vector<std::uint16_t> values;
    values.reserve(static_cast<std::size_t>(register_count));
    for (int i = 0; i < register_count; ++i) {
      const std::size_t offset = static_cast<std::size_t>(3 + i * 2);
      values.push_back(static_cast<std::uint16_t>((frame[offset] << 8) | frame[offset + 1]));
    }
    return values;
  }

private:
  void writeAll(const std::vector<std::uint8_t> & payload)
  {
    std::size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(timeout_sec_));
    while (offset < payload.size()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        throw std::runtime_error("Timeout while writing Modbus request");
      }
      const auto remaining = std::chrono::duration<float>(deadline - now).count();
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(fd_, &wfds);
      timeval tv{};
      tv.tv_sec = static_cast<int>(remaining);
      tv.tv_usec = static_cast<int>((remaining - tv.tv_sec) * 1000000.0f);
      const int ret = select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
      if (ret <= 0) {
        continue;
      }
      const ssize_t written = ::write(fd_, payload.data() + offset, payload.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
      }
    }
    tcdrain(fd_);
  }

  std::vector<std::uint8_t> readAvailable(float timeout_sec)
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    timeval tv{};
    tv.tv_sec = static_cast<int>(timeout_sec);
    tv.tv_usec = static_cast<int>((timeout_sec - tv.tv_sec) * 1000000.0f);
    const int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      return {};
    }
    std::vector<std::uint8_t> buffer(256);
    const ssize_t count = ::read(fd_, buffer.data(), buffer.size());
    if (count <= 0) {
      return {};
    }
    buffer.resize(static_cast<std::size_t>(count));
    return buffer;
  }

  std::vector<std::uint8_t> readResponseFrame(int slave_id, int expected_byte_count)
  {
    std::vector<std::uint8_t> buffer;
    const auto hard_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(timeout_sec_ * 3.0f));
    auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(timeout_sec_));
    const float inter_byte_timeout = std::max(0.05f, std::min(0.35f, timeout_sec_ * 1.5f));

    while (std::chrono::steady_clock::now() < hard_deadline) {
      std::vector<std::uint8_t> frame;
      if (extractFrame(buffer, slave_id, expected_byte_count, frame)) {
        return frame;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        if (buffer.empty()) {
          break;
        }
        deadline = std::min(
          hard_deadline,
          now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<float>(inter_byte_timeout)));
      }
      const auto remaining = std::chrono::duration<float>(deadline - now).count();
      if (remaining <= 0.0f) {
        continue;
      }
      auto chunk = readAvailable(remaining);
      if (!chunk.empty()) {
        buffer.insert(buffer.end(), chunk.begin(), chunk.end());
        deadline = std::min(
          hard_deadline,
          std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<float>(inter_byte_timeout)));
        if (buffer.size() > 4096) {
          buffer.erase(buffer.begin(), buffer.end() - 512);
        }
      }
    }

    throw std::runtime_error("Timeout waiting for Modbus response");
  }

  bool extractFrame(
    std::vector<std::uint8_t> & buffer,
    int slave_id,
    int expected_byte_count,
    std::vector<std::uint8_t> & frame)
  {
    while (buffer.size() >= 5) {
      auto it = std::find(buffer.begin(), buffer.end(), static_cast<std::uint8_t>(slave_id));
      if (it == buffer.end()) {
        buffer.clear();
        return false;
      }
      if (it != buffer.begin()) {
        buffer.erase(buffer.begin(), it);
      }
      if (buffer.size() < 5) {
        return false;
      }

      if (buffer[1] != 0x03) {
        buffer.erase(buffer.begin());
        continue;
      }
      if (buffer[2] != expected_byte_count) {
        buffer.erase(buffer.begin());
        continue;
      }
      const std::size_t frame_len = static_cast<std::size_t>(3 + expected_byte_count + 2);
      if (buffer.size() < frame_len) {
        return false;
      }
      const std::uint16_t expected_crc = modbusCrc16(buffer.data(), frame_len - 2);
      const std::uint16_t received_crc = static_cast<std::uint16_t>(
        buffer[frame_len - 2] | (buffer[frame_len - 1] << 8));
      if (expected_crc == received_crc) {
        frame.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(frame_len));
        buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(frame_len));
        return true;
      }
      buffer.erase(buffer.begin());
    }
    return false;
  }

  std::string port_;
  int baudrate_;
  float timeout_sec_;
  int fd_{-1};
};

namespace
{

QColor batteryColor(bool online, float percentage)
{
  if (!online) {
    return QColor(130, 137, 145);
  }
  if (percentage >= 0.60f) {
    return QColor(74, 184, 108);
  }
  if (percentage >= 0.30f) {
    return QColor(232, 193, 70);
  }
  return QColor(220, 82, 71);
}

QString percentageText(bool online, float percentage)
{
  if (!online || !std::isfinite(percentage)) {
    return QStringLiteral("--");
  }
  const int rounded = std::clamp(static_cast<int>(std::lround(percentage * 100.0f)), 0, 100);
  return QString::number(rounded) + "%";
}

}  // namespace

class BatteryOverlayWidget : public QWidget
{
public:
  struct Visual
  {
    bool online{false};
    float percentage{0.0f};
    QString label;
    int battery_width{92};
    int battery_height{36};
    int font_size{16};
  };

  explicit BatteryOverlayWidget(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::Widget)
  : QWidget(parent, flags)
  {
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAttribute(Qt::WA_ShowWithoutActivating, true);
    setAutoFillBackground(false);
    setFocusPolicy(Qt::NoFocus);
  }

  void setVisual(const Visual & visual)
  {
    visual_ = visual;
    setFixedSize(desiredSize());
    update();
  }

  QSize desiredSize() const
  {
    QFont battery_font = font();
    battery_font.setPointSize(std::max(8, visual_.font_size));

    QFont label_font = battery_font;
    label_font.setPointSize(std::max(8, visual_.font_size - 2));
    QFontMetrics label_metrics(label_font);

    const int label_height = visual_.label.isEmpty() ? 0 : label_metrics.height();
    const int label_width = visual_.label.isEmpty() ? 0 : label_metrics.horizontalAdvance(visual_.label);
    const int width = std::max(visual_.battery_width, label_width) + (kWidgetPaddingPx * 2);
    const int height = visual_.battery_height +
      (visual_.label.isEmpty() ? 0 : label_height + kLabelGapPx) +
      (kWidgetPaddingPx * 2);
    return QSize(width, height);
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    (void)event;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QFont battery_font = font();
    battery_font.setPointSize(std::max(8, visual_.font_size));
    painter.setFont(battery_font);

    QFont label_font = battery_font;
    label_font.setPointSize(std::max(8, visual_.font_size - 2));
    QFontMetrics label_metrics(label_font);

    const int label_height = visual_.label.isEmpty() ? 0 : label_metrics.height();
    const int content_x = width() - kWidgetPaddingPx - visual_.battery_width;
    const int content_y = kWidgetPaddingPx + (visual_.label.isEmpty() ? 0 : label_height + kLabelGapPx);

    if (!visual_.label.isEmpty()) {
      painter.setFont(label_font);
      painter.setPen(QColor(240, 243, 248));
      painter.drawText(
        QRect(kWidgetPaddingPx, kWidgetPaddingPx, width() - (kWidgetPaddingPx * 2), label_height),
        Qt::AlignHCenter | Qt::AlignVCenter,
        visual_.label
      );
      painter.setFont(battery_font);
    }

    const int cap_width = std::max(6, visual_.battery_width / 10);
    const int cap_gap = 2;
    const int body_width = std::max(10, visual_.battery_width - cap_width - cap_gap);

    QRectF body_rect(content_x, content_y, body_width, visual_.battery_height);
    QRectF cap_rect(
      content_x + body_width + cap_gap,
      content_y + static_cast<int>(visual_.battery_height * 0.28f),
      std::max(3, cap_width - 1),
      std::max(6, static_cast<int>(visual_.battery_height * 0.44f))
    );

    const QColor accent = batteryColor(visual_.online, visual_.percentage);
    const QColor shell = visual_.online ? QColor(22, 26, 31, 220) : QColor(32, 36, 40, 180);
    const QColor fill = QColor(accent.red(), accent.green(), accent.blue(), visual_.online ? 235 : 120);

    painter.setPen(QPen(accent, 2.0));
    painter.setBrush(shell);
    painter.drawRoundedRect(body_rect, 7.0, 7.0);
    painter.drawRoundedRect(cap_rect, 2.0, 2.0);

    QRectF inner_rect = body_rect.adjusted(3.0, 3.0, -3.0, -3.0);
    if (visual_.online) {
      const float clamped = std::clamp(visual_.percentage, 0.0f, 1.0f);
      QRectF fill_rect = inner_rect;
      fill_rect.setWidth(inner_rect.width() * clamped);
      if (fill_rect.width() > 1.0f) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(fill);
        painter.drawRoundedRect(fill_rect, 5.0, 5.0);
      }
    }

    const QString text = percentageText(visual_.online, visual_.percentage);
    const QRect text_rect = body_rect.adjusted(4, 0, -4, 0).toRect();
    painter.setPen(QColor(0, 0, 0, 170));
    painter.drawText(text_rect.translated(1, 1), Qt::AlignCenter, text);
    painter.setPen(QColor(250, 252, 255));
    painter.drawText(text_rect, Qt::AlignCenter, text);
  }

private:
  Visual visual_;
};

BatteryOverlayDisplay::BatteryOverlayDisplay()
{
  topic_property_->setName("Battery Topic");
  topic_property_->setDescription("sensor_msgs/BatteryState topic to display.");

  label_property_ = new rviz_common::properties::StringProperty(
    "Label",
    "",
    "Optional label shown above the battery icon.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  use_message_location_property_ = new rviz_common::properties::BoolProperty(
    "Use Message Location",
    false,
    "Use BatteryState.location when Label is empty.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  hide_when_offline_property_ = new rviz_common::properties::BoolProperty(
    "Hide When Offline",
    true,
    "Hide the overlay when the battery is offline or stale.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  margin_x_property_ = new rviz_common::properties::IntProperty(
    "Margin X",
    24,
    "Right margin in pixels.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    0,
    4096
  );
  margin_y_property_ = new rviz_common::properties::IntProperty(
    "Margin Y",
    20,
    "Top margin in pixels.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    0,
    4096
  );
  battery_width_property_ = new rviz_common::properties::IntProperty(
    "Battery Width",
    92,
    "Battery icon width in pixels.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    32,
    4096
  );
  battery_height_property_ = new rviz_common::properties::IntProperty(
    "Battery Height",
    36,
    "Battery icon height in pixels.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    18,
    4096
  );
  font_size_property_ = new rviz_common::properties::IntProperty(
    "Font Size",
    16,
    "Font size in points.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    8,
    256
  );
  offline_timeout_property_ = new rviz_common::properties::FloatProperty(
    "Offline Timeout (s)",
    5.0f,
    "Hide or mark offline when no fresh message arrives within this interval.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  offline_timeout_property_->setMin(0.0f);

  direct_serial_enabled_property_ = new rviz_common::properties::BoolProperty(
    "Direct Serial Read",
    true,
    "Open the serial port directly when this display is enabled.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  direct_serial_port_property_ = new rviz_common::properties::StringProperty(
    "Serial Port",
    "/dev/ttyACM0",
    "Serial device used by direct battery reading.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  direct_serial_port_candidates_property_ = new rviz_common::properties::StringProperty(
    "Serial Port Candidates",
    "/dev/ttyACM0,/dev/ttyUSB0,/dev/ttyACM1,/dev/ttyUSB1",
    "Comma separated serial device candidates used by direct reading.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  direct_baudrate_property_ = new rviz_common::properties::IntProperty(
    "Baudrate",
    19200,
    "Serial baudrate used by direct reading.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    1,
    1000000
  );
  direct_slave_id_property_ = new rviz_common::properties::IntProperty(
    "Slave ID",
    3,
    "Modbus RTU slave id used by direct reading.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    1,
    247
  );
  direct_poll_hz_property_ = new rviz_common::properties::FloatProperty(
    "Poll Hz",
    6.0f,
    "Direct battery read frequency.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  direct_poll_hz_property_->setMin(0.1f);
  direct_timeout_property_ = new rviz_common::properties::FloatProperty(
    "Read Timeout (s)",
    0.25f,
    "Serial Modbus read timeout.",
    this,
    SLOT(updateOverlayStyle()),
    this
  );
  direct_timeout_property_->setMin(0.05f);
  direct_read_retries_property_ = new rviz_common::properties::IntProperty(
    "Read Retries",
    1,
    "Retry count after a failed direct read.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    0,
    10
  );
  direct_offline_after_failures_property_ = new rviz_common::properties::IntProperty(
    "Offline After Failures",
    3,
    "Mark offline after this many consecutive direct read failures.",
    this,
    SLOT(updateOverlayStyle()),
    this,
    1,
    100
  );
  topic_property_->setHidden(direct_serial_enabled_property_->getBool());
}

BatteryOverlayDisplay::~BatteryOverlayDisplay()
{
  stopDirectReader();
  if (render_panel_ != nullptr) {
    render_panel_->removeEventFilter(this);
  }
  if (host_window_ != nullptr) {
    host_window_->removeEventFilter(this);
  }
  if (overlay_widget_ != nullptr) {
    overlay_widget_->hide();
    overlay_widget_->deleteLater();
    overlay_widget_ = nullptr;
  }
}

void BatteryOverlayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  ensureOverlayWidget();
  startDirectReader();
  refreshOverlay();
}

void BatteryOverlayDisplay::reset()
{
  RTDClass::reset();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = BatteryStateSnapshot{};
  }
  refreshOverlay();
}

void BatteryOverlayDisplay::update(float wall_dt, float ros_dt)
{
  RTDClass::update(wall_dt, ros_dt);
  ensureOverlayWidget();
  refreshOverlay();
}

void BatteryOverlayDisplay::processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  BatteryStateSnapshot next;
  next.has_message = true;
  next.present = msg->present;
  next.percentage = std::isfinite(msg->percentage) ? static_cast<float>(msg->percentage) : 0.0f;
  next.location = QString::fromStdString(msg->location);
  next.received_at = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = next;
  }

  QMetaObject::invokeMethod(this, "updateOverlayFromState", Qt::QueuedConnection);
}

void BatteryOverlayDisplay::onEnable()
{
  RTDClass::onEnable();
  startDirectReader();
  refreshOverlay();
}

void BatteryOverlayDisplay::onDisable()
{
  RTDClass::onDisable();
  stopDirectReader();
  if (overlay_widget_ != nullptr) {
    overlay_widget_->hide();
  }
}

bool BatteryOverlayDisplay::eventFilter(QObject * watched, QEvent * event)
{
  if ((watched == render_panel_ || watched == host_window_) && event != nullptr) {
    switch (event->type()) {
      case QEvent::Resize:
      case QEvent::Show:
      case QEvent::Hide:
      case QEvent::Move:
      case QEvent::LayoutRequest:
      case QEvent::WindowStateChange:
        updateOverlayGeometry();
        refreshOverlay();
        break;
      default:
        break;
    }
  }
  return QObject::eventFilter(watched, event);
}

void BatteryOverlayDisplay::updateOverlayFromState()
{
  refreshOverlay();
}

void BatteryOverlayDisplay::updateOverlayStyle()
{
  topic_property_->setHidden(direct_serial_enabled_property_->getBool());
  restartDirectReader();
  refreshOverlay();
}

void BatteryOverlayDisplay::startDirectReader()
{
  std::lock_guard<std::mutex> lock(direct_reader_mutex_);
  if (direct_reader_running_ || !direct_serial_enabled_property_->getBool()) {
    return;
  }
  direct_reader_stop_requested_ = false;
  direct_reader_running_ = true;
  direct_reader_thread_ = std::thread(&BatteryOverlayDisplay::directReaderLoop, this);
}

void BatteryOverlayDisplay::stopDirectReader()
{
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(direct_reader_mutex_);
    direct_reader_stop_requested_ = true;
    if (direct_reader_thread_.joinable()) {
      thread_to_join = std::move(direct_reader_thread_);
    }
  }
  if (thread_to_join.joinable()) {
    thread_to_join.join();
  }
  direct_reader_running_ = false;
}

void BatteryOverlayDisplay::restartDirectReader()
{
  const bool should_run = isEnabled() && direct_serial_enabled_property_->getBool();
  stopDirectReader();
  if (should_run) {
    startDirectReader();
  }
}

std::vector<std::string> BatteryOverlayDisplay::directPortCandidates() const
{
  std::vector<std::string> ports;
  const std::string primary = direct_serial_port_property_->getStdString();
  if (!primary.empty()) {
    ports.push_back(primary);
  }
  for (const auto & candidate : splitCsv(direct_serial_port_candidates_property_->getStdString())) {
    if (!candidate.empty() && std::find(ports.begin(), ports.end(), candidate) == ports.end()) {
      ports.push_back(candidate);
    }
  }
  return ports;
}

void BatteryOverlayDisplay::directReaderLoop()
{
  int consecutive_failures = 0;
  SerialModbusRtuClient client(
    direct_serial_port_property_->getStdString(),
    direct_baudrate_property_->getInt(),
    direct_timeout_property_->getFloat());

  while (!direct_reader_stop_requested_) {
    const auto ports = directPortCandidates();
    for (const auto & port : ports) {
      if (::access(port.c_str(), F_OK) == 0) {
        client.setPort(port);
        break;
      }
    }

    bool ok = false;
    for (int attempt = 0; attempt <= direct_read_retries_property_->getInt(); ++attempt) {
      if (direct_reader_stop_requested_) {
        break;
      }
      try {
        publishDirectSnapshot(readDirectSnapshot(client));
        consecutive_failures = 0;
        ok = true;
        break;
      } catch (const std::exception &) {
        if (attempt < direct_read_retries_property_->getInt()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
      }
    }

    if (!ok) {
      ++consecutive_failures;
      if (consecutive_failures >= direct_offline_after_failures_property_->getInt()) {
        publishDirectOffline();
      }
    }

    const float poll_hz = std::max(0.1f, direct_poll_hz_property_->getFloat());
    const auto sleep_step = std::chrono::milliseconds(20);
    const auto sleep_total = std::chrono::duration<float>(1.0f / poll_hz);
    auto slept = std::chrono::duration<float>(0.0f);
    while (!direct_reader_stop_requested_ && slept < sleep_total) {
      std::this_thread::sleep_for(sleep_step);
      slept += sleep_step;
    }
  }
}

BatteryOverlayDisplay::DirectBatterySnapshot BatteryOverlayDisplay::readDirectSnapshot(
  SerialModbusRtuClient & client)
{
  const auto values = readDirectRegisters(client, direct_slave_id_property_->getInt(), 114, 5);
  if (values.size() < 5) {
    throw std::runtime_error("State register response too short");
  }
  DirectBatterySnapshot snapshot;
  snapshot.present = socValid(values[2]);
  snapshot.voltage = static_cast<float>(values[0]) * 0.01f;
  snapshot.current = static_cast<float>(toSigned16(values[1])) * 0.01f;
  snapshot.percentage = clampSoc(values[2]) / 100.0f;
  return snapshot;
}

std::vector<std::uint16_t> BatteryOverlayDisplay::readDirectRegisters(
  SerialModbusRtuClient & client,
  int slave_id,
  int start_addr,
  int register_count)
{
  return client.readHoldingRegisters(slave_id, start_addr, register_count);
}

void BatteryOverlayDisplay::publishDirectSnapshot(const DirectBatterySnapshot & snapshot)
{
  BatteryStateSnapshot next;
  next.has_message = true;
  next.present = snapshot.present;
  next.percentage = snapshot.percentage;
  next.location = QString();
  next.received_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = next;
  }
  QMetaObject::invokeMethod(this, "updateOverlayFromState", Qt::QueuedConnection);
}

void BatteryOverlayDisplay::publishDirectOffline()
{
  BatteryStateSnapshot next;
  next.has_message = true;
  next.present = false;
  next.percentage = 0.0f;
  next.location = QString();
  next.received_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = next;
  }
  QMetaObject::invokeMethod(this, "updateOverlayFromState", Qt::QueuedConnection);
}

float BatteryOverlayDisplay::clampSoc(std::uint16_t raw)
{
  return std::max(0.0f, std::min(100.0f, static_cast<float>(raw)));
}

bool BatteryOverlayDisplay::socValid(std::uint16_t raw)
{
  return raw != 0xFFFF && raw <= 100;
}

std::int16_t BatteryOverlayDisplay::toSigned16(std::uint16_t raw)
{
  return static_cast<std::int16_t>(raw);
}

void BatteryOverlayDisplay::ensureOverlayWidget()
{
  if (context_ == nullptr || context_->getViewManager() == nullptr) {
    return;
  }

  rviz_common::RenderPanel * panel = context_->getViewManager()->getRenderPanel();
  if (panel == nullptr) {
    return;
  }

  QWidget * host_window = panel->window();
  if (render_panel_ == panel && host_window_ == host_window && overlay_widget_ != nullptr) {
    return;
  }

  if (render_panel_ != nullptr) {
    render_panel_->removeEventFilter(this);
  }
  if (host_window_ != nullptr) {
    host_window_->removeEventFilter(this);
  }
  if (overlay_widget_ != nullptr) {
    overlay_widget_->hide();
    overlay_widget_->deleteLater();
    overlay_widget_ = nullptr;
  }

  render_panel_ = panel;
  host_window_ = host_window;
  render_panel_->installEventFilter(this);
  if (host_window_ != nullptr && host_window_ != render_panel_) {
    host_window_->installEventFilter(this);
  }

  overlay_widget_ = new BatteryOverlayWidget(
    host_window_,
    Qt::FramelessWindowHint | Qt::Tool | Qt::NoDropShadowWindowHint
  );
  overlay_widget_->hide();
  overlay_widget_->raise();
  updateOverlayGeometry();
}

void BatteryOverlayDisplay::refreshOverlay()
{
  ensureOverlayWidget();
  if (overlay_widget_ == nullptr) {
    return;
  }

  const BatteryStateSnapshot state = snapshotState();
  BatteryOverlayWidget::Visual visual;
  visual.online = isOnline(state);
  visual.percentage = std::clamp(state.percentage, 0.0f, 1.0f);
  visual.label = effectiveLabel(state);
  visual.battery_width = battery_width_property_->getInt();
  visual.battery_height = battery_height_property_->getInt();
  visual.font_size = font_size_property_->getInt();

  overlay_widget_->setVisual(visual);
  updateOverlayGeometry();

  bool should_show = isEnabled();
  if (hide_when_offline_property_->getBool() && !visual.online) {
    should_show = false;
  }
  if (render_panel_ == nullptr || !render_panel_->isVisible()) {
    should_show = false;
  }
  if (host_window_ != nullptr && !host_window_->isVisible()) {
    should_show = false;
  }

  if (should_show) {
    overlay_widget_->show();
    overlay_widget_->raise();
  } else {
    overlay_widget_->hide();
  }
}

void BatteryOverlayDisplay::updateOverlayGeometry()
{
  if (overlay_widget_ == nullptr || render_panel_ == nullptr || host_window_ == nullptr) {
    return;
  }

  const QSize size = overlay_widget_->desiredSize();
  const int x = std::max(
    0, render_panel_->width() - margin_x_property_->getInt() - size.width());
  const int y = std::max(0, margin_y_property_->getInt());
  const QPoint global_top_left = render_panel_->mapToGlobal(QPoint(x, y));
  overlay_widget_->setGeometry(
    global_top_left.x(), global_top_left.y(), size.width(), size.height());
  overlay_widget_->raise();
}

BatteryOverlayDisplay::BatteryStateSnapshot BatteryOverlayDisplay::snapshotState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_state_;
}

QString BatteryOverlayDisplay::effectiveLabel(const BatteryStateSnapshot & state) const
{
  const QString static_label = label_property_->getString().trimmed();
  if (!static_label.isEmpty()) {
    return static_label;
  }
  if (use_message_location_property_->getBool()) {
    return state.location.trimmed();
  }
  return QString();
}

bool BatteryOverlayDisplay::isOnline(const BatteryStateSnapshot & state) const
{
  if (!state.has_message || !state.present) {
    return false;
  }

  const float timeout_sec = std::max(0.0f, offline_timeout_property_->getFloat());
  if (timeout_sec <= 0.0f) {
    return true;
  }

  const auto age = std::chrono::duration_cast<std::chrono::duration<float>>(
    std::chrono::steady_clock::now() - state.received_at
  ).count();
  return age <= timeout_sec;
}

}  // namespace openarmx_battery_monitor

PLUGINLIB_EXPORT_CLASS(
  openarmx_battery_monitor::BatteryOverlayDisplay,
  rviz_common::Display
)
