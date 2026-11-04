#ifndef OPENARMX_BATTERY_MONITOR__BATTERY_OVERLAY_DISPLAY_HPP_
#define OPENARMX_BATTERY_MONITOR__BATTERY_OVERLAY_DISPLAY_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QObject>
#include <QString>

#include "rviz_common/ros_topic_display.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace rviz_common
{
class RenderPanel;

namespace properties
{
class BoolProperty;
class FloatProperty;
class IntProperty;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

class QEvent;

namespace openarmx_battery_monitor
{

class BatteryOverlayWidget;

class BatteryOverlayDisplay : public rviz_common::RosTopicDisplay<sensor_msgs::msg::BatteryState>
{
  Q_OBJECT

public:
  BatteryOverlayDisplay();
  ~BatteryOverlayDisplay() override;

  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg) override;
  void onEnable() override;
  void onDisable() override;
  bool eventFilter(QObject * watched, QEvent * event) override;

private Q_SLOTS:
  void updateOverlayFromState();
  void updateOverlayStyle();

private:
  using RTDClass = rviz_common::RosTopicDisplay<sensor_msgs::msg::BatteryState>;

  struct BatteryStateSnapshot
  {
    bool has_message{false};
    bool present{false};
    float percentage{0.0f};
    QString location;
    std::chrono::steady_clock::time_point received_at{};
  };

  struct DirectBatterySnapshot
  {
    bool present{false};
    float percentage{0.0f};
    float voltage{0.0f};
    float current{0.0f};
  };

  class SerialModbusRtuClient;

  void startDirectReader();
  void stopDirectReader();
  void restartDirectReader();
  void directReaderLoop();
  void publishDirectSnapshot(const DirectBatterySnapshot & snapshot);
  void publishDirectOffline();
  std::vector<std::string> directPortCandidates() const;
  DirectBatterySnapshot readDirectSnapshot(SerialModbusRtuClient & client);
  std::vector<std::uint16_t> readDirectRegisters(
    SerialModbusRtuClient & client,
    int slave_id,
    int start_addr,
    int register_count);
  static float clampSoc(std::uint16_t raw);
  static bool socValid(std::uint16_t raw);
  static std::int16_t toSigned16(std::uint16_t raw);

  void ensureOverlayWidget();
  void refreshOverlay();
  void updateOverlayGeometry();
  BatteryStateSnapshot snapshotState() const;
  QString effectiveLabel(const BatteryStateSnapshot & state) const;
  bool isOnline(const BatteryStateSnapshot & state) const;

  rviz_common::RenderPanel * render_panel_{nullptr};
  QWidget * host_window_{nullptr};
  BatteryOverlayWidget * overlay_widget_{nullptr};

  rviz_common::properties::StringProperty * label_property_{nullptr};
  rviz_common::properties::BoolProperty * use_message_location_property_{nullptr};
  rviz_common::properties::BoolProperty * hide_when_offline_property_{nullptr};
  rviz_common::properties::IntProperty * margin_x_property_{nullptr};
  rviz_common::properties::IntProperty * margin_y_property_{nullptr};
  rviz_common::properties::IntProperty * battery_width_property_{nullptr};
  rviz_common::properties::IntProperty * battery_height_property_{nullptr};
  rviz_common::properties::IntProperty * font_size_property_{nullptr};
  rviz_common::properties::FloatProperty * offline_timeout_property_{nullptr};
  rviz_common::properties::BoolProperty * direct_serial_enabled_property_{nullptr};
  rviz_common::properties::StringProperty * direct_serial_port_property_{nullptr};
  rviz_common::properties::StringProperty * direct_serial_port_candidates_property_{nullptr};
  rviz_common::properties::IntProperty * direct_baudrate_property_{nullptr};
  rviz_common::properties::IntProperty * direct_slave_id_property_{nullptr};
  rviz_common::properties::FloatProperty * direct_poll_hz_property_{nullptr};
  rviz_common::properties::FloatProperty * direct_timeout_property_{nullptr};
  rviz_common::properties::IntProperty * direct_read_retries_property_{nullptr};
  rviz_common::properties::IntProperty * direct_offline_after_failures_property_{nullptr};

  mutable std::mutex state_mutex_;
  BatteryStateSnapshot latest_state_;

  std::atomic_bool direct_reader_running_{false};
  std::atomic_bool direct_reader_stop_requested_{false};
  std::thread direct_reader_thread_;
  std::mutex direct_reader_mutex_;
};

}  // namespace openarmx_battery_monitor

#endif  // OPENARMX_BATTERY_MONITOR__BATTERY_OVERLAY_DISPLAY_HPP_
