#ifndef GPS_PANEL_H
#define GPS_PANEL_H

#define ROADMAP 0
#define TERRAIN 1
#define SATELLITE 2
#define HYBRID 3

#define PNG 0
#define GIF 1
#define JPEG 2

#define MAX_REQUEST_URL_LENGTH 8192

#include "overlay_utils.h"

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Qt headers
#include <QWidget>

// RViz 2 headers
#include <rviz_common/display_context.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/display.hpp>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS 2 messages
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Python headers
#include <Python.h>

// STL headers
#include <string>
#include <fstream>
#include <iostream>
#include <memory>

// Boost
#include <boost/circular_buffer.hpp>

namespace ros_ship_visualization
{

class OverlayGpsDisplay : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::NavSatFix>
{
  Q_OBJECT
public:
  OverlayGpsDisplay();
  virtual ~OverlayGpsDisplay();

protected:
  virtual void onInitialize() override;
  virtual void reset() override;

private:
  void processMessage(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg);
  bool download_map(const std::string & request_url);
  void load_map_downloader_script();
  bool build_request_url(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg, std::string & request_url);
  inline bool check_map_image_file();

  // Property variables
  rviz_common::properties::IntProperty* zoom_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* scale_property_;
  rviz_common::properties::IntProperty* position_x_property_;
  rviz_common::properties::IntProperty* position_y_property_;
  rviz_common::properties::IntProperty* messages_per_plot_property_;
  rviz_common::properties::IntProperty* history_length_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::EnumProperty* maptype_property_;

  // Data storage
  boost::circular_buffer<sensor_msgs::msg::NavSatFix> fix_buffer_;
  PyObject* map_downloader_function_;
  std::string map_image_path_;
  std::string api_key_;
  std::shared_ptr<rviz_common::OverlayObject> overlay_;

private Q_SLOTS:
  void updateGooleMapAPIProperty();
  void updateDisplayProperty();
  void updateHistoryLength();
};

}  // namespace ros_ship_visualization

#endif // GPS_PANEL_H