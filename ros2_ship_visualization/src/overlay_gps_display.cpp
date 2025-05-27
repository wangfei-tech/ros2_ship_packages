#include "overlay_gps_display.h"

#include <QImage>
#include <opencv2/opencv.hpp>
#include <Python.h>
#include <fstream>
#include <memory>

namespace ros_ship_visualization
{
OverlayGpsDisplay::OverlayGpsDisplay()
: Node("overlay_gps_display")
{
  this->declare_parameter("google_static_map_api_key", "");
  api_key_ = this->get_parameter("google_static_map_api_key").as_string();
  
  std::string package_path = ament_index_cpp::get_package_share_directory("ros_ship_visualization");
  map_image_path_ = package_path + "/data/map.png";
  
  load_map_downloader_script();
  
  // Create properties
  zoom_property_ = new rviz_common::properties::IntProperty("Zoom", 19, "zoom of map", this, SLOT(updateGooleMapAPIProperty()));
  zoom_property_->setMax(22);
  zoom_property_->setMin(0);
  
  width_property_ = new rviz_common::properties::IntProperty("Width", 320, "request map image width", this, SLOT(updateGooleMapAPIProperty()));
  width_property_->setMax(640);
  width_property_->setMin(0);
  
  height_property_ = new rviz_common::properties::IntProperty("Height", 320, "request map image height", this, SLOT(updateGooleMapAPIProperty()));
  height_property_->setMax(640);
  height_property_->setMin(0);
  
  scale_property_ = new rviz_common::properties::IntProperty("Scale", 1, "request map image scale", this, SLOT(updateGooleMapAPIProperty()));
  scale_property_->setMax(2);
  scale_property_->setMin(1);
  
  history_length_property_ = new rviz_common::properties::IntProperty("History Length", 15, "history length", this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  
  fix_buffer_ = boost::circular_buffer<sensor_msgs::msg::NavSatFix>(history_length_property_->getInt());
  
  maptype_property_ = new rviz_common::properties::EnumProperty("Map Type", "roadmap", "map type", this, SLOT(updateGooleMapAPIProperty()));
  maptype_property_->addOption("roadmap", ROADMAP);
  maptype_property_->addOption("terrain", TERRAIN);
  maptype_property_->addOption("satellite", SATELLITE);
  maptype_property_->addOption("hybrid", HYBRID);
  
  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.8, "image alpha", this, SLOT(updateDisplayProperty()));
  alpha_property_->setMax(1);
  alpha_property_->setMin(0);
  
  position_x_property_ = new rviz_common::properties::IntProperty("Position X", 0, "map image position x", this, SLOT(updateDisplayProperty()));
  position_y_property_ = new rviz_common::properties::IntProperty("Position Y", 0, "map image position y", this, SLOT(updateDisplayProperty()));
  
  messages_per_plot_property_ = new rviz_common::properties::IntProperty("Message per plot", 5, "message per plot", this, SLOT(updateDisplayProperty()));
  messages_per_plot_property_->setMin(1);
}

OverlayGpsDisplay::~OverlayGpsDisplay()
{
  delete zoom_property_;
  delete width_property_;
  delete height_property_;
  delete scale_property_;
  delete alpha_property_;
  delete maptype_property_;
  
  Py_XDECREF(map_downloader_function_);
  Py_Finalize();
}

bool OverlayGpsDisplay::download_map(std::string request_url)
{
  PyObject* args = PyTuple_New(1);
  PyObject* kw_args = PyDict_New();
  PyObject* request_url_str = PyUnicode_FromString(request_url.c_str());
  PyTuple_SetItem(args, 0, request_url_str);
  
  try
  {
    PyObject* responce = PyObject_Call(map_downloader_function_, args, kw_args);
    Py_XDECREF(responce);
  }
  catch(...)
  {
    Py_DECREF(args);
    Py_DECREF(kw_args);
    return false;
  }
  
  Py_DECREF(args);
  Py_DECREF(kw_args);
  return true;
}

void OverlayGpsDisplay::onInitialize()
{
  RTDClass::onInitialize();
}

void OverlayGpsDisplay::reset()
{
  RTDClass::reset();
  fix_buffer_.clear();
}

void OverlayGpsDisplay::processMessage(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if(msg->header.stamp.sec % messages_per_plot_property_->getInt() == 0)
  {
    fix_buffer_.push_back(*msg);
    std::string request_url;
    if(build_request_url(msg, request_url) == false)
    {
      return;
    }
    
    try
    {
      download_map(request_url);
    }
    catch(...)
    {
      RCLCPP_ERROR(this->get_logger(), "failed to request map");
    }
    
    if(check_map_image_file() == false)
    {
      this->setStatus(rviz_common::properties::StatusProperty::Error, "MapImageFileNotExist", "map image file does not exist. Check API Key.");
      return;
    }
    
    this->setStatus(rviz_common::properties::StatusProperty::Ok, "MapImageFileNotExist", "map image file exist");
    
    cv::Mat map_image = cv::imread(map_image_path_);
    if(map_image.cols <= 0 || map_image.rows <= 0)
    {
      this->setStatus(rviz_common::properties::StatusProperty::Error, "MapImageFileIsInvalidSize", "map image file is invalid size. Check API Key.");
      return;
    }
    
    this->setStatus(rviz_common::properties::StatusProperty::Ok, "MapImageFileIsInvalidSize", "map image file is valid size");
    
    if(!overlay_)
    {
      static int count = 0;
      rviz_common::UniformStringStream ss;
      ss << "OverlayGpsDisplayObject" << count++;
      overlay_ = std::make_shared<rviz_common::OverlayObject>(ss.str());
      overlay_->show();
    }
    
    if (overlay_)
    {
      overlay_->setDimensions(width_property_->getInt(), height_property_->getInt());
      overlay_->setPosition(position_x_property_->getInt(), position_y_property_->getInt());
    }
    
    overlay_->updateTextureSize(width_property_->getInt(), height_property_->getInt());
    rviz_common::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_);
    
    for (int i = 0; i < overlay_->getTextureWidth(); i++)
    {
      for (int j = 0; j < overlay_->getTextureHeight(); j++)
      {
        QColor color(map_image.data[j * map_image.step + i * map_image.elemSize() + 2],
                     map_image.data[j * map_image.step + i * map_image.elemSize() + 1],
                     map_image.data[j * map_image.step + i * map_image.elemSize() + 0],
                     alpha_property_->getFloat() * 255.0);
        Hud.setPixel(i, j, color.rgba());
      }
    }
  }
}

void OverlayGpsDisplay::updateGooleMapAPIProperty()
{
  // Implementation remains the same
}

void OverlayGpsDisplay::updateDisplayProperty()
{
  // Implementation remains the same
}

void OverlayGpsDisplay::updateHistoryLength()
{
  fix_buffer_.clear();
  fix_buffer_ = boost::circular_buffer<sensor_msgs::msg::NavSatFix>(history_length_property_->getInt());
}

void OverlayGpsDisplay::load_map_downloader_script()
{
  std::string package_path = ament_index_cpp::get_package_share_directory("ros_ship_visualization");
  std::string map_downloader_path = package_path + "/script/map_downloader.py";
  
  std::ifstream ifs(map_downloader_path);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open map downloader script: %s", map_downloader_path.c_str());
    return;
  }
  
  std::string map_downloader_script((std::istreambuf_iterator<char>(ifs)), 
                                   std::istreambuf_iterator<char>());
  
  Py_Initialize();
  PyObject* main_module = PyImport_AddModule("__main__");
  PyObject* global_dict = PyModule_GetDict(main_module);
  
  PyRun_String(map_downloader_script.c_str(), Py_file_input, global_dict, global_dict);
  
  map_downloader_function_ = PyDict_GetItemString(global_dict, "download_map");
  if (!map_downloader_function_ || !PyCallable_Check(map_downloader_function_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load download_map function from Python script");
    Py_XDECREF(map_downloader_function_);
    map_downloader_function_ = nullptr;
  }
}

bool OverlayGpsDisplay::build_request_url(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg, std::string& request_url)
{
  request_url = "https://maps.googleapis.com/maps/api/staticmap?";
  
  std::string center_request = "center=" + std::to_string(msg->longitude) + "," + std::to_string(msg->latitude);
  request_url = request_url + center_request;
  
  std::string markers_request = "&markers=color:red%7C" + std::to_string(msg->longitude) + "," + std::to_string(msg->latitude);
  request_url = request_url + markers_request;
  
  std::string zoom_request = "&zoom=" + std::to_string(zoom_property_->getInt());
  request_url = request_url + zoom_request;
  
  std::string size_request = "&size=" + std::to_string(width_property_->getInt()) + "x" + std::to_string(height_property_->getInt());
  request_url = request_url + size_request;
  
  if(maptype_property_->getOptionInt() == ROADMAP)
  {
    std::string maptype_url = "&maptype=roadmap";
    request_url = request_url + maptype_url;
  }
  if(maptype_property_->getOptionInt() == TERRAIN)
  {
    std::string maptype_url = "&maptype=terrain";
    request_url = request_url + maptype_url;
  }
  if(maptype_property_->getOptionInt() == SATELLITE)
  {
    std::string maptype_url = "&maptype=satellite";
    request_url = request_url + maptype_url;
  }
  if(maptype_property_->getOptionInt() == HYBRID)
  {
    std::string maptype_url = "&maptype=hybrid";
    request_url = request_url + maptype_url;
  }
  
  std::string path_url = "&path=color:blue|weight:5";
  for(auto fix_data = fix_buffer_.rbegin(); fix_data != fix_buffer_.rend(); ++fix_data)
  {
    path_url = path_url + "|" + std::to_string(fix_data->longitude) + "," + std::to_string(fix_data->latitude);
  }
  request_url = request_url + path_url;
  request_url = request_url + "&format=png";
  
  std::string key_request = "&key=" + api_key_;
  request_url = request_url + key_request;
  
  if(request_url.size() > MAX_REQUEST_URL_LENGTH)
  {
    QString message = QString("%1%2").arg(request_url.size()).arg(" request url is too long");
    this->setStatus(rviz_common::properties::StatusProperty::Error, "TooLongRequestUrl", message);
    return false;
  }
  else
  {
    QString message = QString("%1%2").arg(request_url.size()).arg("characters");
    this->setStatus(rviz_common::properties::StatusProperty::Ok, "TooLongRequestUrl", message);
  }
  
  return true;
}

bool OverlayGpsDisplay::check_map_image_file()
{
  std::ifstream ifs(map_image_path_);
  return ifs.is_open();
}
}  // namespace ros_ship_visualization

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_ship_visualization::OverlayGpsDisplay, rviz_common::Display)