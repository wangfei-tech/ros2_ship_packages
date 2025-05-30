//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <nmea_gps_sensor_plugin.h>
#include <gazebo/physics/physics.hh>

//headers in stl
#include <sys/time.h>
#include <time.h>
#include <vector>

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;

namespace gazebo
{

  nmea_gps_sensor_plugin::nmea_gps_sensor_plugin()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  nmea_gps_sensor_plugin::~nmea_gps_sensor_plugin()
  {
    updateTimer.Disconnect(updateConnection);

    node_handle_->shutdown();
    delete node_handle_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void nmea_gps_sensor_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    world = _model->GetWorld();

    // load parameters
    if (!_sdf->HasElement("robotNamespace"))
      namespace_.clear();
    else
      namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

    if (!_sdf->HasElement("bodyName"))
    {
      link = _model->GetLink();
      link_name_ = link->GetName();
    }
    else {
      link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
      link = _model->GetLink(link_name_);
    }

    if (!link)
    {
      ROS_FATAL("nmea_gps_sensor_plugin plugin error: bodyName: %s does not exist\n", link_name_.c_str());
      return;
    }

    // default parameters
    frame_id_ = "/world";

    reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
    reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
    reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
    reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

    fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
    fix_.status.service = 0;

    if (_sdf->HasElement("topicName"))
      nmea_sentence_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
    else
      nmea_sentence_topic_ = "nmea_sentence";

    if (_sdf->HasElement("frameId"))
      frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

    if (_sdf->HasElement("referenceLatitude"))
      _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);

    if (_sdf->HasElement("referenceLongitude"))
      _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

    if (_sdf->HasElement("referenceHeading"))
      if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
        reference_heading_ *= M_PI/180.0;

    if (_sdf->HasElement("referenceAltitude"))
      _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);

    if (_sdf->HasElement("status")) {
      int status = fix_.status.status;
      if (_sdf->GetElement("status")->GetValue()->Get(status))
        fix_.status.status = static_cast<sensor_msgs::NavSatStatus::_status_type>(status);
    }

    if (_sdf->HasElement("service")) {
      unsigned int service = fix_.status.service;
      if (_sdf->GetElement("service")->GetValue()->Get(service))
        fix_.status.service = static_cast<sensor_msgs::NavSatStatus::_service_type>(service);
    }

    fix_.header.frame_id = frame_id_;
    velocity_.header.frame_id = frame_id_;

    position_error_model_.Load(_sdf);
    velocity_error_model_.Load(_sdf, "velocity");

    // calculate earth radii
    double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
    double prime_vertical_radius = equatorial_radius * sqrt(temp);
    radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
    radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    node_handle_ = new ros::NodeHandle(namespace_);
    nmea_sentence_publisher_ = node_handle_->advertise<nmea_msgs::Sentence>(nmea_sentence_topic_, 10);
    //fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
    //velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);

    Reset();

    // connect Update function
    updateTimer.setUpdateRate(4.0);
    updateTimer.Load(world, _sdf);
    updateConnection = updateTimer.Connect(boost::bind(&nmea_gps_sensor_plugin::Update, this));
  }

  void nmea_gps_sensor_plugin::Reset()
  {
    updateTimer.Reset();
    position_error_model_.reset();
    velocity_error_model_.reset();
  }

  void nmea_gps_sensor_plugin::dynamicReconfigureCallback(nmea_gps_sensor_plugin::GNSSConfig &config, uint32_t level)
  {
    using sensor_msgs::NavSatStatus;
    if (level == 1) {
      if (!config.STATUS_FIX) {
        fix_.status.status = NavSatStatus::STATUS_NO_FIX;
      } else {
        fix_.status.status = (config.STATUS_SBAS_FIX ? NavSatStatus::STATUS_SBAS_FIX : 0) |
                             (config.STATUS_GBAS_FIX ? NavSatStatus::STATUS_GBAS_FIX : 0);
      }
      fix_.status.service = (config.SERVICE_GPS     ? NavSatStatus::SERVICE_GPS : 0) |
                            (config.SERVICE_GLONASS ? NavSatStatus::SERVICE_GLONASS : 0) |
                            (config.SERVICE_COMPASS ? NavSatStatus::SERVICE_COMPASS : 0) |
                            (config.SERVICE_GALILEO ? NavSatStatus::SERVICE_GALILEO : 0);
    } else {
      config.STATUS_FIX      = (fix_.status.status != NavSatStatus::STATUS_NO_FIX);
      config.STATUS_SBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_SBAS_FIX);
      config.STATUS_GBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_GBAS_FIX);
      config.SERVICE_GPS     = (fix_.status.service & NavSatStatus::SERVICE_GPS);
      config.SERVICE_GLONASS = (fix_.status.service & NavSatStatus::SERVICE_GLONASS);
      config.SERVICE_COMPASS = (fix_.status.service & NavSatStatus::SERVICE_COMPASS);
      config.SERVICE_GALILEO = (fix_.status.service & NavSatStatus::SERVICE_GALILEO);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void nmea_gps_sensor_plugin::Update()
  {
    common::Time sim_time = world->GetSimTime();
    double dt = updateTimer.getTimeSinceLastUpdate().Double();

    math::Pose pose = link->GetWorldPose();

    gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
    gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);

    // An offset error in the velocity is integrated into the position error for the next timestep.
    // Note: Usually GNSS receivers have almost no drift in the velocity signal.
    position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());

    fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
    velocity_.header.stamp = fix_.header.stamp;

    fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
    fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
    fix_.altitude  = reference_altitude_  + position.z;
    velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
    velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
    velocity_.vector.z = velocity.z;

    fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_.position_covariance[0] = position_error_model_.drift.x*position_error_model_.drift.x + position_error_model_.gaussian_noise.x*position_error_model_.gaussian_noise.x;
    fix_.position_covariance[4] = position_error_model_.drift.y*position_error_model_.drift.y + position_error_model_.gaussian_noise.y*position_error_model_.gaussian_noise.y;
    fix_.position_covariance[8] = position_error_model_.drift.z*position_error_model_.drift.z + position_error_model_.gaussian_noise.z*position_error_model_.gaussian_noise.z;

    nmea_sentence_publisher_.publish(build_GPGGA_sentence());
    nmea_sentence_publisher_.publish(build_GPRMC_sentence());

    //fix_publisher_.publish(fix_);
    //velocity_publisher_.publish(velocity_);
  }

  nmea_msgs::Sentence nmea_gps_sensor_plugin::build_GPGGA_sentence()
  {
    nmea_msgs::Sentence sentence;
    sentence.header = fix_.header;
    struct timeval time_value;
    struct tm *time_st;
    time_t timer;
    timer = time(NULL);
    time_st = gmtime(&timer);
    gettimeofday(&time_value, NULL);
    sentence.sentence = std::to_string(time_st->tm_hour) + std::to_string(time_st->tm_min) + std::to_string(time_st->tm_sec);
    sentence.sentence = sentence.sentence + "." + std::to_string(time_value.tv_usec) + ",";
    //sentence.sentence = sentence.sentence + "A,";
    if(fix_.latitude < 0)
    {
      int digree = std::floor(std::fabs(fix_.latitude));
      double min = (std::fabs(fix_.latitude)-std::floor(std::fabs(fix_.latitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "S,";
    }
    else
    {
      int digree = std::floor(std::fabs(fix_.latitude));
      double min = (std::fabs(fix_.latitude)-std::floor(std::fabs(fix_.latitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "N,";
    }
    if(fix_.longitude < 0)
    {
      int digree = std::floor(std::fabs(fix_.longitude));
      double min = (std::fabs(fix_.longitude)-std::floor(std::fabs(fix_.longitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "W,";
    }
    else
    {
      int digree = std::floor(std::fabs(fix_.longitude));
      double min = (std::fabs(fix_.longitude)-std::floor(std::fabs(fix_.longitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "E,";
    }
    sentence.sentence = sentence.sentence + "1,08,1.0,";
    sentence.sentence = sentence.sentence + std::to_string(fix_.altitude) + ",";
    sentence.sentence = sentence.sentence + "M,";
    sentence.sentence = sentence.sentence + std::to_string(fix_.altitude) + ",";
    sentence.sentence = sentence.sentence + "M,,0000";
    sentence.sentence = sentence.sentence + "*" + get_nmea_checksum(sentence.sentence);
    sentence.sentence = "$GPGGA," + sentence.sentence;
    return sentence;
  }

  nmea_msgs::Sentence nmea_gps_sensor_plugin::build_GPRMC_sentence()
  {
    nmea_msgs::Sentence sentence;
    sentence.header = fix_.header;
    struct timeval time_value;
    struct tm *time_st;
    time_t timer;
    timer = time(NULL);
    time_st = gmtime(&timer);
    gettimeofday(&time_value, NULL);
    sentence.sentence = std::to_string(time_st->tm_hour) + std::to_string(time_st->tm_min) + std::to_string(time_st->tm_sec);
    sentence.sentence = sentence.sentence + "." + std::to_string(time_value.tv_usec) + ",";
    sentence.sentence = sentence.sentence + "A,";
    if(fix_.latitude < 0)
    {
      int digree = std::floor(std::fabs(fix_.latitude));
      double min = (std::fabs(fix_.latitude)-std::floor(std::fabs(fix_.latitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "S,";
    }
    else
    {
      int digree = std::floor(std::fabs(fix_.latitude));
      double min = (std::fabs(fix_.latitude)-std::floor(std::fabs(fix_.latitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "N,";
    }
    if(fix_.longitude < 0)
    {
      int digree = std::floor(std::fabs(fix_.longitude));
      double min = (std::fabs(fix_.longitude)-std::floor(std::fabs(fix_.longitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "W,";
    }
    else
    {
      int digree = std::floor(std::fabs(fix_.longitude));
      double min = (std::fabs(fix_.longitude)-std::floor(std::fabs(fix_.longitude)))/60;
      sentence.sentence = sentence.sentence + std::to_string(digree) + std::to_string(min) + ",";
      sentence.sentence = sentence.sentence + "E,";
    }
    double velocity_knot = (std::pow(velocity_.vector.x,2) + std::pow(velocity_.vector.y,2) + std::pow(velocity_.vector.z,2))*1.94384;
    sentence.sentence = sentence.sentence + std::to_string(velocity_knot) + ",";
    double velocity_vector = std::atan2(velocity_.vector.y, velocity_.vector.x)*180/3.14159265359;
    sentence.sentence = sentence.sentence + std::to_string(velocity_vector) + ",";
    sentence.sentence = sentence.sentence + ",,,,A";
    sentence.sentence = sentence.sentence + "*" + get_nmea_checksum(sentence.sentence);
    sentence.sentence = "$GPRMC," + sentence.sentence;
    return sentence;
  }

  std::string nmea_gps_sensor_plugin::get_nmea_checksum(std::string sentence)
  {
    return "00";
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(nmea_gps_sensor_plugin)

} // namespace gazebo
