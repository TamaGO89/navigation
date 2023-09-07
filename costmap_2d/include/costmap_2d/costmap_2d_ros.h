/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Transform.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf);
  ~Costmap2DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  void resume();

  void updateMap();

  /**
   * @brief Reset each individual layer
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent() const
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * @brief Is the costmap stopped
   */
  bool isStopped() const
    {
      return stopped_;
    }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;

  /** @brief Returns costmap name */
  inline const std::string& getName() const noexcept
    {
      return name_;
    }

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  double getTransformTolerance() const
    {
      return transform_tolerance_;
    }

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
  Costmap2D* getCostmap() const
    {
      return layered_costmap_->getCostmap();
    }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  inline const std::string& getGlobalFrameID() const noexcept
    {
      return global_frame_;
    }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  inline const std::string& getBaseFrameID() const noexcept
    {
      return robot_base_frame_;
    }
  LayeredCostmap* getLayeredCostmap() const
    {
      return layered_costmap_;
    }

  /** @brief Returns the current padded footprint as a geometry_msgs::Polygon. */
  geometry_msgs::Polygon getRobotFootprintPolygon() const
  {
    return costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  inline const std::vector<geometry_msgs::Point>& getRobotFootprint() const noexcept
  {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  inline const std::vector<geometry_msgs::Point>& getUnpaddedRobotFootprint() const noexcept
  {
    return unpadded_footprint_;
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

  inline double to_double( const XmlRpc::XmlRpcValue& xml ) {
    switch ( xml.getType() ) {
      case XmlRpc::XmlRpcValue::TypeDouble:
        return double(xml);
      case XmlRpc::XmlRpcValue::TypeInt:
        return double( int(xml) );
      default:
        return 0.0;
    }
  };
  // EDIT : TAMA 2023-08-01 : Initialization for costmap
  inline bool init( const XmlRpc::XmlRpcValue& xml ) {
    std::cout << this->name_.c_str() << " : START " << std::endl;
    // DELETE THREAD
    if ( this->map_update_thread_ != NULL ) {
      this->map_update_thread_shutdown_ = true;
      this->map_update_thread_->join();
      delete this->map_update_thread_;
      this->map_update_thread_ = NULL;
    }
    // COSTMAP
    if ( ! this->layered_costmap_->isSizeLocked() ) {
      bool update = false;
      double resolution = this->getCostmap()->getResolution();
      if ( xml.hasMember("resolution") && resolution != to_double(xml["resolution"]) ) {
        resolution = to_double(xml["resolution"]); update = true; }
      uint32_t width = this->getCostmap()->getSizeInCellsX();
      if ( xml.hasMember("width") && width != uint32_t(to_double(xml["width"])/resolution) ) {
        width = uint32_t(to_double(xml["width"])/resolution); update = true; }
      uint32_t height = this->getCostmap()->getSizeInCellsY();
      if ( xml.hasMember("height") && height != uint32_t(to_double(xml["height"])/resolution) ) {
        height = uint32_t(to_double(xml["height"])/resolution); update = true; }
      double origin_x = this->getCostmap()->getOriginX();
      if ( xml.hasMember("origin_x") && origin_x != to_double(xml["origin_x"]) ) {
        origin_x = to_double(xml["origin_x"]); update = true; }
      double origin_y = this->getCostmap()->getOriginY();
      if ( xml.hasMember("origin_y") && origin_y != to_double(xml["origin_y"]) ) {
        origin_y = to_double(xml["origin_y"]); update = true; }
      if ( update )
        this->layered_costmap_->resizeMap( width, height, resolution, origin_x, origin_y );
    }
    // LAYERS
    const XmlRpc::XmlRpcValue& vec ( xml["layers"] );
    std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
    bool enabled;
    {
      boost::unique_lock<Costmap2D::mutex_t> lock(*(this->getCostmap()->getMutex()));
      for ( boost::shared_ptr<Layer> & plugin : * plugins ) {
        const std::string& name ( plugin->getName() );
        enabled=false;
        for ( int i=0; i<vec.size(); ++i ) {
          const std::string& layer_name ( vec[i] );
          if ( ( name.find(layer_name) + layer_name.size() ) == name.size() ) {
            enabled=true;
            break;
          }
        }
        plugin->setEnabled(enabled);
      }
    }
    // FOOTPRINT
    {
      bool update = false;
      if ( xml.hasMember("footprint_padding") )
        this->footprint_padding_ = to_double( xml["footprint_padding"] );
      if ( xml.hasMember("footprint") ) {
        XmlRpc::XmlRpcValue& footprint ( xml["footprint"] );
        std::vector<geometry_msgs::Point> point_vector;
        switch ( footprint.getType() ) {
          case XmlRpc::XmlRpcValue::TypeString:
            if ( makeFootprintFromString( footprint, point_vector ) )
              this->setUnpaddedRobotFootprint( point_vector );
            break;
          case XmlRpc::XmlRpcValue::TypeArray:
            if ( footprint.size() > 2 )
              this->setUnpaddedRobotFootprint( makeFootprintFromXMLRPC( footprint, "footprint" ) );
            break;
          default:
            break;
        }
      } else if ( xml.hasMember("robot_radius") ) {
        this->setUnpaddedRobotFootprint( makeFootprintFromRadius( to_double( xml["robot_radius"] ) ) );
      } else if ( update ) {
        this->setUnpaddedRobotFootprint( this->unpadded_footprint_ );
      }
    }
    // THREADING
    this->map_update_thread_shutdown_ = false;
    if ( xml.hasMember("transform_tolerance") && this->transform_tolerance_ != to_double(xml["transform_tolerance"]) ) {
      this->transform_tolerance_ = to_double(xml["transform_tolerance"]); }
    if ( xml.hasMember("publish_frequency") && this->publish_cycle.toSec() != 1.0/to_double(xml["publish_frequency"]) ) {
      this->publish_cycle.fromSec( 1.0 / to_double(xml["publish_frequency"]) ); }
    if ( xml.hasMember("update_frequency") && to_double(xml["update_frequency"]) > 0.0 ) {
      this->map_update_thread_ =
          new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, to_double(xml["update_frequency"]))); }
    return true;
  };

protected:
  LayeredCostmap* layered_costmap_;
  std::string name_;
  tf2_ros::Buffer& tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

private:
  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);

  void loadOldParameters(ros::NodeHandle& nh);
  void warnForOldParameters(ros::NodeHandle& nh);
  void checkOldParam(ros::NodeHandle& nh, const std::string &param_name);
  void copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  void movementCB(const ros::TimerEvent &event);
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
  ros::Time last_publish_;
  ros::Duration publish_cycle;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  Costmap2DPublisher* publisher_;
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;

  boost::recursive_mutex configuration_mutex_;

  ros::Subscriber footprint_sub_;
  ros::Publisher footprint_pub_;
  std::vector<geometry_msgs::Point> unpadded_footprint_;
  std::vector<geometry_msgs::Point> padded_footprint_;
  float footprint_padding_;
  costmap_2d::Costmap2DConfig old_config_;
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
