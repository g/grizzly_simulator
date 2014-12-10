/**
Software License Agreement (BSD)

\file      grizzly_plugin.h
\authors   Yan Ma <yanma@clearpathrobotics.com>, Ryan Gariepy <rgariepy@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <grizzly_msgs/Drive.h>
#include <grizzly_msgs/eigen.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


namespace gazebo
{
  class GrizzlyPlugin : public ModelPlugin
  {
    public: 
      GrizzlyPlugin();
      virtual ~GrizzlyPlugin();
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );
      
    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();
  
    private:
      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnDrive( const grizzly_msgs::DriveConstPtr &msg);

      /// Parameters
      std::string node_namespace_;
      std::string rl_joint_name_;
      std::string rr_joint_name_;
      std::string fl_joint_name_;
      std::string fr_joint_name_;
      std::string fa_joint_name_;
      std::string base_geom_name_;

      ///Torque applied to the wheels
      float torque_;

      ros::NodeHandle *rosnode_;
  
      ros::Publisher encoder_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher joint_state_pub_;
      ros::Subscriber drive_sub_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
      sensors::SensorPtr parent_sensor_;

      /// Speeds of the wheels
      grizzly_msgs::Drive wheel_ang_vel_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      bool set_joints_[5];
      physics::JointPtr joints_[5];
      physics::CollisionPtr base_geom_;

      tf::TransformBroadcaster transform_broadcaster_;
      sensor_msgs::JointState js_;

      void spin();
      boost::thread *spinner_thread_;
      event::ConnectionPtr contact_event_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };
}
#endif
