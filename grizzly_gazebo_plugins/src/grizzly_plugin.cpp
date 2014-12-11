/**
Software License Agreement (BSD)

\file      grizzly_plugin.cpp
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
 

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <grizzly_plugin/grizzly_plugin.h>
#include <ros/time.h>

using namespace gazebo;

// enum {FL=0, FR=1, RL= 2, RR=3, FA=4};
enum {
  FL=grizzly_msgs::Drives::FrontLeft, 
  FR=grizzly_msgs::Drives::FrontRight,
  RL=grizzly_msgs::Drives::RearLeft,
  RR=grizzly_msgs::Drives::RearRight,
  FA=4};

GrizzlyPlugin::GrizzlyPlugin()
{
}

GrizzlyPlugin::~GrizzlyPlugin()
{
  delete rosnode_;
  delete spinner_thread_;
}

void GrizzlyPlugin::FiniChild()
{
  rosnode_->shutdown();
  spinner_thread_->join();
}
    
void GrizzlyPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();

  this->node_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";


  rl_joint_name_ = "joint_rear_left_wheel";
  if (_sdf->HasElement("rearLeftJoint"))
    rl_joint_name_ = _sdf->GetElement("rearLeftJoint")->Get<std::string>();

  rr_joint_name_ = "joint_rear_right_wheel";
  if (_sdf->HasElement("rearRightJoint"))
    rr_joint_name_ = _sdf->GetElement("rearRightJoint")->Get<std::string>();

  fl_joint_name_ = "joint_front_left_wheel";
  if (_sdf->HasElement("frontLeftJoint"))
    fl_joint_name_ = _sdf->GetElement("frontLeftJoint")->Get<std::string>();

  fr_joint_name_ = "joint_front_right_wheel";
  if (_sdf->HasElement("frontRightJoint"))
    fr_joint_name_ = _sdf->GetElement("frontRightJoint")->Get<std::string>();

  fa_joint_name_ = "frontAxleJoint";
  if (_sdf->HasElement("frontAxleJoint"))
    fa_joint_name_ = _sdf->GetElement("frontAxleJoint")->Get<std::string>();

  torque_ = 15.0;
  if (_sdf->HasElement("torque"))
    torque_ = _sdf->GetElement("torque")->Get<double>();

  base_geom_name_ = "base_link";
  if (_sdf->HasElement("baseGeom"))
    base_geom_name_ = _sdf->GetElement("baseGeom")->Get<std::string>();
  base_geom_ = model_->GetChildCollision(base_geom_name_);


  //base_geom_->SetContactsEnabled(true);

  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");
  gzdbg << "plugin model name: " << modelName << "\n";

  js_.name.push_back( fl_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( fr_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( rl_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( rr_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( fa_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  prev_update_time_ = 0;
  last_cmd_vel_time_ = 0;

  wheel_ang_vel_.rear_left = 0.0;
  wheel_ang_vel_.rear_right= 0.0;
  wheel_ang_vel_.front_left = 0.0;
  wheel_ang_vel_.front_left = 0.0;

  set_joints_[0] = false;
  set_joints_[1] = false;
  set_joints_[2] = false;
  set_joints_[3] = false;
  set_joints_[4] = false;

  //TODO: fix this
  joints_[RL] = model_->GetJoint(rl_joint_name_);
  joints_[RR] = model_->GetJoint(rr_joint_name_);
  joints_[FL] = model_->GetJoint(fl_joint_name_);
  joints_[FR] = model_->GetJoint(fr_joint_name_);
  joints_[FA] = model_->GetJoint(fa_joint_name_);

  if (joints_[RL]) set_joints_[RL] = true;
  if (joints_[RR]) set_joints_[RR] = true;
  if (joints_[FL]) set_joints_[FL] = true;
  if (joints_[FR]) set_joints_[FR] = true;
  if (joints_[FA]) set_joints_[FA] = true;

  //initialize time and odometry position
  prev_update_time_ = last_cmd_vel_time_ = this->world_->GetSimTime();

  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo_grizzly", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  drive_sub_ = rosnode_->subscribe("cmd_drive", 1, &GrizzlyPlugin::OnDrive, this );
  encoder_pub_  = rosnode_->advertise<grizzly_msgs::Drive>("motors/encoders", 1);
  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &GrizzlyPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GrizzlyPlugin::UpdateChild, this));
}


void GrizzlyPlugin::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;
  grizzly_msgs::Drive encoder_msg;

  // set joint velocity based on teleop input and publish joint states
  js_.header.stamp.sec = time_now.sec;
  js_.header.stamp.nsec = time_now.nsec;
  if (set_joints_[RL])
  {
    joints_[RL]->SetVelocity( 0, wheel_ang_vel_.rear_left);
    joints_[RL]->SetMaxForce( 0, torque_ );
    js_.position[RL] = joints_[RL]->GetAngle(0).Radian();
    js_.velocity[RL] = encoder_msg.rear_left = joints_[RL]->GetVelocity(0);
  }
  if (set_joints_[RR])
  {
    joints_[RR]->SetVelocity( 0, wheel_ang_vel_.rear_right);
    joints_[RR]->SetMaxForce( 0, torque_ );
    js_.position[RR] = joints_[RR]->GetAngle(0).Radian();
    js_.velocity[RR] = encoder_msg.rear_right = joints_[RR]->GetVelocity(0);
  }
  if (set_joints_[FL])
  {
    joints_[FL]->SetVelocity( 0, wheel_ang_vel_.front_left);
    joints_[FL]->SetMaxForce( 0, torque_ );
    js_.position[FL] = joints_[FL]->GetAngle(0).Radian();
    js_.velocity[FL] = encoder_msg.front_left = joints_[FL]->GetVelocity(0);
  }
  if (set_joints_[FR])
  {
    joints_[FR]->SetVelocity( 0, wheel_ang_vel_.front_right);
    joints_[FR]->SetMaxForce( 0, torque_ );
    js_.position[FR] = joints_[FR]->GetAngle(0).Radian();
    js_.velocity[FR] = encoder_msg.front_right = joints_[FR]->GetVelocity(0);
  }

  if (set_joints_[FA])
  {
    js_.position[4] = joints_[FA]->GetAngle(0).Radian();
    js_.velocity[4] = joints_[FA]->GetVelocity(0);
  }

  // publish joint states
  // joint_state_pub_.publish( js_ );

  // publish encoder message
  encoder_msg.header.stamp.sec = time_now.sec;
  encoder_msg.header.stamp.nsec = time_now.nsec;
  encoder_pub_.publish( encoder_msg );

  // Timeout if we haven't received a cmd in <0.1 s
  common::Time time_since_last_cmd = time_now - last_cmd_vel_time_;
  if (time_since_last_cmd.Double() > 0.1)
  {
    wheel_ang_vel_.rear_left = 0;
    wheel_ang_vel_.rear_right = 0;
    wheel_ang_vel_.front_left = 0;
    wheel_ang_vel_.front_right = 0;
  }
}


void GrizzlyPlugin::OnDrive( const grizzly_msgs::DriveConstPtr &msg)
{
  last_cmd_vel_time_ = this->world_->GetSimTime();  
  wheel_ang_vel_ = *msg;
}

void GrizzlyPlugin::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(GrizzlyPlugin);
