// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <chrono>
#include <cinttypes>
#include <math.h>

#include "rclcpp/rclcpp.hpp"


#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

using std::placeholders::_1;

class MyController : public rclcpp::Node
{
public:
  MyController()
  : Node("my_controller")
  { 
   
    client1_ = this->create_client<turtlesim::srv::Kill>("kill");
    client2_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("my_turtle/pose", 10, std::bind(&MyController::topic_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("my_turtle/cmd_vel", 10);

    while (!client1_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    while (!client2_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }    
    
    // Add a pointer for spawning request 
    this-> spawn_request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
  }

  //function to call the kill service
  void kill_turtle()
  {	
  	auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
  	kill_request->name = "turtle1";
  	auto result_future = client1_->async_send_request(kill_request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
	{
	  RCLCPP_ERROR(this->get_logger(), "Kill service call failed :(");
	}
  }

  //function to call the spawn service
  void spawn_turtle()
  {
  	auto result_future = client2_->async_send_request(spawn_request_);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
	{
	  RCLCPP_ERROR(this->get_logger(), "Kill service call failed :(");
	}
  }
  
   std::shared_ptr<turtlesim::srv::Spawn::Request> spawn_request_;

private:

  //get the robot's position and set the velocity
  void topic_callback(const turtlesim::msg::Pose::SharedPtr msg) const
  {	
  	auto vel = geometry_msgs::msg::Twist();
  	if(msg->x<2)
	{
		vel.linear.x = 1.0;
		vel.angular.z = -1.0;
	}
	else if(msg->x>9)
	{
		vel.linear.x = 1.0;
		vel.angular.z = 1.0;
	}
	else
	{
		vel.linear.x = 0.1 + 2*sin (M_PI*msg->x/7 -2*M_PI/7);;
		vel.angular.z = 0.0;
	}
	if(msg->y>9)
	{
		vel.linear.x = 0.0;
		vel.angular.z = 0.0;
	}
	publisher_->publish(vel);
	
  }
  
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client1_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client2_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<MyController>();
  
  node->spawn_request_-> x = 2.0;
  node->spawn_request_-> y = 1.0;
  node->spawn_request_-> theta = 0.0;
  node->spawn_request_-> name = "my_turtle";

  node -> kill_turtle();
  node -> spawn_turtle();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

