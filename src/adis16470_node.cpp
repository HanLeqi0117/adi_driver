// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "adi_driver/adis16465.h"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node
{
  public:

    ImuNode(rclcpp::NodeOptions options) : Node("imu", options)
    {
      // Declare parameters
      this->declare_parameter("device", "/dev/ttyACM0");
      this->declare_parameter("frame_id", "imu");
      this->declare_parameter("burst_mode", false);
      this->declare_parameter("publish_temperature", false);
      this->declare_parameter("rate", 100.0);

      // Get parameters
      device_param_ = this->get_parameter("device");
      frame_id_param_ = this->get_parameter("frame_id");
      burst_mode_param_ = this->get_parameter("burst_mode");
      publish_temperature_param_ = this->get_parameter("publish_temperature");
      rate_param_ = this->get_parameter("rate");
      
      RCLCPP_INFO(this->get_logger(), "device: %s", device_param_.as_string().c_str());
      RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_param_.as_string().c_str());
      RCLCPP_INFO(this->get_logger(), "rate: %f", rate_param_.as_double());
      RCLCPP_INFO(this->get_logger(), "burst_mode: %s", (burst_mode_param_.as_bool() ? "true" : "false"));
      RCLCPP_INFO(this->get_logger(), "publish_temperature: %s", (publish_temperature_param_.as_bool() ? "true" : "false"));

      // Define Publishers and Services
      this->imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("data_raw", 100);
      if (this->publish_temperature_param_.as_bool())
      {      
        this->temp_data_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 100);
      }

      // Bias estimate service
      this->bias_service_ = this->create_service<std_srvs::srv::Trigger>("bias_estimate", std::bind(&ImuNode::bias_estimate_, this, _1, _2));

      // Reset service
      this->reset_service_ = this->create_service<std_srvs::srv::Trigger>("reset", std::bind(&ImuNode::reset_, this, _1, _2));

      // Open port with Timer
      this->imu_port_open_timer_ = this->create_wall_timer(1.0s, std::bind(&ImuNode::open_port_callback_, this));

      // Publish with Timer
      this->imu_data_pub_timer_ = this->create_wall_timer(1.0s / this->rate_param_.as_double(), std::bind(&ImuNode::pub_callback_, this));

    }

    ~ImuNode()
    {
      this->imu_.closePort();
      // printf("imu port is closed.");
    }

  private:

    // Imu Instance
    Adis16465 imu_;

    // Parameters
    rclcpp::Parameter device_param_;
    rclcpp::Parameter frame_id_param_;
    rclcpp::Parameter burst_mode_param_;
    rclcpp::Parameter publish_temperature_param_;
    rclcpp::Parameter rate_param_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_data_publisher_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bias_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    // Timers
    rclcpp::TimerBase::SharedPtr imu_data_pub_timer_;
    rclcpp::TimerBase::SharedPtr imu_port_open_timer_;

    // Callbacks
    bool bias_estimate_ (std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
      (void) req;

      RCLCPP_INFO(this->get_logger(), "bias_estimate");
      if (imu_.bias_correction_update() < 0)
      {
        res->success = false;
        res->message = "Bias correction update failed";
        return false;
      }
      res->success = true;
      res->message = "Success";
      return true;
    }

    bool reset_(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                std_srvs::srv::Trigger::Response::SharedPtr res)
    {
      (void) req;

      RCLCPP_INFO(this->get_logger(), "reset");
      if (imu_.reset() < 0)
      {
        res->success = false;
        res->message = "failed to reset!";
        return false;
      }
      res->success = true;
      res->message = "Success";
      return true;    
    }

    bool pub_callback_()
    {
      if(!this->is_opened_())return false;

      if (this->burst_mode_param_.as_bool())
      {
        if (this->imu_.update_burst() == 0)
        {
          this->publish_imu_data_();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update burst");
        }
      }
      else if (this->publish_temperature_param_.as_bool())
      {
        if (this->imu_.update() == 0)
        {
          this->publish_imu_data_();
          this->publish_temp_data_();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update");
        }
      }
      else if (this->burst_mode_param_.as_bool() && this->publish_temperature_param_.as_bool())
      {
        if (this->imu_.update_burst() == 0)
        {
          this->publish_imu_data_();
          this->publish_temp_data_();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update burst");
        }
      }
      else
      {
        if (this->imu_.update() == 0)
        {
          this->publish_imu_data_();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update");
        }
      }
      return true;
    }

    bool open_port_callback_()
    {
      if(this->is_opened_())return true;
      this->open_();
      if(!this->is_opened_())
      {
        RCLCPP_WARN(this->get_logger(), "Keep trying to open the device in 1 second period...");
        return false;
      }
      else return true;
    }


    // Publish Data
    void publish_imu_data_()
    {
      sensor_msgs::msg::Imu data;
      data.header.frame_id = frame_id_param_.as_string();
      data.header.stamp = this->now();

      // Linear acceleration
      data.linear_acceleration.x = this->imu_.accl[0];
      data.linear_acceleration.y = this->imu_.accl[1];
      data.linear_acceleration.z = this->imu_.accl[2];

      // Angular velocity
      data.angular_velocity.x = this->imu_.gyro[0];
      data.angular_velocity.y = this->imu_.gyro[1];
      data.angular_velocity.z = this->imu_.gyro[2];

      // Orientation (not provided)
      data.orientation.x = 0;
      data.orientation.y = 0;
      data.orientation.z = 0;
      data.orientation.w = 1;

      this->imu_data_publisher_->publish(data);    
    }

    void publish_temp_data_()
    {
      sensor_msgs::msg::Temperature data;
      data.header.frame_id = this->frame_id_param_.as_string();
      data.header.stamp = this->now();

      // imu Temperature
      data.temperature = imu_.temp;
      data.variance = 0;
      
      this->temp_data_publisher_->publish(data);
    }

    // Port

    bool is_opened_(void)
    {
      return ((this->imu_.fd_ >= 0) ? true : false);
    }

    /**
     * @brief Open IMU device file
     */
    bool open_(void)
    {
      // Open device file
      if (this->imu_.openPort(this->device_param_.as_string()) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open device %s", this->device_param_.as_string().c_str());
      }
      rclcpp::sleep_for(10ms);// Wait 10ms for SPI ready
      int16_t pid = 0;
      this->imu_.get_product_id(pid);
      RCLCPP_INFO(this->get_logger(), "Product ID: %x", pid);
      this->imu_.set_bias_estimation_time(0x070a);  // default: 0x070a bias_on: 0x3f0a

      return true;
    }    
  };

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.automatically_declare_parameters_from_overrides(false);

  rclcpp::spin(std::make_shared<ImuNode>(options));

  rclcpp::shutdown();
  
  return 0;
}
