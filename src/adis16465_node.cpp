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

// プレースホルダーのナームスペース下の要素を使う
using namespace std::placeholders;
// chrono_literalsのネームスペース下の要素を使う
using namespace std::chrono_literals;

// IMUノードのクラスを定義する（Nodeクラスを継承する）
class ImuNode : public rclcpp::Node
{
  public:

    // クラスのコンストラクターを定義する
    ImuNode(rclcpp::NodeOptions options) : Node("imu", options)
    {
      // Nodeのパラメータを宣言する(パラメータに関する説明はLaunchファイルで行う)
      // Declare parameters
      this->declare_parameter("device", "/dev/ttyACM0");
      this->declare_parameter("frame_id", "imu");
      this->declare_parameter("burst_mode", false);
      this->declare_parameter("publish_temperature", false);
      this->declare_parameter("rate", 100.0);

      // パラメータを取得する
      // Get parameters
      device_param_ = this->get_parameter("device");
      frame_id_param_ = this->get_parameter("frame_id");
      burst_mode_param_ = this->get_parameter("burst_mode");
      publish_temperature_param_ = this->get_parameter("publish_temperature");
      rate_param_ = this->get_parameter("rate");
      
      // ROSのログを出力する
      RCLCPP_INFO(this->get_logger(), "device: %s", device_param_.as_string().c_str());
      RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_param_.as_string().c_str());
      RCLCPP_INFO(this->get_logger(), "rate: %f", rate_param_.as_double());
      RCLCPP_INFO(this->get_logger(), "burst_mode: %s", (burst_mode_param_.as_bool() ? "true" : "false"));
      RCLCPP_INFO(this->get_logger(), "publish_temperature: %s", (publish_temperature_param_.as_bool() ? "true" : "false"));

      // Define Publishers and Services
      // パブリッシャーとサブスクライバーを定義する
      // IMUの生データを送信するパブリッシャー
      this->imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("data_raw", 100);
      if (this->publish_temperature_param_.as_bool())
      {
        // IMUの温度情報を送信するパブリッシャー
        this->temp_data_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 100);
      }

      // Define Services
      // Bias estimate service
      // 偏差を計測するサービス
      this->bias_service_ = this->create_service<std_srvs::srv::Trigger>("bias_estimate", std::bind(&ImuNode::bias_estimate_, this, _1, _2));
      // Reset service
      // IMUをリセットするサービス
      this->reset_service_ = this->create_service<std_srvs::srv::Trigger>("reset", std::bind(&ImuNode::reset_, this, _1, _2));

      // Define the timers
      // Open port with Timer
      // IMUのポートに接続するCallback関数のタイマー
      this->imu_port_open_timer_ = this->create_wall_timer(1.0s, std::bind(&ImuNode::open_port_callback_, this));
      // Publish with Timer
      // 情報を送信するCallback関数のタイマー
      this->imu_data_pub_timer_ = this->create_wall_timer(1.0s / this->rate_param_.as_double(), std::bind(&ImuNode::pub_callback_, this));

    }
    
    // デストラクタ
    ~ImuNode()
    {
      // ポートとの接続を切る
      this->imu_.closePort();
      // printf("imu port is closed.");
    }

  private:

    // Imu Instance
    // IMUのインスタンスを宣言する
    Adis16465 imu_;

    // Parameters
    // パラメータのインスタンスを宣言する
    rclcpp::Parameter device_param_;
    rclcpp::Parameter frame_id_param_;
    rclcpp::Parameter burst_mode_param_;
    rclcpp::Parameter publish_temperature_param_;
    rclcpp::Parameter rate_param_;

    // Publishers
    // パブリッシャーのインスタンスを宣言する
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_data_publisher_;

    // Services
    // サービスのインスタンスを宣言する
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bias_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    // Timers
    // タイマーを宣言する
    rclcpp::TimerBase::SharedPtr imu_data_pub_timer_;
    rclcpp::TimerBase::SharedPtr imu_port_open_timer_;

    // Callbacks
    // バイアス推定を行う関数
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

    // IMUをリセットする関数
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

    // データを送信するCallback関数
    bool pub_callback_()
    {
      // シリアルの接続状態を確認
      if(!this->is_opened_())return false;

      // burstのデータ通信方式の場合
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
      // IMUの温度情報を送信する場合
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
      // burst通信方式の場合かつIMUの温度情報を送信する場合
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
      // 普通の通信方式の場合
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

    // IMUのシリアルに接続するCallback関数
    bool open_port_callback_()
    {
      // シリアルの接続状態を確認する
      if(this->is_opened_())return true;
      // 接続を試みる
      this->open_();
      // 接続できなかった場合
      if(!this->is_opened_())
      {
        RCLCPP_WARN(this->get_logger(), "Keep trying to open the device in 1 second period...");
        return false;
      }
      else return true;
    }


    // Publish Data
    // IMUの生データをROSインターフェイスの形に作り直し、ROSメッセージを送信する関数
    void publish_imu_data_()
    {
      // ROSのデフォルトのIMUメッセージインタフェース
      sensor_msgs::msg::Imu data;
      // IMUの座標ID
      data.header.frame_id = frame_id_param_.as_string();
      // 時間スタンプ
      data.header.stamp = this->now();

      // Linear acceleration
      // 線加速度、IMUの加速度センサから取得する
      data.linear_acceleration.x = this->imu_.accl[0];
      data.linear_acceleration.y = this->imu_.accl[1];
      data.linear_acceleration.z = this->imu_.accl[2];

      // Angular velocity
      // 角速度、IMUのジャイロセンサから取得する
      data.angular_velocity.x = this->imu_.gyro[0];
      data.angular_velocity.y = this->imu_.gyro[1];
      data.angular_velocity.z = this->imu_.gyro[2];

      // Orientation (not provided)
      // 姿勢を表す四元数、ADIS16465は6軸センサのため、姿勢に関する情報が提供されていない。9軸のIMUセンサの場合、地磁気センサのデータを取得し、姿勢のデータも設定できる。
      data.orientation.x = 0;
      data.orientation.y = 0;
      data.orientation.z = 0;
      data.orientation.w = 1;

      // ROSのパブリッシャーAPIを使用してデータを送信する
      this->imu_data_publisher_->publish(data);    
    }

    // IMUの温度を送信する関数
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
    // IMUの接続しているかどうかを確信する関数
    bool is_opened_(void)
    {
      return ((this->imu_.fd_ >= 0) ? true : false);
    }

    /**
     * @brief Open IMU device file
     */
    // ポートに接続する関数
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
  // ROSのAPIを初期化する
  // argcはターミナルにおける引数の数、例：ros2 run turtlesim turtlesim_nodeの場合、この数は2である(runから数える)
  // argvはターミナルにおける引数の文字列を収納する配列、上記の例で説明すると、argvの中に['turtlesim', 'turtlesim_node']が入っていることになる
  rclcpp::init(argc, argv);

  // Nodeに関するオプションのAPIインスタンスを初期化する
  rclcpp::NodeOptions options;
  // ROSパラメータに関する設定
  options.allow_undeclared_parameters(false);
  options.automatically_declare_parameters_from_overrides(false);

  // spin関数はプログラムの処理をここでブロックする関数
  // 初期化したNodeのインスタンスのポインターをspin関数の引数としてspin関数に渡す
  rclcpp::spin(std::make_shared<ImuNode>(options));

  // ROSのAPIをシャットダウンする
  rclcpp::shutdown();
  
  return 0;
}
