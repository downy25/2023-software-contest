#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include "tutorial_interfaces/msg/numscan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <conio.h>
#define GAIN 0.7
#define BREAK_GAIN 0.5
#define INIT_SPEED 80
#define BREAK_SPEED 60
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

class lanechange : public rclcpp::Node
{
public:
    lanechange() : Node("lanechange")
    {
        //scan_sub topic
        scan_subscription_ = this->create_subscription<tutorial_interfaces::msg::Numscan>(         
            "scan_error", 10, std::bind(&lanechange::topic_callback_scan, this, _1));

        //lanefollow_sub topic
        vision_subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(         
            "vision_error", 10, std::bind(&lanechange::topic_callback_vision, this, _1));

        //dynamixel_pub topic
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
        timer_ = this->create_wall_timer(5ms, std::bind(&lanechange::publish_velcmd_msg, this));

        //vision_pub flag topic
        vision_publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("vision_flag", 10);    
        timer_ = this->create_wall_timer(5ms, std::bind(&lanechange::publish_vision_flag_msg, this));
    }
private:
     void topic_callback_scan(const tutorial_interfaces::msg::Numscan::SharedPtr msg) {
        scan_error = msg->num; //scan error
        flag_scan = msg->flag; //scan flag
     }
     void topic_callback_vision(const tutorial_interfaces::msg::Num::SharedPtr msg)      
     {
         lanefollow_error = msg->num; //lanefollow error
         lanefollow_count = msg->count; //curve or linear
         flag_lanefollow = msg->flag; 
         flag_object = msg->flag_object; //find object
     }
     void publish_velcmd_msg() {
         static auto msg = geometry_msgs::msg::Twist();

         //mode
         switch (mode) {
         case 0:  //init lanefollow
             msg.linear.x = INIT_SPEED - lanefollow_error * GAIN;
             msg.linear.y = -(INIT_SPEED + lanefollow_error * GAIN);
             change = false; //�ٽõ��ƿͼ� �ʱ�ȭ
             cout << "mode0_lanefollow_error: " << lanefollow_error << endl;
             if (flag_scan && flag_object) mode = 1; //scan data find_object
             break;
         case 1:
             
         case 2:
             msg.linear.x = BREAK_SPEED - scan_error * BREAK_GAIN;
             msg.linear.y = -(BREAK_SPEED + scan_error * BREAK_GAIN);
             cout << "mode2_scan_error: " << scan_error << endl;
             if (flag_scan && flag_object) change = true;
             else if (!flag_scan && flag_object) {
                 mode = 3;
                 change = false;
             }
             break;
         case 3:
             msg.linear.x = INIT_SPEED - lanefollow_error * GAIN;
             msg.linear.y = -(INIT_SPEED + lanefollow_error * GAIN);
             change = false;
             cout << "mode3_lanefollow_error: " << lanefollow_error << endl;
             if (flag_scan && flag_object)mode = 4; //scan data find_object
             break;
         case 4:
             msg.linear.x = INIT_SPEED - scan_error * GAIN;
             msg.linear.y = -(INIT_SPEED + scan_error * GAIN);
             cout << "mode4_scan_error: " << scan_error << endl;
             if (flag_scan && !flag_object)mode = 5; //scan data not find_object
             break;
         case 5:
             msg.linear.x = INIT_SPEED - scan_error * GAIN;
             msg.linear.y = -(INIT_SPEED + scan_error * GAIN);
             cout << "mode5_scan_error: " << scan_error << endl;
             if (flag_scan && flag_object) {
                 mode = 0; //scan data not find_object
                 change = true;
             }
             break;
         case 6:
             msg.linear.x = (pwm_l + 3) - lanefollow_error;
             msg.linear.y = -(pwm_r + 3) + lanefollow_error;
             cout << "flag_check: " << flag_check << endl;
             cout << "pwm_l : " << msg.linear.x <<"pwm_r : "<< msg.linear.y <<"error: "<<lanefollow_error<< endl;
             if (flag_check)mode = 1;
             break;
         }
         dynamixel_publisher_->publish(msg);
     }
     void publish_vision_flag_msg() {
         static auto msg = tutorial_interfaces::msg::Num();
         msg.flag = change;
         vision_publisher_->publish(msg);
     }
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_; //dynamixel pub
     rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr vision_publisher_; //vision flag pub
     rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr vision_subscription_; //vision data sub       
     rclcpp::Subscription<tutorial_interfaces::msg::Numscan>::SharedPtr scan_subscription_; //scan data sub
     int lanefollow_error, scan_error, mode = 0, count = 0, lanefollow_count;
     bool flag_lanefollow = false, flag_scan = false, flag_object = false, change = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lanechange>());
  rclcpp::shutdown();
  return 0;
}


//test linear distance
         /*
         static bool flag = false;

         if (lanefollow_count == 180&&!flag) {
             msg.linear.x = 0;
             msg.linear.y = 0;
             flag = true;
         }

         if (flag) {
             msg.linear.x = 0;
             msg.linear.y = 0;
         }
         else {
             msg.linear.x = INIT_SPEED - lanefollow_error * GAIN;
             msg.linear.y = -(INIT_SPEED + lanefollow_error * GAIN);
         }*/