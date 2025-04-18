#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <conio.h>
#include <functional>
#include <memory>
#include <cstdio>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "lanefollow_ros2/camera.hpp"
#include "lanefollow_ros2/vision.hpp"
#include "tutorial_interfaces/msg/num.hpp" //IDL
using namespace cv;
using namespace std;
using namespace dahun;

using std::placeholders::_1;

using namespace std::chrono_literals;

class lanefollowwindow : public rclcpp::Node
{
public:
    lanefollowwindow() : Node("lanefollow")
    {   
        //camera
        size_t depth = rmw_qos_profile_default.depth;
        rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
        rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
        reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile.reliability(reliability_policy);
        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image1", qos_profile,
            std::bind(&lanefollowwindow::show_image, this, _1));

        //error pub
        vision_publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("vision_error", 10);    // CHANGE
        timer_ = this->create_wall_timer(
            30ms, std::bind(&lanefollowwindow::timer_callback, this));
    }

private:
    void show_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str());

        // Convert to an OpenCV matrix by assigning the data.
        cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char*>(msg->data.data()), msg->step);
        if (msg->encoding == "rgb8") {
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        }

        cv::Mat cvframe = frame;
        Mat Roi_L, Roi_R, frame1, Roi_L1, Roi_R1;
        Mat labels, stats, centroids;
        Point2d crnt_ptl, crnt_ptr; //roi 4/3 ~ 
        static Point2d prev_ptR(70, 15), prev_ptL(70, 15); //roi 4/3 ~ 
        Point2d center_r(0, 15), center_l(140, 15); //roi 4/3 ~  °íÁ¤
        Image_Pretreatment(frame);
        Roi_L = frame(Rect(0, 0, frame.cols / 2, frame.rows));
        Roi_R = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        Point2d cpt_L = find_object(labels, stats, centroids, Roi_L, prev_ptL, crnt_ptl);
        Point2d cpt_R = find_object(labels, stats, centroids, Roi_R, prev_ptR, crnt_ptr);
        int error_l = get_error(cpt_L, center_l);
        int error_R = get_error(cpt_R, center_r);
        error = error_l + error_R;
        line_Catch(prev_ptL, crnt_ptl, prev_ptR, crnt_ptr, flag_object);
        Cruve_Checker(i, error, CruveCheck);
        cvtColor(frame, frame, COLOR_GRAY2BGR);
        imshow("roi_r", Roi_R);
        imshow("roi_l", Roi_L);
        waitKey(5);
    }
    void timer_callback()
    {
        auto message = tutorial_interfaces::msg::Num();   
        message.num = error; //lanefollow error
        message.flag = CruveCheck;  //Curve Check
        message.flag_object = flag_object; //flag_object
        message.count = i; //count
        vision_publisher_->publish(message); //vision_pub_msg 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr vision_publisher_;  //vision_pub
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;  //camera_sub
    int error, i = 0;
    bool CruveCheck = false, flag_object = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lanefollowwindow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}