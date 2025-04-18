#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <signal.h>
#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "tutorial_interfaces/msg/num.hpp"
#include "tutorial_interfaces/msg/numscan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <conio.h>
/*#include "camera.hpp"
#include "vision.hpp"*/
using namespace std;
using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)
#define Xf 23
#define Xfb 23
#define DISTANCE 50

using std::placeholders::_1;
using namespace std::chrono_literals;

class RPLidarScanSubscriber : public rclcpp::Node
{
public:
    RPLidarScanSubscriber() : Node("rplidar_scan_subscriber")
    {   
        //scan_sub topic
        auto qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());
        rplidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos_profile, std::bind(&RPLidarScanSubscriber::ScanCallback, this, _1));

        //IDL_pub topic
        publisher_ = this->create_publisher<tutorial_interfaces::msg::Numscan>("scan_error", 10);    // CHANGE
        timer_ = this->create_wall_timer(5ms, std::bind(&RPLidarScanSubscriber::timer_callback, this));

        //IDL_sub topic
        vision_subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(          // CHANGE
            "vision_flag", 10, std::bind(&RPLidarScanSubscriber::topic_callback_vision, this, _1));
    }
private:
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        Mat lidar(600, 600, CV_8UC1, Scalar(255));
        vector <double> distance;
        vector <Point2d> distance_pt;
        double mindistance, pt_distance;
        int min_index;
        
        Point min_center;
        
        line(lidar, Point(lidar.cols / 2, lidar.rows), Point(lidar.cols / 2, 0), 0);
        line(lidar, Point(0, lidar.rows / 2), Point(lidar.cols, lidar.rows / 2), 0);

        int count = scan->scan_time / scan->time_increment;

        cvtColor(lidar, lidar, COLOR_GRAY2BGR);
        for (int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            float range = scan->ranges[i];
            double range_x, range_y;
            range_x = lidar.cols / 2 - ((range / 3) * cos(degree * M_PI / 180.) * (lidar.cols / 2)); //3m
            range_y = lidar.rows / 2 - ((range / 3) * sin(degree * M_PI / 180.) * (lidar.rows / 2)); //3m

            pt_distance = sqrt(pow(range_x - lidar.cols / 2, 2) + pow(range_y - lidar.rows / 2, 2));
            Point2i pt = Point2i(range_x, range_y);
            distance.push_back(pt_distance);
            distance_pt.push_back(pt);
            circle(lidar, Point(range_y, range_x), 1, Scalar(0, 0, 255), -1);
        }
        mindistance = *min_element(distance.begin(), distance.end());
        min_index = min_element(distance.begin(), distance.end()) - distance.begin();
        min_center = distance_pt[min_index];
        cen = sqrt(pow(min_center.x - lidar.rows / 2, 2) + pow(min_center.y - lidar.cols / 2, 2));
        y = lidar.cols/2-min_center.x;  //최소거리점의 Y좌표
        Xc = min_center.y - lidar.rows / 2;   //최소거리점의 X좌표
        

        cout << "min_center: " << Point(Xc, y) << "DISTANCE: " << cen << endl;
        cout << endl;

        if (!flag_out) {
            if (!flag_in) { //mode0
                if (cen <= DISTANCE){
                    //count++;
                    flag = true; //true --> left //DISTANCE 35cm
                    flag_in = true;
                }
            }
        }
        else { 
            if ((cen >= 26&&y<=-15) && flag_in) {//mode 1
                flag_out = true;
                flag = true;
            }
        }
    }
    void topic_callback_vision(const tutorial_interfaces::msg::Numscan::SharedPtr msg) {
        change = msg->flag;
    }
    void timer_callback()
    {
        auto message = tutorial_interfaces::msg::Numscan();
        if (flag&&!flag_out) {
            error = (Xf - Xc); // y값 넣어서 오류 조정
            if (change) {
                flag = false; //mode 0
                flag_out = true;
            }
        }
        else if (flag && flag_out) {
            error = Xc - 2 * Xfb; // y값 넣어서 오류 조정
            if (change) {
                flag_out = false;
                flag_in = false;
                flag = false;
            }
        }
        message.num = error;      
        message.flag = flag;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_interfaces::msg::Numscan>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rplidar_subscriber_;
    rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr vision_subscription_;
    int error, Xc, y, cen, count = 0;
    bool flag = false, flag_in = false, flag_out = false, change = false;
    double dexl = 0,dxel1=0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPLidarScanSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

