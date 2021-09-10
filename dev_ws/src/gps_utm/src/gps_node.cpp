/**
 * @file gps_node.cpp Publisher:NavSatFix，PoseStamped
 * @brief 
 * @author Xuan Tan 
 * @version 1.0
 * @date 2021-08-26
 */
#include <memory>
#include <iostream>
#include "linux/serial.h"
#include "/home/aimibot/gpslink/gpslink_sdk/include/gpslink/gpslink_interface.h"     
#include "/home/aimibot/gpslink/example/include/CoorConv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double utm_x;
double utm_y;
double head;
double x_last;
double y_last;
double heading_last;
double sum_x;
double sum_y;
double sum_heading;

UTMCoor average_xy;
double average_heading;

bool initposeflag = true;//


bool gpsinit_x_y_h(UTMCoor xy, double heading, int start_num)
{   
    static int flag = 0;
    double utm_e = 0.03;
    double heading_e = 0.3;
    if(flag > 0)
    {
        if(flag < (start_num+1))
        {
            if ((fabs(xy.x - x_last)) < utm_e && (fabs(xy.y - y_last) < utm_e) && (fabs(heading - heading_last) < heading_e))
            {
                sum_x += xy.x;
                sum_y += xy.y;
                sum_heading += heading;
                flag++;
                x_last = xy.x;
                y_last = xy.y;
                heading_last = heading;
            }
            else
            {
                flag = 0;
                sum_x = 0.0;
                sum_y = 0.0;
                sum_heading = 0.0;
            }
        }
        else if (flag == (start_num+1))
        {
            average_xy.x = sum_x / start_num;
            average_xy.y = sum_y / start_num;
            average_heading = sum_heading / start_num;
            return true;
        }
    }
    else
    {
        flag++;
        x_last = xy.x;
        y_last = xy.y;
        heading_last = heading;
    }
    return false;
}

rclcpp::Node::SharedPtr nh; 
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_fix;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_initpose;
Gpslink_Interface interface;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("gps_node");

    //nh->declare_parameter("port");
    //nh->declare_parameter("baud");
    //int baudrate = 115200;
    //std::string uart_name;
    //nh->get_parameter("port",uart_name);
    //nh->get_parameter("baud",baudrate);	
    //const char* uart = uart_name.data();
    //interface.Set_Serial(uart,baudrate);

    rclcpp::Clock clock;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_fix;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_initpose;
    publisher_fix = nh->create_publisher<sensor_msgs::msg::NavSatFix>("gps",100);
    publisher_pose = nh->create_publisher<geometry_msgs::msg::PoseStamped>("utm_pose",100);
    publisher_initpose = nh->create_publisher<geometry_msgs::msg::PoseStamped>("init_utm",100);

    UTMCoor	uc;
    UTMCoor	utm;
    GpsKsxt_data message;//gps ksxt data
    //open the gps port
    int ret = interface.start();
    if(!ret)
    {
        cout << "open port success!" << endl;
    }
    else
    {
        cout << "open port failed! please check the port NAME or change the port permissions!" << endl;
        cout << ret << endl;
        return(0);
    }

    while(rclcpp::ok())
    {
//cout<<"hh"<<endl;
        interface.gpsksxt_data={};//Empty data cache
        bool success = interface.read_message(message);
        if(success)
        {
            if (atoi(interface.gpsksxt_data.spSat.c_str()) == 3 && (atoi(interface.gpsksxt_data.soSat.c_str()) == 3 || atoi(interface.gpsksxt_data.soSat.c_str()) == 2))
            {        
                sensor_msgs::msg::NavSatFix fix;
                fix.header.stamp = clock.now();
                fix.header.frame_id = "gps_link";
                fix.longitude = atof(interface.gpsksxt_data.Lon.c_str());
                fix.latitude =  atof(interface.gpsksxt_data.Lat.c_str());
                fix.altitude =  atof(interface.gpsksxt_data.Alt.c_str());
                publisher_fix->publish(fix);

                LatLonToUTMXY(DegToRad(atof(interface.gpsksxt_data.Lon.c_str())), DegToRad(atof(interface.gpsksxt_data.Lat.c_str())),  static_cast<int>(atof(interface.gpsksxt_data.Lat.c_str()) / 6) + 31 , uc);
               
                if (initposeflag)
                {
                    if(gpsinit_x_y_h(uc, atof(interface.gpsksxt_data.Heading.c_str()),40))
                    {
                        initposeflag = false;
                        //publisher GpsPoseStamped
                        auto pose_istamp = geometry_msgs::msg::PoseStamped();
                        pose_istamp.header.stamp = clock.now();
                        pose_istamp.header.frame_id = "initpose_link";
                        //Position
                        pose_istamp.pose.position.x = uc.x;
                        pose_istamp.pose.position.y = uc.y;
                        pose_istamp.pose.position.z = static_cast<int>(atof(interface.gpsksxt_data.Lat.c_str()) / 6) + 31;//To be confirmed？

                        tf2::Quaternion orientation;
                        orientation.setRPY(0.0, 0.0, DegToRad(atof(interface.gpsksxt_data.Heading.c_str())));
                        //orientation
                        auto orientation_ = tf2::toMsg(orientation);
                        pose_istamp.pose.orientation = orientation_;
                        publisher_initpose->publish(pose_istamp);
                        cout<<"init,"<<utm.x<<","<<utm.y<<","<<interface.gpsksxt_data.Heading<<endl;
                    }
                    else
                    {
                        cout<<"Initial value of UTM coordinate is not confirmed"<<endl;
                    }
                }
                else
                {        
                    utm.x = uc.x - average_xy.x;
                    utm.y = uc.y - average_xy.y;
                    //publisher GpsPoseStamped
                    auto pose_stamp = geometry_msgs::msg::PoseStamped();
                    pose_stamp.header.stamp = clock.now();
                    pose_stamp.header.frame_id = "utm_link";
                    //Position
                    pose_stamp.pose.position.x = utm.x;
                    pose_stamp.pose.position.y = utm.y;
                    pose_stamp.pose.position.z = static_cast<int>(atof(interface.gpsksxt_data.Lat.c_str()) / 6) + 31;//To be confirmed？
                    //orientation
                    pose_stamp.pose.orientation.x = 0.0;
                    pose_stamp.pose.orientation.y = 0.0;
                    pose_stamp.pose.orientation.z = 0.0;
                    pose_stamp.pose.orientation.w = 1.0;
                    publisher_pose->publish(pose_stamp);
                    cout<<"UTM,"<<utm.x<<","<<utm.y<<","<<interface.gpsksxt_data.Heading<<endl;
                } 
            }   
            else
            {
                cout<<"No RTK Data"<<endl;
            }
        }
        rclcpp::spin_some(nh);
    }
    return 0;
}

