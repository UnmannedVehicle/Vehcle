/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */
#include "rclcpp/rclcpp.hpp"
#include <osm_planner/osm_planner.h>
#include <osm_planner/osm_localization.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mymsgs/srv/new_target.hpp>
#include <iostream>
#include <string>
#include <osm_planner/CoorConv.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/msg/odometry.hpp>
rclcpp::Node::SharedPtr n; 
int zone = 49;
bool pose_init = false;
using namespace std;
double x,y;
double x_now = 0.0,y_now = 0.0;
double bais_x,bais_y;
//extern std::shared_ptr<Localization>  localization_source_;
osm_planner::Planner plan;
osm_planner::Parser parser;
//std::shared_ptr<osm_planner::Parser> map = std::make_shared<osm_planner::Parser>(); 
   
//osm_planner::Localization localization(map, "source");
 
    bool makePlanCallback(const std::shared_ptr<mymsgs::srv::NewTarget::Request> req,std::shared_ptr<mymsgs::srv::NewTarget::Response> res) 
    {
    	RCLCPP_INFO(n->get_logger(),"plan");
        //boost::shared_ptr<const nav_msgs::Odometry> odom = ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic_, ros::Duration(3));


        res->result = plan.makePlan(req->latitude, req->longitude);
        return true;
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
    {
    	RCLCPP_INFO(n->get_logger(),"gps_init");
    	UTMCoor xy;
    	LatLonToUTMXY (DegToRad(msg->latitude),DegToRad(msg->longitude),int(msg->longitude/6.0)+31,xy);
        zone = int(msg->longitude/6.0)+31;
    	x = xy.x;
        y = xy.y; 
        x_now = 0.0;
        y_now = 0.0; 
    	plan.setPosition(msg->latitude,msg->longitude,0.0,true);
    	pose_init = true;
        //localization.setPositionFromGPS(msg->latitude,msg->longitude,msg->altitude,true);
    }

    void utmCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
    	RCLCPP_INFO(n->get_logger(),"utm_init");
    	x = msg->pose.position.x;
    	y = msg->pose.position.y;  
    	zone = int(msg->pose.position.z);
    	double lat;
    	double lon;
    	double roll, pitch, yaw;//定义存储r\p\y的容器
    	geometry_msgs::msg::Quaternion orientation;
    	orientation = msg->pose.orientation;
    	tf2::Quaternion quat(orientation.x,orientation.y,orientation.w,orientation.z);

    	tf2::Matrix3x3 m(quat);
    	m.getRPY(roll, pitch, yaw);//进行转换

    	WGS84Corr latlon;
    	UTMXYToLatLon (x,y,zone,false,latlon);
    	lat = RadToDeg(latlon.lat);
    	lon = RadToDeg(latlon.log);
    	plan.setPosition(lat,lon,yaw,true);
    	pose_init = true;
        //localization.setPositionFromGPS(msg->latitude,msg->longitude,msg->altitude,true);
    }

	void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) 
	{    
		int num = msg->poses.size();
		
		x_now = -msg->poses[num-1].pose.position.y;
		y_now = msg->poses[num-1].pose.position.x;
		double lat;
    	double lon;
    	x = x+x_now;
    	y = y+y_now;
    	WGS84Corr latlon;    	
    	UTMXYToLatLon (x,y,zone,false,latlon);
    	lat = RadToDeg(latlon.lat);
    	lon = RadToDeg(latlon.log);
    	plan.setPosition(lat,lon,0.0,true);
		RCLCPP_INFO(n->get_logger(),"lat_now:%f,lon_now:%f",lat,lon);
	}

	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
	{    		
		x_now = msg->pose.pose.position.x;
		y_now = msg->pose.pose.position.y;
		double lat;
    	double lon;
    	double x_init = x+x_now;
    	double y_init = y+y_now;
    	
    	WGS84Corr latlon;    	
    	UTMXYToLatLon (x_init,y_init,zone,false,latlon);
    	lat = RadToDeg(latlon.lat);
    	lon = RadToDeg(latlon.log);
    	if(pose_init)
    		plan.setPosition(lat,lon,0.0,true);
		//RCLCPP_INFO(n->get_logger(),"lat_now:%f,lon_now:%f",lat,lon);
	}
	

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    n = rclcpp::Node::make_shared("test_osm") ;
    rclcpp::Clock clock;
	
    n->declare_parameter("osm_map_path");
    n->declare_parameter("filter_of_ways");
    n->declare_parameter("origin_latitude");
    n->declare_parameter("origin_longitude");
    n->declare_parameter("interpolation_max_distance");
    
    
    double origin_lat, origin_lon;
    n->get_parameter("origin_latitude",origin_lat);
    n->get_parameter("origin_longitude",origin_lon);
    UTMCoor xy;
    LatLonToUTMXY (DegToRad(origin_lat),DegToRad(origin_lon),int(origin_lon/6.0)+31,xy);
    bais_x = xy.x;
    bais_y = xy.y;        
            
    
    std::string file = "map.osm";
    n->get_parameter("osm_map_path",file);
    
    
    
    std::vector<std::string> types_of_ways ;
    types_of_ways.push_back("footway"); 
    types_of_ways.push_back("secondary"); 	
    n->get_parameter("filter_of_ways",types_of_ways);
    

   
    double interpolation_max_distance = 2.0;
    n->get_parameter("interpolation_max_distance",interpolation_max_distance);
    
    
    
    plan.initialize(n, origin_lat, origin_lon, file);
    

    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr utm_sub;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	
	path_sub = n->create_subscription<nav_msgs::msg::Path>("shortest_path",1,pathCallback);
    gps_sub = n->create_subscription<sensor_msgs::msg::NavSatFix>("init_gps",1,&gpsCallback);
    utm_sub = n->create_subscription<geometry_msgs::msg::PoseStamped>("init_utm",1,&utmCallback);
    odom_sub = n->create_subscription<nav_msgs::msg::Odometry>("odom",1,odomCallback);
    
    rclcpp::Service<mymsgs::srv::NewTarget>::SharedPtr service =n->create_service<mymsgs::srv::NewTarget>("make_plan", &makePlanCallback);
    
    tf2_ros::TransformBroadcaster br(n);
    tf2::Quaternion quaternion,q;  
    quaternion.setRPY(0,0,3.175/2);
    q.setRPY(0,0,0);
    auto orientation_ = tf2::toMsg(quaternion);  
    auto orien = tf2::toMsg(q);  
    geometry_msgs::msg::TransformStamped trans;		
	trans.header.frame_id = "world";
    trans.child_frame_id = "map";
	trans.transform.translation.x = bais_x;
	trans.transform.translation.y = bais_y;
	trans.transform.translation.z = 0.0;
	trans.transform.rotation = orien;
	
    while (rclcpp::ok()) 
    {
        geometry_msgs::msg::TransformStamped transform;
		transform.header.stamp = clock.now();
		trans.header.stamp = clock.now();
		transform.header.frame_id = "map";
    	transform.child_frame_id = "base";
		transform.transform.translation.x = x - bais_x;
		transform.transform.translation.y = y - bais_y;
		transform.transform.translation.z = 0.0;
		transform.transform.rotation = orientation_;
		br.sendTransform(transform);
		br.sendTransform(trans);
        rclcpp::spin_some(n);      
    }
    return 0;
}


