#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
using namespace std;


rclcpp::Node::SharedPtr nh; 
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_pub;
bool flag_next = false;   
bool flag_new = false;
bool flag_done = true;
int num = 0, i = 0;
float x[1024],y[1024];
float x_[] = {1.0,2.0,3.0,4.0};
float y_[] = {0.0,0.0,0.0,0.0};
float goal_x, goal_y;
float init_x = 0.0 ,init_y = 0.0;
float last_x = 0.0 ,last_y = 0.0;
void initCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	init_x = msg->pose.position.x;
	init_y = msg->pose.position.y;	
}


void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) 
{  
    flag_new = false;  
    RCLCPP_INFO(nh->get_logger(), "%d",msg->poses.size());
    num = msg->poses.size();
    for(int n=0;n<=num;n++)
    {
		x[n] = msg->poses[n].pose.position.x + init_x;
		y[n] = -msg->poses[n].pose.position.y + init_y;
		
	}
    flag_new = true;
    //RCLCPP_INFO(nh->get_logger(), "%d",flag_new);
}


void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    float now_x = msg->pose.pose.position.x;
    float now_y = msg->pose.pose.position.y;
    //RCLCPP_INFO(nh->get_logger(), "now_x:%f,now_y:%f",now_x,now_y);
    static geometry_msgs::msg::PoseStamped goal;
    
    if(flag_new)
    {

    	goal_y = int((x[i] - init_x)*100.0)/100.0;
    	goal_x = int((y[i] - init_y)*100.0)/100.0;
		if(abs(goal_x - last_x) < 0.4)
			goal_x = last_x;
		if(abs(goal_y - last_y) < 0.4)
			goal_y = last_y;		
    	RCLCPP_INFO(nh->get_logger(), "x:%f,y:%f",goal_x,goal_y);
		
		goal.pose.position.x = goal_x;
		goal.pose.position.y = goal_y;
		goal.header.frame_id = "map";
		   
		path_pub->publish(goal);
		if((abs(goal_x - now_x) < 0.8) && (abs(goal_y - now_y) < 0.8))
		{
			RCLCPP_INFO(nh->get_logger(), "goal_x:%f,goal_y:%f",goal_x,goal_y);
			i = i+1;
			last_x = goal_x;
			last_y = goal_y;
    	    //flag_new = true;
    	    if (i > num - 1)
    	    {
    	    	i= 0;
    	    	flag_new = false;
    	    }
    	    	
    	}
    }
/*    
    else if(flag_next)
    {
    	goal_x = x[i];
    	goal_y = y[i];
    	flag_new = false;
    	flag_next = false;
    	
		RCLCPP_INFO(nh->get_logger(), "goal_x:%f,goal_y:%f",goal_x,goal_y);
		goal.pose.position.x = goal_x;
		goal.pose.position.y = goal_y;
		goal.header.frame_id = "map";
		path_pub->publish(goal);
    }    
    
    else if(!flag_new & !flag_next & !flag_done)
    {
    	if((abs(x[i-1] - now_x) < 0.5 && abs(y[i-1] - now_y) < 0.5))
    	{
    	    //RCLCPP_INFO(nh->get_logger(), "goal_x:%f,goal_y:%f",goal.pose.position.x,goal.pose.position.y);
	    	path_pub->publish(goal);
	    }
	    if(abs(goal_x - now_x) < 0.5 && abs(goal_y - now_y) < 0.5)
		{
			RCLCPP_INFO(nh->get_logger(), "goal_x:%f,goal_y:%f",goal_x,goal_y);
			i = i+1;
    	    flag_new = false;
    	    flag_next = true;
    	    //flag_stop = false;
    	    if(i == num)
    	    {
			    flag_new = false;
			    flag_next = false;
			    flag_done = true;    	    	
    	    }
    	}    	
    }*/
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("navigation") ;
    rclcpp::Clock clock;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_sub;
    
    
    path_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("goal",100);
    
    init_sub = nh->create_subscription<geometry_msgs::msg::PoseStamped>("init_utm",10,initCallback); 
    odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>("odom",1,odomCallback);
    path_sub = nh->create_subscription<nav_msgs::msg::Path>("shortest_path",1,pathCallback);
    num = sizeof(x) / sizeof(x[0]);
    while(rclcpp::ok())
    {
    	rclcpp::spin_some(nh);
    }

    return 0;
}
