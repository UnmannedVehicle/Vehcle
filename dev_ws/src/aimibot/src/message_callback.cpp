#include "iostream"
#include "../include/aimibot/message_callback.hpp"
#include "legacy/mavlink_interface.h"      //51
#include "../include/aimibot/aimibot.hpp"
#include "limits.h"

using namespace std;
void aimibot::subscribeGpsCommand(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    
    geometry_msgs::msg::Quaternion orientation;
    orientation = msg->pose.orientation;
    
    tf2::Quaternion quat(orientation.x,orientation.y,orientation.w,orientation.z);
    double roll, pitch;//定义存储r\p\y的容器
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, gps_yaw);//进行转换
    //gps_yaw = yaw / 180.0 * pi;
    head_init = false;
}


void aimibot::subscribeVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	int ret;
	float wz = (msg->angular.z) ;
	float vx = msg->linear.x ;
	
    // mavlink_interface->Mav_Set_Cmd_Long_Mode(0,AMB_MODE_FLAG_SPEED_MODE_ENABLED);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_Torque(0,1,2,3,4,5,2);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_Acceleration(0,100);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_GPIO_Output(0,0x0f0f);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_Analog_Output(0,0,1,2,3,4,5,6);
	//~ if ((vx != 0) || (wz != 0))
	{
		ret = interface.Mav_Set_Cmd_Long_Velocity(0,vx,wz);
		//~ printf("vx=%f,wz=%f\n",vx,wz);
	
	// usleep(5000);
	//ret = interface.Mav_Set_Cmd_Long_Frequency(1,2,50,50,50,0,50);
		if(ret==0)
		{
			//cout << "Success! timestamp = " << timestamp <<endl;
		}
		else
		{
			 cout << "Failed!" <<endl;
		}
		interface.Mav_Set_Heatbeat(0xff);
	}
}

void aimibot::subscribeGpioCommand(const mymsgs::msg::Control::SharedPtr msg)
{
	int ret;
	int flag = msg->gpio;
    // mavlink_interface->Mav_Set_Cmd_Long_Mode(0,AMB_MODE_FLAG_SPEED_MODE_ENABLED);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_Torque(0,1,2,3,4,5,2);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_Acceleration(0,100);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_GPIO_Output(0,0x0f0f);

	// usleep(5000);
	// mavlink_interface->Mav_Set_Cmd_Long_Analog_Output(0,0,1,2,3,4,5,6);
	//~ if ((vx != 0) || (wz != 0))
	{
		ret = interface.Mav_Set_Cmd_Long_GPIO_Output(0,flag);
		//~ printf("vx=%f,wz=%f\n",vx,wz);
	
	// usleep(5000);
	//ret = interface.Mav_Set_Cmd_Long_Frequency(1,2,50,50,50,0,50);
		if(ret==0)
		{
			//cout << "Success! timestamp = " << timestamp <<endl;
		}
		else
		{
			 cout << "Failed!" <<endl;
		}
		interface.Mav_Set_Heatbeat(0xff);
	}
}

