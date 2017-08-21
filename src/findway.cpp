float SPD_MAX = 0.5;
float SAFE_RANGE = 0.8;
float STOP_RANGE = 0.2;
float MAP_ORIGIN_TO_STRUCT_X = 1.0;
float MAP_ORIGIN_TO_STRUCT_Y = 1.0;
float SCAN_DELAY = 2.0;
float SCAN_SPEED = 1.0;
float BRAKE_COE = 1.0;
int WAYPOINT_GROUP = 1;
bool OUTPUT_GAZEBO = true;
bool OUTPUT_PX4 = true;

#include "ros/ros.h"

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CameraPose.h>
#include <px4_autonomy/Takeoff.h>
#include <px4_autonomy/Velocity.h>

#include <sstream>

ros::Publisher pub_spd_gazebo;
ros::Publisher pub_spd_px4;
ros::Publisher pub_cam;

void sendSpd_px4(float vx, float vy, float vz, float wz) {
	px4_autonomy::Velocity spd;
	spd.x = -vy;
	spd.y = vx;
	spd.z = vz;
	spd.yaw_rate = 0;
	spd.header.stamp = ros::Time::now();
	pub_spd_px4.publish(spd);
}

void sendSpd_gazebo(float vx, float vy, float wz) {
	geometry_msgs::Twist spd;
	spd.linear.x = vx;
	spd.linear.y = vy;
	spd.angular.z = wz;
	pub_spd_gazebo.publish(spd);
}


void sendSpd(float vx, float vy, float wz) {
	if (OUTPUT_GAZEBO) {
		sendSpd_gazebo(vx,vy,wz);
	}

	if (OUTPUT_PX4) {
		sendSpd_px4(vx,vy,0,wz);
	}
}

void sendSpd(float vx, float vy) {
	sendSpd(vx, vy, 0);
}

void stop() {
	sendSpd(0,0,0);
}

/////////////////////////////////////////////

typedef struct _pos_points_s
{
	float x;
	float y;
	bool need_scan;
	float waypoint_delay;
} _pos_points_t;

int progress = 0;	//the current target to go
int numofWaypoints = 0;
_pos_points_t wayPoints[50];

void addPosPoint(float x, float y, bool need_scan, float delaytime) {
	_pos_points_t pos;
	pos.x = x + MAP_ORIGIN_TO_STRUCT_X;
	pos.y = y + MAP_ORIGIN_TO_STRUCT_Y;
	pos.need_scan = need_scan;
	pos.waypoint_delay = delaytime;
	wayPoints[numofWaypoints] = pos;
	numofWaypoints ++;
}

void addPosPoint(float x, float y, bool need_scan) {
	addPosPoint(x, y, need_scan, 0);
}

void addPosPoint(float x, float y, float delaytime) {
	addPosPoint(x, y, false, delaytime);
}

void addPosPoint(float x, float y) {
	addPosPoint(x, y, false);
}

bool wayPointInit() {
	switch(WAYPOINT_GROUP) {
		case 1:
			addPosPoint(	0, 	1	,true);
			addPosPoint(	1, 	2	,true);
			addPosPoint(	1.7,3.2	);
			addPosPoint(	2.7,3.2	);
			addPosPoint(	3, 	2	,true);
			addPosPoint(	4, 	1	,true);
			addPosPoint( 	7, 	1	);
			addPosPoint( 	7,	2	,true);
			addPosPoint( 	8,	3	,true);
			addPosPoint( 	7.2,1	);
			addPosPoint( 	4.5,1	);
			addPosPoint( 	4, 	3	,true);
			addPosPoint( 	4, 	5	,true);
			addPosPoint( 	7.5,5	);
			addPosPoint( 	7.8,7	,true);
			addPosPoint( 	7.5,5	);
			break;
		case 2:
			addPosPoint(1,	1	,true);
			addPosPoint(4,	1	,true);
			addPosPoint(4.5,-0.38	,true);
			addPosPoint(5.8,-1.05	,true);
			addPosPoint(7,	-0.82	,true);
			addPosPoint(7.2,0.77	,true);
			addPosPoint(9.4,1.14);
			addPosPoint(8.7,2.8);
			addPosPoint(5.53,2.9);
			addPosPoint(5.78,5.84,true);
			addPosPoint(6.83,8.75);
			addPosPoint(4.7,8.9);
			addPosPoint(2.4,7);
			addPosPoint(0.375,8.635);
			break;
		case 3:
			addPosPoint(1.58,1.175	,true);
			addPosPoint(4.77,1.	,true);
			break;
			
		default:
			ROS_ERROR("no such waypoint group!!");
			return false;

	}
	return true;
}

///////////////////////////////////////////

float current_x;
float current_y;
bool gotlaser = false;

void laserCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	gotlaser = true;
	float x = msg -> pose.pose.position.x;
	float y = msg -> pose.pose.position.y;
	current_x = x;
	current_y = y;
}

void scan() {
	ROS_INFO("start scan");
	static bool last_scan_ori;
	mavros_msgs::CameraPose cam_msg;
	cam_msg.yaw = last_scan_ori ? 1 : -1;
	cam_msg.yaw *= SCAN_SPEED;
	last_scan_ori = !last_scan_ori;
	pub_cam.publish(cam_msg);
	ros::Duration(SCAN_DELAY).sleep();
	cam_msg.yaw = 0;
	pub_cam.publish(cam_msg);
	ROS_INFO("scan done");
}

int status;
ros::Publisher pub_takeoff;

void statusCB(const std_msgs::UInt8::ConstPtr& msg) {
	status = msg -> data;
}

void takeoff() {
	while (status != 1) {
		ros::Duration(1).sleep();
		ROS_INFO("waitting for OFFBOARD");
		ros::spinOnce();
	}

	px4_autonomy::Takeoff cmd_tf;
	cmd_tf.take_off = 1;
	cmd_tf.header.stamp = ros::Time::now();
	pub_takeoff.publish(cmd_tf);

	while (status != 5) {
		ros::Duration(1).sleep();
		ROS_INFO("taking off...");
		ros::spinOnce();
	}
	
}

////////////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_nav");
	ros::NodeHandle n;

	n.getParam("/uav_nav/spd_max", SPD_MAX);
	n.getParam("/uav_nav/origin_x", MAP_ORIGIN_TO_STRUCT_X);
	n.getParam("/uav_nav/origin_y", MAP_ORIGIN_TO_STRUCT_Y);
	n.getParam("/uav_nav/scan_spd", SCAN_SPEED);
	n.getParam("/uav_nav/scan_spd", SCAN_DELAY);
	n.getParam("/uav_nav/stop_range", STOP_RANGE);
	n.getParam("/uav_nav/safe_range", SAFE_RANGE);
	n.getParam("/uav_nav/brake_coe", BRAKE_COE);
	n.getParam("/uav_nav/waypoint_group", WAYPOINT_GROUP);
	n.getParam("/uav_nav/output_gazebo", OUTPUT_GAZEBO);
	n.getParam("/uav_nav/output_px4", OUTPUT_PX4);

	if (!wayPointInit()) return 0;

	pub_spd_gazebo = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_cam = n.advertise<mavros_msgs::CameraPose>("/Cam_Pose", 1);
	pub_takeoff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1);
	pub_spd_px4 = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel",1);

	ros::Subscriber sub_laser = n.subscribe("/amcl_pose", 1, laserCB);
	ros::Subscriber sub_status = n.subscribe("/px4/status", 1, statusCB);
	ros::Rate loop_rate(10);

	while (!gotlaser) {
		ros::Duration(1).sleep();
		ROS_INFO("waitting for amcl_pose");
		ros::spinOnce();
	}

	ROS_INFO("get laser, start guide");

	takeoff();

	ROS_INFO("take off complete");

	while (ros::ok()) {
		// ROS_INFO("%.2f\t%.2f", wayPoints[0].x, wayPoints[0].y);
		ros::spinOnce();

		float disx = wayPoints[progress].x - current_x;
		float disy = wayPoints[progress].y - current_y;
		float dis = sqrt(disx * disx + disy * disy);

		if (dis < STOP_RANGE) {
			stop();
			if (wayPoints[progress].need_scan == false) {
				float t = wayPoints[progress].waypoint_delay;
				if (t != 0) {
					ros::Duration(t).sleep();
				}
			} else {
				scan();
			}
			progress ++;
			continue;
		}

		float brake_coe = BRAKE_COE;
		if (dis < SAFE_RANGE) {
			brake_coe *= dis / SAFE_RANGE;
		}

		float spdx = SPD_MAX / dis * disx * brake_coe;
		float spdy = SPD_MAX / dis * disy * brake_coe;
		sendSpd(spdx, spdy);

		loop_rate.sleep();
		
		if (progress >= numofWaypoints) {
			stop();
			ROS_INFO("nav finished");
			ros::spin();
		}
	}

	return 0;
}
