#define SPD_MAX 1
#define SAFE_RANGE 0.8
#define STOP_RANGE 0.2
#define MAP_TO_ORIGIN_X 1.0
#define MAP_TO_ORIGIN_Y 1.0
#define WAY_POINT_DELAY 0.1
#define SCAN_POINT_DELAY 2
#define SCAN_SPEED 1

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <mavros_msgs/CameraPose.h>

#include <sstream>

ros::Publisher pub_spd;
ros::Publisher pub_cam;

void sendSpd(float vx, float vy, float wz) {
	geometry_msgs::Twist spd;
	spd.linear.x = vx;
	spd.linear.y = vy;
	spd.angular.z = wz;
	pub_spd.publish(spd);
}

void sendSpd(float vx, float vy) {
	sendSpd(vx, vy, 0);
}

void stop() {
	sendSpd(0,0,0);
}

typedef struct _pos_points_s
{
	float x;
	float y;
	bool need_scan;
} _pos_points_t;

int progress = 0;	//the current target to go
int numofWaypoints = 0;
_pos_points_t wayPoints[50];

void addPosPoint(float x, float y, bool need_scan) {
	_pos_points_t pos;
	pos.x = x + MAP_TO_ORIGIN_X;
	pos.y = y + MAP_TO_ORIGIN_Y;
	pos.need_scan = need_scan;
	wayPoints[numofWaypoints] = pos;
	numofWaypoints ++;
}

void addPosPoint(float x, float y) {
	addPosPoint(x, y, false);
}

void wayPointInit() {
	addPosPoint(	-1, 1	);
	addPosPoint(	1, 	2	,true);
	addPosPoint(	2, 	3.2	);
	addPosPoint(	4, 	2	,true);
	addPosPoint( 	6, 	0.8	);
	addPosPoint( 	7, 	2	,true);
	addPosPoint( 	6, 	0.8	);
	addPosPoint( 	4, 	4	);
	addPosPoint( 	5, 	6	,true);
	addPosPoint( 	6, 	4.8	);
	addPosPoint( 	7, 	6	,true);
	addPosPoint( 	6, 	4.8	);
	addPosPoint( 	5, 	6	);
}


float current_x;
float current_y;
void laserCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	float x = msg -> pose.pose.position.x;
	float y = msg -> pose.pose.position.y;
	current_x = x;
	current_y = y;
}

void scan() {
	static bool last_scan_ori;
	mavros_msgs::CameraPose cam_msg;
	cam_msg.yaw = last_scan_ori ? 1 : -1;
	cam_msg.yaw *= SCAN_SPEED;
	last_scan_ori = !last_scan_ori;
	pub_cam.publish(cam_msg);
	ros::Duration(SCAN_POINT_DELAY).sleep();
	cam_msg.yaw = 0;
	pub_cam.publish(cam_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_nav");
	wayPointInit();
	ros::NodeHandle n;
	pub_spd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_cam = n.advertise<mavros_msgs::CameraPose>("/Cam_Pose", 1);
	ros::Subscriber sub_laser = n.subscribe("/amcl_pose", 1, laserCB);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		// ROS_INFO("%.2f\t%.2f", wayPoints[0].x, wayPoints[0].y);
		ros::spinOnce();

		float disx = wayPoints[progress].x - current_x;
		float disy = wayPoints[progress].y - current_y;
		float dis = sqrt(disx * disx + disy * disy);

		if (dis < STOP_RANGE) {
			stop();
			if (wayPoints[progress].need_scan == false) {
				ros::Duration(WAY_POINT_DELAY).sleep();
			} else {
				scan();
			}
			progress ++;
			continue;
		}

		float brake_coe = 1.0;
		if (dis < SAFE_RANGE) {
			brake_coe *= dis / SAFE_RANGE * 0.8;
		}

		float spdx = SPD_MAX / dis * disx * brake_coe;
		float spdy = SPD_MAX / dis * disy * brake_coe;
		sendSpd(spdx, spdy);

		loop_rate.sleep();
		
		if (progress >= numofWaypoints) {
			stop();
			ros::spin();
		}
	}

	return 0;
}