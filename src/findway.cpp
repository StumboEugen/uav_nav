float SPD_MAX = 0.5;
float SAFE_RANGE = 0.8;
float STOP_RANGE = 0.2;
float MAP_ORIGIN_TO_STRUCT_X = 1.0;
float MAP_ORIGIN_TO_STRUCT_Y = 1.0;
float SCAN_DELAY = 2.0;
float SCAN_SPEED = 1.0;
float BRAKE_COE = 1.0;
float HEIGHT = 1.0;
float SPD_CLIMB = 0.1;
float SPD_TURN = 0.5;
float ANGLE_TOLERANCE = 0.3;
float Z_TOLERANCE = 0.05;
int WAYPOINT_GROUP = 1;

const double PI = 3.1415926;

#include "ros/ros.h"

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CameraPose.h>
#include <px4_autonomy/Takeoff.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Position.h>

#include <sstream>

ros::Publisher pub_pose_px4;

float last_set_x, last_set_y, last_set_z, last_set_yaw;
void sendPose_px4(float px, float py, float pz, float yaw) {

	last_set_x = px;
	last_set_y = py;
	last_set_z = pz;
	last_set_yaw = yaw;

	px4_autonomy::Position pos;
	pos.x =-py;
	pos.y = px;
	pos.z = pz;
	yaw += PI/2;
	if (yaw > PI) {
		yaw -= PI * 2;
	}
	pos.yaw = yaw;
	pos.header.stamp = ros::Time::now();
	pub_pose_px4.publish(pos);
}

void hover(float time) {
	for (int count = 0; count <= (int)(time * 20); count++) {
		sendPose_px4(last_set_x, last_set_y, last_set_z, last_set_yaw);
		ros::Duration(0.05).sleep();
		ros::spinOnce();
	}
}

bool gotErWeiMa = false;
int numErWeiMa = -1;
void erweimaCB(const std_msgs::Int32 msg) {
	if (msg.data < 0) {
		gotErWeiMa = false;
	} else {
		numErWeiMa = msg.data;
		gotErWeiMa = true;
	}
}

bool gotlaser = false;
float laser_x, laser_y, laser_z, laser_yaw;
void laserPoseCB(const geometry_msgs::PoseStamped &msg) {
	gotlaser = true;
	laser_x = msg.pose.position.x;
	laser_y = msg.pose.position.y;
	laser_z = msg.pose.position.z;
	laser_yaw = (msg.pose.orientation.z) * 2;
}

float pose_x, pose_y, pose_z, pose_yaw;
void poseCB(const px4_autonomy::Position &msg) {
	pose_x = msg.y;
	pose_y =-msg.x;
	pose_z = msg.z;
	float yaw = msg.yaw;
	yaw -= PI/2;
	if (yaw < -PI) {
		yaw += PI * 2;
	}
	pose_yaw = yaw;
}

int status;
void statusCB(const std_msgs::UInt8::ConstPtr& msg) {
	status = msg -> data;
}

typedef struct _pos_points_s
{
	float x, y, z, yaw;
	bool change_z;
	bool change_yaw;
	float delay;
	bool need_delay;
	bool need_scan;
} _pos_points_t;

int progress = 0;
int numofWayPoints = 0;
_pos_points_t wayPoints[50];

void setZ(float z) {
	wayPoints[numofWayPoints - 1].change_z = true;
	wayPoints[numofWayPoints - 1].z = z;
}

void setYaw(float yaw) {
	wayPoints[numofWayPoints - 1].change_yaw = true;
	wayPoints[numofWayPoints - 1].yaw = yaw;
}

void setScan() {
	wayPoints[numofWayPoints - 1].need_scan = true;
}

void setDelay(float delay) {
	wayPoints[numofWayPoints - 1].need_delay = true;
	wayPoints[numofWayPoints - 1].delay = delay;
}

void addPosPoint(float x, float y) {
	_pos_points_t pp;
	pp.x = x + MAP_ORIGIN_TO_STRUCT_X;
	pp.y = y + MAP_ORIGIN_TO_STRUCT_Y;
	if (numofWayPoints != 0) {
		pp.z = wayPoints[numofWayPoints - 1].z;
		pp.yaw = wayPoints[numofWayPoints - 1].yaw;
	} else {
		pp.z = HEIGHT;
		pp.yaw = 0;
	}
	pp.change_z = false;
	pp.change_yaw = false;
	pp.need_scan = false;
	wayPoints[numofWayPoints] = pp;
	numofWayPoints ++;

	setDelay(1);	//default delay 1 sec
}

bool wayPointInit();

bool arrived_z(float from, float to) {
	if (fabs(from - to) < Z_TOLERANCE) {
		return true;
	} else {
		return false;
	}
}

bool arrived_yaw(float from, float to) {
	if ( fabs(from - to) < ANGLE_TOLERANCE) {
		return true;
	} else {
		return false;
	}
}

void turnYaw(float yaw_from, float yaw_to) {
	float turn_ori;
	float yaw_diff = yaw_to - yaw_from;
	if (yaw_diff > PI || (yaw_diff < 0 && yaw_diff > -PI)) {
		turn_ori = -1;
	} else {
		turn_ori = 1;
	}

	float yaw2set = yaw_from;
	ros::spinOnce();

	while (!arrived_yaw( pose_yaw, yaw_to)) {
		
		if ( fabs(yaw2set - yaw_to) < SPD_TURN * 0.1) {
			yaw2set = yaw_to;
		} else {
			yaw2set += turn_ori * SPD_TURN * 0.05;
			if (yaw2set < -PI) {
				yaw2set += 2 * PI;
			}
			if (yaw2set > PI) {
				yaw2set -= 2 * PI;
			}
		}
		sendPose_px4(last_set_x, last_set_y, last_set_z, yaw2set);
		ros::Duration(0.05).sleep();
		ros::spinOnce();
	}
	ROS_INFO("turn finished");
}

void climbZ(float z_from, float z_to) {
	float climb_ori;
	if (z_from > z_to) {
		climb_ori = -1;
	} else {
		climb_ori = 1;
	}

	ros::spinOnce();

	float z2set = z_from;

	while (!arrived_z( laser_z, z_to)) {
		if (fabs( z2set - z_to) < SPD_CLIMB * 0.1) {
			z2set = z_to;
		} else {
			z2set += climb_ori * SPD_CLIMB * 0.05;
		}
		sendPose_px4(last_set_x, last_set_y, z2set, last_set_yaw);
		ros::Duration(0.05).sleep();
		ros::spinOnce();
	}
	ROS_INFO("climb finished");
}

ros::Publisher pub_takeoff;

void takeOff() {
	while (status != 1) {
		ros::Duration(1).sleep();
		ROS_INFO("waitting for OFFBOARD");
		ros::spinOnce();
	}

	last_set_x = pose_x;
	last_set_y = pose_y;
	last_set_yaw = 0;

	px4_autonomy::Takeoff cmd_tf;
	cmd_tf.take_off = 1;
	cmd_tf.header.stamp = ros::Time::now();
	pub_takeoff.publish(cmd_tf);

	while (status != 5) {
		ros::Duration(1).sleep();
		ROS_INFO("taking off...");
		ros::spinOnce();
	}
	ROS_INFO("Start climb");
	climbZ(pose_z, HEIGHT);
}

void land() {
	px4_autonomy::Takeoff cmd_tf;
	cmd_tf.take_off = 2;
	cmd_tf.header.stamp = ros::Time::now();
	pub_takeoff.publish(cmd_tf);

	while (status != 1) {
		ros::Duration(1).sleep();
		ROS_INFO("landing...");
		ros::spinOnce();
	}

	ROS_INFO("landed");
}

void scan() {
	ROS_INFO("scanning...");
	bool gotit = false;
	for (int time_count = 0; time_count <= SCAN_DELAY * 20; time_count ++) {
		ros::spinOnce();
		if (gotErWeiMa) {
			ROS_INFO("got ERWEIMA: %d", numErWeiMa);
			gotit = true;
			break;
		}
		ros::Duration(0.05).sleep();
	}

	if (!gotit) {
		ROS_ERROR("Scan TIMEOUT!!");
	}
}

bool needSetStartXY = false;
float start_x, start_y;
float spdx, spdy;
int counts_XY;
float x2set, y2set;
void setStartXY(_pos_points_t wp) {
	if (needSetStartXY) {
		start_x = pose_x;
		start_y = pose_y;
		
		x2set = start_x;
		y2set = start_y;

		float disx = wp.x - start_x;
		float disy = wp.y - start_y;
		float dis = sqrt(disx * disx + disy * disy);
		spdx = SPD_MAX / dis * disx;
		spdy = SPD_MAX / dis * disy;

		needSetStartXY = false;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "uav_nav");
	ros::NodeHandle n;

	n.getParam("/uav_nav/spd_max", SPD_MAX);
	n.getParam("/uav_nav/origin_x", MAP_ORIGIN_TO_STRUCT_X);
	n.getParam("/uav_nav/origin_y", MAP_ORIGIN_TO_STRUCT_Y);
	n.getParam("/uav_nav/scan_spd", SCAN_SPEED);
	n.getParam("/uav_nav/scan_delay", SCAN_DELAY);
	n.getParam("/uav_nav/stop_range", STOP_RANGE);
	n.getParam("/uav_nav/safe_range", SAFE_RANGE);
	n.getParam("/uav_nav/brake_coe", BRAKE_COE);
	n.getParam("/uav_nav/waypoint_group", WAYPOINT_GROUP);
	n.getParam("/uav_nav/height", HEIGHT);
	n.getParam("/uav_nav/spd_climb", SPD_CLIMB);
	n.getParam("/uav_nav/spd_turn", SPD_TURN);
	n.getParam("/uav_nav/angle_tolerance", ANGLE_TOLERANCE);
	n.getParam("/uav_nav/z_tolerance", Z_TOLERANCE);

	if (!wayPointInit()) return 0;

	pub_pose_px4 = n.advertise<px4_autonomy::Position>("/px4/cmd_pose",1);
	pub_takeoff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1);

	ros::Subscriber sub_laser = n.subscribe("/mavros/vision_pose/pose", 1, laserPoseCB);
	ros::Subscriber sub_pose = n.subscribe("/px4/pose", 1, poseCB);	
	ros::Subscriber sub_status = n.subscribe("/px4/status", 1, statusCB);
	ros::Subscriber sub_erweima = n.subscribe("/rec_num", 1, erweimaCB);

	ros::Rate loop_rate(20);

	while (!gotlaser) {
		ros::spinOnce();
		ROS_INFO("waitting for amcl_pose");
		ros::Duration(1).sleep();
	}
	ROS_INFO("get laser, start guide");

	takeOff();

	ROS_INFO("take off complete");
	hover(1);

	needSetStartXY = true;

	while (ros::ok()) {
		
		_pos_points_t wp = wayPoints[progress];
		setStartXY(wp);

		float dis_now_x = wp.x - pose_x;
		float dis_now_y = wp.y - pose_y;
		float dis_now = sqrt(dis_now_x * dis_now_x + dis_now_y + dis_now_y);

		if (dis_now < STOP_RANGE) {
			ROS_INFO("Reach Point NO.%i:  %.2f\t%.2f", 
				progress, wp.x, wp.y);

			if (wp.need_delay) {
				hover(wp.delay);
			}

			if (wp.change_yaw) {
				float from = pose_yaw;
				float to = wp.yaw;
				ROS_INFO("need turn From %.2f\tto %.2f", from, to);
				turnYaw(from, to);
				hover(0.5);
			}

			if (wp.change_z) {
				float from = pose_z;
				float to = wp.z;
				ROS_INFO("need climb\tFrom %.2f\tto %.2f", from, to);
				climbZ(from, to);
			}

			if (wp.need_scan) {
				ROS_INFO("need scan");
				scan();
			}

			progress ++;

			if ( progress >= numofWayPoints) {
				ROS_INFO("nav finished");
				hover(3);
				ROS_INFO("start land");
				ros::spinOnce();
				land();
				ros::spin();
			}

			needSetStartXY = true;

			ROS_INFO("NEXT: \t%.2f\t%.2f\txplus:%.2f\typlus:%.2f\n"
			, wayPoints[progress].x, wayPoints[progress].y,
			wayPoints[progress].x - wp.x,
			wayPoints[progress].y - wp.y);

			continue;
		}
		
		float dis_set_x = wp.x - x2set;
		float dis_set_y = wp.y - y2set;
		float dis_set = sqrt(dis_set_x * dis_set_x + dis_set_y + dis_set_y);
		if ( dis_set < SPD_MAX * 0.1) {
			x2set = wp.x;
			y2set = wp.y;
		} else {
			x2set += spdx * 0.05;
			y2set += spdy * 0.05;
		}
		
		sendPose_px4(x2set, y2set, last_set_z, last_set_yaw);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

bool wayPointInit() {
	switch(WAYPOINT_GROUP) {
		case 5:
			addPosPoint( 1.00, 1.00);
			addPosPoint( 4.00, 1.00); setZ(1.3);
			addPosPoint( 4.40, 0.00); setYaw(3.1);
			addPosPoint( 5.25,-1.00); setZ(0.8);
			addPosPoint( 7.36,-1.20);
			addPosPoint( 7.34, 0.67);
			addPosPoint( 7.25,-1.20);
			addPosPoint( 4.56,-0.83); setYaw(1.57);
			addPosPoint( 4.28, 2.00);
			addPosPoint( 5.07, 3.00);
			addPosPoint( 7.42, 2.92);
			addPosPoint( 7.93, 4.41);
			addPosPoint( 7.48, 2.90);
			addPosPoint( 5.62, 2.83);
			addPosPoint( 5.73, 5.00);
			addPosPoint( 5.82, 5.80);
			addPosPoint( 6.10, 7.00); setYaw(-1.57);
			addPosPoint( 7.40, 7.79);
			addPosPoint( 5.80, 8.80);
			addPosPoint( 3.93, 9.10);
			addPosPoint( 3.43, 7.55); setYaw(0);
			addPosPoint( 2.27, 6.87); setYaw(3.1);
			addPosPoint( 1.21, 7.75);
			addPosPoint( 1.33, 9.22);
			addPosPoint(-0.15, 8.80);
			break;

		case 6:
			addPosPoint( 0.00,-5.70); setZ(1.5); setScan();
			addPosPoint( 0.00,-5.70); setYaw(-1.57); setScan();
			addPosPoint( 0.00,-5.70); setYaw(-3.1); setZ(1); setScan();
			addPosPoint( 0.00, 0.00); setYaw(0);
			break;

		case 100:
			addPosPoint( 1.0, 1.0); setYaw(-1.57);
			addPosPoint( 1.0, 1.0); setYaw(0);
			addPosPoint( 1.0, 3.0); setYaw(1.57);
			addPosPoint( 3.0, 3.0); setYaw(3.14);
			addPosPoint( 3.0, 1.0); setYaw(-1.57);
			addPosPoint( 7.0, 1.0); setYaw(1.57);
			addPosPoint( 7.0, 3.0); setYaw(3.14);
			addPosPoint( 7.0, 1.0);
			addPosPoint( 5.0, 1.0); setYaw(0);
			addPosPoint( 5.0, 3.0); setYaw(1.57);
			addPosPoint( 4.0, 3.0);
			addPosPoint( 4.0, 5.0); setYaw(3.14);
			addPosPoint( 2.5, 5.0); setYaw(-1.57);
			addPosPoint( 7.4, 5.0); setYaw(1.57);
			addPosPoint( 7.4, 7.0); setYaw(3.14);
			addPosPoint( 7.4, 5.0);
			addPosPoint( 5.25,5.0);
			addPosPoint( 5.25,7.0); setYaw(0);
			addPosPoint( 5.25,7.0); setYaw(1.57);
			addPosPoint( 5.25,9.0); setYaw(3.14);
			addPosPoint( 5.5, 9.0); setYaw(-1.57);
			addPosPoint( 7.0, 9.0); setYaw(1.57);
			addPosPoint( 7.0, 11.0);
			addPosPoint( 3.0, 11.0);setYaw(3.14);
			addPosPoint( 3.5, 9.0); setYaw(0);
			addPosPoint( 3.5, 9.0); setYaw(-1.57);
			addPosPoint( 0.6, 9.0); setYaw(1.57);
			addPosPoint( 0.6, 11.0);setYaw(0);
			addPosPoint( 0.6, 9.0);
			addPosPoint( 1.75,9.0); setYaw(-1.57);
			addPosPoint( 1.75,7.0); setYaw(0);
			addPosPoint( 3.0, 7.0); setYaw(1.57);
			addPosPoint( 0.6, 7.0); setYaw(-1.57);
			addPosPoint( 0.6, 5.0); setYaw(0);
			addPosPoint( 0.0, 5.5);
			break;

		case 101:
			addPosPoint( 1.0, 1.5);
			addPosPoint( 1.0, 3.3); setYaw(1.57);
			addPosPoint( 4.0, 3.0); setYaw(0);		setZ(1);
			addPosPoint( 5.2, 2.5); setYaw(-1.57);	setZ(1.5);
			addPosPoint( 4.0, 1.0); setZ(1.3);
			addPosPoint( 7.3, 1.0); setYaw(3.1);
			addPosPoint( 7.3, 3.4);
			addPosPoint( 7.3, 1.0);
			addPosPoint( 4.0, 1.0); setYaw(1.57);
			addPosPoint( 4.0, 5.3); setYaw(-1.57);	setZ(1);
			addPosPoint( 6.2, 5.3); setYaw(3.1);
			addPosPoint( 5.4, 6.5); setZ(1.2);
			addPosPoint( 5.4, 6.9); setYaw(0); 		setZ(1);
			addPosPoint( 5.6, 7.4); setYaw(-1.57);
			addPosPoint( 5.4, 5.4); setYaw(3.1); 	setZ(1.3);
			addPosPoint( 3.0, 5.7); setYaw(1.57);
			addPosPoint( 3.0, 7.5); setYaw(-1.57);
			addPosPoint( 0.9, 7.5);
			addPosPoint( 0.9, 5.5); setYaw(1.57);
			addPosPoint( 0.9, 7.3);
			addPosPoint( 2.0, 8.0);
			addPosPoint( 2.0, 9.6);
			addPosPoint( 0.5, 9.6); setYaw(0);		setZ(1.2);
			addPosPoint( 0.5, 11.3);setYaw(-1.57);
			addPosPoint( 0.7, 9.5); setYaw(0);
			addPosPoint( 3.0, 9.5); setZ(1);
			addPosPoint( 3.7, 10.0);setYaw(1.57);
			addPosPoint( 3.0, 11.5);setYaw(3.1);	setZ(1.3);
			addPosPoint( 2.5, 10.0);setYaw(-1.57);
			addPosPoint( 6.0, 10.0);
			addPosPoint( 6.8, 9.5); setYaw(3.1);
			addPosPoint( 5.7, 10.8);setZ(1.45);
			addPosPoint( 6.0, 11.5);setYaw(1.57);
			addPosPoint( 6.5, 10.5);

			break;

		default:
			ROS_ERROR("no such waypoint group! %d", WAYPOINT_GROUP);
			return false;
	}
	return true;
}