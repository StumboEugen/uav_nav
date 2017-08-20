#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CameraShoot.h>
#include <mavros_msgs/CameraPose.h>

namespace mavros {
namespace extra_plugins{

	class CameraCtrlPlugin : public plugin::PluginBase {
	public:
		CameraCtrlPlugin() : PluginBase(),
			camera_ctrl_nh("~camera_ctrl")
		{}

		void initialize(UAS &uas_) {
			PluginBase::initialize(uas_);
			camShoot_srv = camera_ctrl_nh.advertiseService("/CameraShoot", &CameraCtrlPlugin::Cam_shoot, this);
			camPose_sub = camera_ctrl_nh.subscribe("/Cam_Pose", 1, &CameraCtrlPlugin::Pose_cb, this);
		}

		Subscriptions get_subscriptions() {
			return {};
		}

	private:
		ros::NodeHandle camera_ctrl_nh;
		ros::ServiceServer camShoot_srv;
		ros::Subscriber camPose_sub;
		float yaw_last = 0;
		float pitch_last = 0;

		bool Cam_shoot (mavros_msgs::CameraShoot::Request &req ,
						mavros_msgs::CameraShoot::Response &res) {
			ROS_INFO("Take a picture");
			return true;
		}

		void Pose_cb (const mavros_msgs::CameraPose::ConstPtr &pose) {
			if (yaw_last != pose -> yaw) {
				mavlink::common::msg::AUX_CONTROL auxmsg = {};
				auxmsg.content = pose -> yaw;
				auxmsg.auxnum = 2;
				UAS_FCU(m_uas)->send_message_ignore_drop(auxmsg);
				yaw_last = pose -> yaw;
			}

			if (pitch_last != pose -> pitch) {
				mavlink::common::msg::AUX_CONTROL auxmsg = {};
				auxmsg.content = pose -> pitch;
				auxmsg.auxnum = 3;
				UAS_FCU(m_uas)->send_message_ignore_drop(auxmsg);
				pitch_last = pose -> pitch;
			}
		}
	};

}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CameraCtrlPlugin, mavros::plugin::PluginBase)