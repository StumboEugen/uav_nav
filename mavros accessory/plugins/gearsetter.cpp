#include <mavros/mavros_plugin.h>

#include <mavros_msgs/GearSet.h>

namespace mavros {
namespace extra_plugins{

	class GearSetterPlugin : public plugin::PluginBase {
	public:
		GearSetterPlugin() : PluginBase(),
			gear_nh("~gear")
			{}

		void initialize (UAS &uas_) {
			PluginBase::initialize(uas_);
			gearservice = gear_nh.advertiseService("/GearSet", &GearSetterPlugin::set_gear, this);
		}

		Subscriptions get_subscriptions() {
			return {};
		}

	private:
		ros::NodeHandle gear_nh;
		ros::ServiceServer gearservice;

		bool set_gear(	mavros_msgs::GearSet::Request &req,
						mavros_msgs::GearSet::Response &res) {
			uint8_t gear_status;
			mavlink::common::msg::AUX_CONTROL auxmsg = {};
			auxmsg.auxnum = 1;
			if (req.gearup == true) {
				auxmsg.content = 1;
				UAS_FCU(m_uas)->send_message_ignore_drop(auxmsg);
				ROS_INFO("GEAR UP");
			}

			if (req.gearup == false) {
				auxmsg.content = -1;
				UAS_FCU(m_uas)->send_message_ignore_drop(auxmsg);
				ROS_INFO("GEAR DOWN");
			}
			return true;
		}
	};
}}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GearSetterPlugin, mavros::plugin::PluginBase)
