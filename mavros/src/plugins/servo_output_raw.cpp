/**
 * @brief ServoOutputRaw plugin
 * @file servo_output_raw.cpp
 * @author Mingsong Li
 *
 * @addtogroup plugin
 * @{
 */
/*
 * 
 *
 * 
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.hpp>
#include <mavros_msgs/ServoOutputRaw.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief ActuatorOutput plugin
 *
 * Subscribe actuator output from FCU controller.
 */
class ServoOutputRawPlugin : public plugin::PluginBase {
public:
	ServoOutputRawPlugin() : PluginBase(),
		nh("~servo_output")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		actuator_outputs_pub = nh.advertise<mavros_msgs::ServoOutputRaw>("raw", 10);

	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&ServoOutputRawPlugin::handle_actuator_outputs),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher actuator_outputs_pub;

	/* -*- rx handlers  -*- */

	void handle_actuator_outputs(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &actuator_outputs)
	{
		auto actuator_outputs_msg = boost::make_shared<mavros_msgs::ServoOutputRaw>();
		actuator_outputs_msg -> header.stamp = m_uas->synchronise_stamp(actuator_outputs.time_usec);

		actuator_outputs_msg->port = actuator_outputs.port;
		actuator_outputs_msg->servo1_raw = actuator_outputs.servo1_raw;
		actuator_outputs_msg->servo2_raw = actuator_outputs.servo2_raw;
		actuator_outputs_msg->servo3_raw = actuator_outputs.servo3_raw;
		actuator_outputs_msg->servo4_raw = actuator_outputs.servo4_raw;
		actuator_outputs_msg->servo5_raw = actuator_outputs.servo5_raw;
		actuator_outputs_msg->servo6_raw = actuator_outputs.servo6_raw;
		actuator_outputs_msg->servo7_raw = actuator_outputs.servo7_raw;
		actuator_outputs_msg->servo8_raw = actuator_outputs.servo8_raw;
		actuator_outputs_msg->servo9_raw = actuator_outputs.servo9_raw;
		actuator_outputs_msg->servo10_raw = actuator_outputs.servo10_raw;
		actuator_outputs_msg->servo11_raw = actuator_outputs.servo11_raw;
		actuator_outputs_msg->servo12_raw = actuator_outputs.servo12_raw;
		actuator_outputs_msg->servo13_raw = actuator_outputs.servo13_raw;
		actuator_outputs_msg->servo14_raw = actuator_outputs.servo14_raw;
		actuator_outputs_msg->servo15_raw = actuator_outputs.servo15_raw;
		actuator_outputs_msg->servo16_raw = actuator_outputs.servo16_raw;
		actuator_outputs_pub.publish(actuator_outputs_msg);
	}

	/* -*- callbacks -*- */

	
};
}	// namespace std_plugins
}	// namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ServoOutputRawPlugin, mavros::plugin::PluginBase)
