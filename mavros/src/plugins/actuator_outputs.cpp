/**
 * @brief ActuatorOutputs plugin
 * @file actuator_outputs.cpp
 * @author Mingsong Li
 *
 * @addtogroup plugin
 * @{
 */
/*
 * 
 *
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ActuatorOutputs.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief ActuatorOutputs plugin
 *
 * Subscribe actuator outputs from FCU controller.
 */
class ActuatorOutputsPlugin : public plugin::PluginBase {
public:
	ActuatorOutputsPlugin() : PluginBase(),
		nh("~actuator_outputs")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		actuator_outputs_pub = nh.advertise<mavros_msgs::ActuatorOutputs>("outputs", 10);
	
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&ActuatorOutputsPlugin::handle_actuator_outputs),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher actuator_outputs_pub;

	/* -*- rx handlers -*- */

	void handle_actuator_outputs(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ACTUATOR_OUTPUT_STATUS &actuator_outputs)
	{
		auto actuator_outputs_msg = boost::make_shared<mavros_msgs::ActuatorOutputs>();
		actuator_outputs_msg->header.stamp = m_uas->synchronise_stamp(actuator_outputs.time_usec);

		actuator_outputs_msg->active = actuator_outputs.active;
		const auto &arr = actuator_outputs.actuator;
		std::copy(arr.cbegin(), arr.cend(), actuator_outputs_msg->actuator.begin());

		actuator_outputs_pub.publish(actuator_outputs_msg);
	}

	/* -*- callbacks -*- */

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ActuatorOutputsPlugin, mavros::plugin::PluginBase)
