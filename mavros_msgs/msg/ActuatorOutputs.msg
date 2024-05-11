# ActuatorOutputs.msg
#
# ROS representation of MAVLink ACTUATOR_OUTPUT_STATUS 
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_STATUS

std_msgs/Header header
uint32 active
float32[32] actuator