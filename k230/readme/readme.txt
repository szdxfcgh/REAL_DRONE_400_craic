K230 serial protocol

K230 prints one line per event:

ROS_MSG:qr,man,apple,left
ROS_MSG:target,man,0.87,320,240
ROS_MSG:target,apple,0.82,300,250

ROS side publishes:

/craic/qr_result        std_msgs/String: man,apple,left
/craic/target_detected  std_msgs/String: man,0.870,320.0,240.0
/craic/k230/raw         std_msgs/String: original serial line

Preferred ROS node:

roslaunch craic_mission k230_bridge.launch mode:=serial serial_port:=/dev/ttyACM0 serial_baud:=115200

The standalone k230_serial_node.py is kept as a fallback template. The active
copy has been merged into craic_mission/scripts, and k230_serial_node.launch now
uses pkg="craic_mission".
