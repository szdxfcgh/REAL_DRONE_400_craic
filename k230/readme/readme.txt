K230 serial protocol

K230 prints one line per event:

ROS_MSG:qr,<class_a>,<class_b>,<left|right>
ROS_MSG:target,<label>,<confidence>,<cx>,<cy>
ROS_MSG:ring,<x>,<y>,<z>,<confidence>
ROS_MSG:special,<confidence>,<cx>,<cy>
ROS_MSG:landing,<side>,<dx>,<dy>,<confidence>

Examples:

ROS_MSG:qr,man,apple,left
ROS_MSG:target,man,0.87,320,240
ROS_MSG:ring,7.50,0.00,1.60,0.91
ROS_MSG:special,0.76,318,242
ROS_MSG:landing,right,-0.12,0.08,0.88

ROS side publishes:

/craic/qr_result        std_msgs/String: class_a,class_b,left|right
/craic/target_detected  std_msgs/String: label,confidence,cx,cy
/craic/ring_pose        geometry_msgs/PoseStamped
/craic/special_target   std_msgs/String: confidence,cx,cy
/craic/landing_marker   std_msgs/String: side,dx,dy,confidence
/craic/k230/raw         std_msgs/String: original line

Preferred ROS node:

roslaunch craic_mission k230_serial_node.launch serial_port:=/dev/ttyACM0 baud_rate:=115200

Topic checks:

rostopic echo /craic/k230/raw
rostopic echo /craic/qr_result
rostopic echo /craic/target_detected
rostopic echo /craic/ring_pose
rostopic echo /craic/special_target
rostopic echo /craic/landing_marker

The standalone k230_serial_node.py is kept as a fallback template. The active
copy has been merged into craic_mission/scripts, and k230_serial_node.launch now
uses pkg="craic_mission".
