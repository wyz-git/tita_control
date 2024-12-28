export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=200
sudo cp *.service /lib/systemd/system
sudo apt-get install ros-humble-tf-transformations
sudo apt-get install python3-pip
sudo apt install ffmpeg
pip install pyzbar
pip install paho-mqtt==1.5.1
chmod 777 /home/robot/ros2_common_ws/install/ros2_aruco/lib/ros2_aruco/aruco_node
systemctl daemon-reload
systemctl restart read-pose.service
systemctl enable read-pose.service
systemctl restart bt-control.service
systemctl enable bt-control.service
systemctl restart code-follow.service
systemctl enable code-follow.service
systemctl restart mqtt-control.service
systemctl enable mqtt-control.service