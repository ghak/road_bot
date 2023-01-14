# road_bot
Freenove car control using ros

There are 3 ways to use this repo
==================================
1)The ros1 branch gives all what is needed to control the car with a joystick using ros1. 
- You can download all the repo on the RPI and launch the server and client launch files
- You can download the repo on RPI and computer. launch the client "client.launch" launch on RPI and the server "server.launch" on computer. Don't forget to export the ROS_MASTER_URI and ROS_IP on both machines

2)Ros1 and Ros2 branches using ros1_bridge.
- Download the ros2 branch and start the ros1_bridge launch file "ros1_server_launch.xml"
- Download the ros1 branch and start the client launch file on RPI

3)Ros2
- Download the standalone branch on RPI and start the server bash file "start_serv.bash"
- Download the ROS2 branch and start the ros2 launch file "stand_alone_server_launch.xml"
- change the ip adress of RPI on "reg_defines.py"

Required:
- Ros package Joy
- Ros package Image_view


