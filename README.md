# Udacity Self-Driving Car Engineer Nanodegree

## Capstone Project

This is the final project of Udacity Self-Driving Car Engineer Nanodegree. The goal of this project is to write ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. The code will be tested on Udacities self driving car 'Carla'.

### Team:

#### Members:
Name | e-mail|Github     
-----| ----- | --------
Artur Khondkaryan | artkh24@gmail.com | [Artur_Khondkaryan](https://github.com/artkh24/)

### System Architecture Diagram
The following is a system architecture diagram showing the ROS nodes and topics used in the project.
Carla has drive-by-wire system (DBW) so the throttle, brake and steering can be electronically controlled. The Control Module gives Throttle, Brake and Steering signal commands to the car or simulator.

![](/images/final-project-ros-graph-v2.png)

### Waypoint Updater

##### Description
The eventual purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.

##### Inputs and Outputs

![](/images/waypoint-updater-ros-graph.png)

The inputs to the Waypoint Updater node are these topics:
- **/base_waypoints:**
Base waypoints is a list of all the waypoints on the driving lane. The car needs to figure out, what waypoints are ahead of its current position. Waypoints  provided by a static.csv file.

- **/traffic_waypoint:**
Waypoint updater receives this waypoint information from the Traffic light detection node.
Traffic waypoint is the waypoint at which the car is going to stop when traffic light is red.
Waypoint Updater node planning velocities so the car can stop smoothly when traffic light is red.

- **/current_pose:**
Current position of the vehicle, provided by the simulator or localization.

- **/obstacle_waypoints:**
This is to determine the obstacle positions in terms of their waypoint positions.
But in this project we don't have obstacles so we are not using them.

As an output, the Waypoint Updater node will publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights

##### Scheme

- Waypoint updater has subscribed to the following topics:

```
rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
```

- The waypoint updater will change waypoints according to the status of the traffic lights and
waypoints will be published using ```publish_waypoints``` API using ```final_waypoints_pub``` publisher:

```
self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
```
- **/final_waypoints:**
This is a subset of /base_waypoints. The first waypoint is the one in /base_waypoints which is closest to the car.

- Final waypoints are generated from base waypoints in the ```generate_lane``` method,
that is responsible for returning ```Lane``` object to the final waypoints publisher.
 

### DBW Node

##### Description

DBW node system electronically controls throttle, brake and steering. DBW node implements logic to accept target linear and angular velocities and publish throttle, brake, and steering commands to  topics.


##### Inputs and outputs

This "drive-by-wire" node subscribes to `/twist_cmd` message which provides the proposed linear and angular velocities.

![](/images/dbw-node-ros-graph.png)


The inputs to the DBW node are following topics:

- **/twist_cmd:**
Twist commands are published by the waypoint follower node. DBW node subscribes to this topic and produces the required output in terms of throttle, brake, and steering commands.

- **/current_velocity:**
This is published by the simulator in our case and used by DBW node to determine the linear velocity of the car and provide it to the controller

- **/vehicle/dbw_enabled:**
This is a status of DBW electronic system that is also published by the simulator in our case. DBW node will use this status to determine whether the brake, throttle and steering are to be published to respective topics or not

Outputs from the DBW node are the throttle, brake and steering commands published to the 
- **/vehicle/throttle_cmd, 
- **/vehicle/brake_cmd,
- **/vehicle/steering_cmd

##### Scheme

DBW node subscribes  to ```twist_cmd```, ```current_velocity``` and ```/vehicle/dbw_enabled``` topics:

```
rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
rospy.Subscriber('/twist_cmd',TwistStamped, self.twist_cb)
rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
```
DBW node extracts the necessary information from the twist command and current velocity messages.
And publishes to the following topics using ```publish``` method:

```
self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)
self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)
```


### Traffic Light Detection and Classification

##### Description
Once the vehicle is able to process waypoints, generate steering and throttle commands, and traverse the course, it will also need stop for obstacles. Traffic lights are only obstacles considered in terms of this project
Traffic Light Detection node takes in data from the ```/image_color```, ```/current_pose```, and ```/base_waypoints``` topics and publishes the locations to stop for red traffic lights to the ```/traffic_waypoint``` topic. 

##### Inputs and Outputs

![](/images/tl-detector-ros-graph.png)

The inputs to Traffic Light Detection node are:

- **/base_waypoints:**
Provides the complete list of waypoints for the course

- **/image_color:**
This topic provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.

- **/current_pose:**
Used to determine the vehicle's location


The output is

- **/traffic_waypoint:**
 Traffic Light detection node will publish the index of the waypoint for nearest upcoming red light's stop line. 

##### Scheme

Traffic light detection node performes 2 tasks:
1. Used vehicle location and traffic light coordianates finds the nearest visible traffic light ahead of the vehicle

2. Used camera image to classify traffic light and traffic light color
To clasify the traffic lights [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection).
was used.
As a pretrained detection model tensorflow [ssd_mobilenet_v1_2017_11_17'](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) 
model was used trained on [Coco dataset](http://cocodataset.org/#home).

The tensorflow object detection api classifies traffic light then after Gausian bluring the openCV minMaxLoc method were used
to define the most bright pixel and then 'brighter_region' method were used in 'tl_classifier.py' to determine the if its red, yellow or green.
Here we are assuming that traffic light has vertical position and from top to bottom first cames red then yellow and then green
To speed up classification process every fifth image was taken for classification
There is also an opportunity in to save generated images with predictions in folder /ros/src/lt_detector/data/light_images

The following are some example of traffic light classification and color detection

![](/images/i1.png)
![](/images/i2.png)
![](/images/i3.png)
![](/images/i4.png)
![](/images/i5.png)
