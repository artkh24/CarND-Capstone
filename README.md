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
Base waypoints is a list of all the waypoints on the driving lane. The car needs to figure out, what waypoints are ahead of its current position.

- **/traffic_waypoint:**
Waypoint updater receives this waypoint information from the Traffic light detection node.
Traffic waypoint is the waypoint at which the car is going to stop when traffic light is red.
Waypoint Updater node planning velocities so the car can stop smoothly when traffic light is red.

- **/current_pose:**
This is the current position of the car.
Waypoint Updater are using this information to estimate car's current position relative to the base waypoints

- **/obstacle_waypoints:**
This is to determine the obstacle positions in terms of their waypoint positions.
But in this project we don't have obstacles so we are not using them.

As an output, the Waypoint Updater node will publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights

##### Implementation

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

- Final waypoints are generated from base waypoints in the ```generate_lane``` method,
that is responsible for returning ```Lane``` object to the final waypoints publisher.
  - This method makes use of ```get_closest_waypoint_idx``` helper method, which in turn
  uses KDTree to determine the closest waypoint index.
  - Based on the input from Traffic Light Detection node, this method also determines
  the target velocities for each waypoint using ```decelerate_waypoints``` helper method


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

Outputs from the DBW node are the throttle, brake and steering commands published to the throttle_cmd, brake_cmd and steering_cmd topics.

##### Implementation

DBW node subscribes  to ```twist_cmd```, ```current_velocity``` and ```/vehicle/dbw_enabled``` topics:

```
rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
rospy.Subscriber('/twist_cmd',TwistStamped, self.twist_cb)
rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
```
The internal logic in DBW node extracts the necessary information from the twist command and current velocity messages. Such as linear and angular velocity. And publishes to respective topic using ```publish``` method as follows:

```
self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)
self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)
```


### Traffic Light Detection and Classification

##### Description
The perception block consists of Obstacle Detection and Traffic Light Detection node. For this project only traffic lights were considered as obstacles.
Traffic Light Detection node takes in data from the ```/image_color```, ```/current_pose```, and ```/base_waypoints``` topics and publishes the locations to stop for red traffic lights to the ```/traffic_waypoint``` topic. As mentioned earlier, Waypoint Updater node will make use of this information to determine vehicle velocity for given waypoints.
The traffic light classifier node is implemented separately and is independent of the Traffic light Detection node, which implements the logic to publish information regarding where the vehicle should come to a stop.

##### Inputs and Outputs

The inputs and outputs to Traffic Light Detection node are shown below:

![](/images/tl-detector-ros-graph.png)

The inputs to Traffic Light Detection node are:

- **/base_waypoints:**
Provides the complete list of waypoints for the course. This is the same list as the list used by Waypoint Updater node and is sent only once at initialization.

- **/image_color:**
This topic  provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights by traffic light classifier node.

- **/current_pose:**
Traffic Light Detection node will receive this information from the simulator (or car's localization block in real world scenario) to determine the current position of the car.


The output of Traffic Light Detection node is:

- **/traffic_waypoint:**
This is the topic to which Traffic Light detection node will publish the index of the waypoint for nearest upcoming red light's stop line. This will be used as an input by waypoint updater node.

##### Implementation

There are two tasks that are performed by traffic light detection node:
1. Use the vehicle's location and the ```(x, y)``` coordinates for traffic lights to find the nearest visible traffic light ahead of the vehicle. This takes place in the ```process_traffic_lights``` method of ```tl_detector.py```. To find the closest waypoints to the vehicle and lights, ```get_closest_waypoint``` method is used.
Using these waypoint indices, we determine which light is ahead of the vehicle along the list of waypoints.

2. Use the camera image data to classify the color of the traffic light. The core functionality of this step takes place in the ```get_light_state``` method of ```tl_detector.py```.
In order to train the classifier we have utilized [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection).

