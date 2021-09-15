```
POLITECNICO DI MILANO
```
```
ROBOTICS 2020 - 2021
prof. Matteo Matteucci
```
**DELIVERABLE**

# Homework 1

```
TITLE
First robotics project
```
```
SCENARIO
SCOUT 2. 0 AgileX Robotics
```
```
GROUP
Chicago
```
```
Team Members
ID Surname Name
10529039 Cappelletti Andrea
```
10568052 Castelli Francesco

10674448 Foini Davide


## INDEX

- FILES INSIDE OUR PROJECT
- ROS Parameters
- TF TREE structure
- START NODES
- ADDITIONAL INFO
- FINAL CONCLUSION


### 1. FILES INSIDE OUR PROJECT

In this section, we present all the files inside ourproject.
To be more clear, we report the tree structure ofour
project through the command line

:~$ tree

Our project directory is called “chicago”.
Inside the project directory we divided the subdirectories
into cfg (config), launch (the launch configurationfor the
project), msg (the messages directory), src (the source
code in C++ of the project) and srv for the services.

Inside the project directory chicago there are alsothe two
files needed for the build of the entire project:
package.xml and CMakeLists.txt.


We used a coherent naming convention for the entire
project, more in detail:

chicago/cfg/
**Parameters.cfg** contains dynamic reconfig params

chicago/launch/
**Launch.launch** contains the directive to start theproject

chicago/msg/
**MotorSpeed.msg** contains the defined message
**CustomOdom.msg** contains the output odometry

chicago/src/
**ApparentBaseline.cpp** the node to compute the apparent
baseline

```
Odometry.cpp the node to compute the odometry
```
chicago/srv/
**SetOdometry.srv** service to set the odometry
**ResetOdometry.srv** service to reset the odometry


```
2. ROS Parameters
```
In this section, we present the parameters we definedin
our project.

Static

```
Starting coordinates
● gear_rateo, double, [ 0. 027 ] (calibrated)
● wheel_radius, double, [ 0. 1575 ]
● apparent_baseline, double, [ 1. 0625 ] (calibrated)
```
We have computed the gear rateo and the apparent
baseline taking into account some consideration thatwe
report in the sections below (additional info).

The wheel radius is already defined from the project
specification and reported in the project structure.

```
Starting coordinates
● x, integer, [ 0 ]
● y, integer, [ 0 ]
● th, integer, [ 0 ]
```
```
Starting integration method (Euler)
● method, integer, [ 0 ]
```
The integration method is defined as an enum withtwo
values, respectively “euler” and “rk”.
“euler” has index 0 , meanwhile “rk” has index 1.

Dynamic
● method_enum, enum, [euler, rk]


```
3. TF TREE structure
```
Below you can find the TF TREE structure of our project:it
is very simple because we chose to use just one frame
“odom” that represents the scout transformed to the
“world” frame.


### 4 .START NODES

In order to launch the nodes, run this command from
terminal

:~$ roslaunch chicago Launch.launch

We organized our work in just two nodes: one for
estimating the apparent baseline of our robot, andthe
second to compute all the required tasks.

**Services**
As presented above, in our project we have two services:
one to set the odometry and one to reset the odometry.

To launch the services run the following commands

**SetOdometry.srv**

rosservice call /SetOdometry [x] [y] [th]

Substituting [x], [y] and [th] with the values.

**ResetOdometry.srv**

rosservice call /ResetOdometry


**Dynamic Reconfiguration**
In order to perform the dynamic reconfiguration, runthe
following command

rosrun dynamic_reconfigure dynparam set

Odometry method [index]

Where [index] is the method chosen

**0** : Euler
**1** : RK


### 5. ADDITIONAL INFO

**Apparent Baseline**
To estimate the apparent baseline, we imported the
information about the motor speeds from the bags,and
we converted them into linear velocities for the
(approximated) two wheels of the robot.

Then by just applying the Skid Steering (approximate)
Kinematics we computed the apparent baseline.
The main problem was that we had a lot of values forit,
and lots of them were not physically possible (i.ezero or
negative values or infinite values).

So what we decided to do was to put some boundarieson
the acceptable apparent baseline values and then
compute an average of them.

This led us to an acceptable value for the apparent
baseline that we then set as a parameter.

**Calibration**
After we completed the odometry computation we
decided to try to calibrate the apparent baselineand the
gear rateo to obtain a better result.
Helping us with visualizing the TF on rviz after sometries
we obtained that the values **1. 0625** for the apparent
baseline and **0. 027** for the gear rateo that combinedcan
get closer to the odometry given by the manufacturer.


**Default parameters**
The default parameters x, y, theta can be changedin the
launch file Launch.launch or via command line withthe
command
rosparam set [param] [value]

param: x/y/th
value: any integer value

Regarding the default integration method, it can be
changed in the Parameters.cfg file modifying the default
value of the enum parameter “method”:

- 0 for Euler;
- 1 for Runge-Kutta.

**Publishers**
We chose to use three publishers:

- **twist_pub** that advertises to the topic /twist the
    TwistStamped messages;
- **custom_pub** that advertises to the topic
    /custom_odom the CustomOdom messages;
- **odom_pub** that advertises to the topic /odometry
    the Odometry messages;


### 6. FINAL CONCLUSION

Once started the node, it can be seen thanks to rvizthat
the trajectory of our robot follows faithfully thegiven
path, even though the two trajectories are slightly
dierent.

In addition, changing the integration method fromEuler
to Runge-Kutta slightly increases the precision ofthe
trajectory.

This frames are taken 60 seconds into bag 1 :


# Homework 2

```
TITLE
Second robotics project
```
```
SCENARIO
SCOUT 2. 0 AgileX Robotics
```
```
GROUP
Chicago
```
```
Team Members
ID Surname Name
10529039 Cappelletti Andrea
```
10568052 Castelli Francesco

10674448 Foini Davide


## INDEX

- FILES INSIDE OUR PROJECT
- TF TREE structure
- BAGS
- START NODES
- SENSORS CHOICE


### 1. FILES INSIDE OUR PROJECT

In this section, we present all the files inside ourproject.
To be more clear, we report the tree structure ofour
project through the command line

:~$ tree

Our project directory is called “boston”.


In the boston directory, we can find the package.xml file
with all the imports needed to run the project andthe
usual CMakeLists.txt.

We structured the project with the main folders: maps,
launch, rviz.

Particularly, maps contain all the files related tothe
mapping as shown by the figure above.

Launch contains the two main launch files, that are
gmapping.launch and localization.launch.

gmapping.launch performs mapping and the
localization.launch performs localization based onthe
map in output from the gmapping.


```
2. TF TREE structure
```
Our project involves just the left part of the tree,the one
which has the map frame as root for the reasons we’ll
explain further on.


### 3. BAGS

Mapping: bag 1
Localization: all the other bags


### 4 .START NODES

We structured our project in two main launch files:

- gmapping.launch performs mapping;
- localization.launch performs localization based on
    the map generated by gmapping.

Both launch files:

- start a tf static transform publisher to compensate
    for the rotation between the odometry and the laser;
- set the use_sim_time parameter to true.

The localization.launch files include other launchfiles
necessary to perform the localization task:

- the ekf node;
- the imu tools node;
- the amcl node;
- map server with loaded map;
- rviz configuration.


### 5. SENSORS CHOICE

We decided for gmapping to use the odometry published

by the scout because usingplotjuggerwe realizedthat
the odometry of the scout and the one given by the
camera were very similar.

For the localization task, we opted again for the
odometry of the scout fused in the ekf with the imudata
(obtained via imu_filter_madgwick) to obtain a filtered
odometry to use during amcl.


