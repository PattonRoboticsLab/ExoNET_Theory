# How to Use ROS Actions 

## What are ROS Actions?

ROS actions are another way to implement Client-Server interactions in ROS. Outside of ROS actions there are ROS topics and ROS services that allow clients and servers to interact with each other. 

ROS actions are designed for situations where the client wants to make a request to the server and that request will take a long time to complete (in other words won't be complete instantly). For example, let's say the client wants to request the server to move the robot 100 feet forward. This type of interaction would best be implemented using a ROS action because the server cannot simply complete the request instantly; it has to move the robot 100ft! 

ROS actions also allow the client to do two important things: *receive feedback/progress on the request* and *cancel a request while its being executed* 

## Components of an Action

ROS Actions are actually built upon ROS messages. When the programmer defines a ROS action, the definition actually gets translated down to ROS messages that are sent over ROS topics. In other words, a ROS action is implemented using ROS messages and topics. 

A ROS action is defined using an *action specification* An action specification can really be broken down into three components:

**1. Goal:** The goal is defined by the client. It essentially the request to the server and depends on the scenario at play.   For example, let's take the situation again where the client wants to request the server to move the robot 100ft. The goal in this situation would be the Pose of the robot after it has moved 100ft. 

**2. Feedback:** The server has the ability to send the client feedback or progress on the request. Feedback could take on several forms depending on the situation. It could be time left until a request is complete or a percentage of progress. It can also be the current state of the request (ex: the robots current position). 

**3. Result:** The result is a piece of information that is specified and returned by the server, **only once**, at the end of the request. This is very useful when the purpose of the request is for the client to receive some information. For example, a client could request the server to generate a point cloud of an environment. The result of this action would be the point cloud itself. 

## .action files

The specification we talked about in the section above is defined in a *.action* file. The .action file has sections that define each of the component above: Goal, Feedback, and Result. Each section is then separated by three hyphens (---). An action file is always placed in a directory called: *action* in a package. Here is an example of a .action file:

```
#Define the goal
uint32 dishwasher_id #Specify which dishwasher we want ot use
---
#Define the result
uint32 total_dishes_cleaned
---
#Define a feedback message
float32 percent_complete
```

## Using Catkin to build Actions

After we have defined our ROS action in a .action file, we must build the action (i.e translate the action in messages and topics). As expected, Catkin will do this for us. To allow Catkin to locate our newly created action, we must make the following changes:

Add the following to your package's CMakeLists.txt:

```
find_package(catkin REQUIRED genmsg actionlib_msgs)
add_action_files(DIRECTORY action FILES <name of your .action file here>)
generate_messages(DEPENDENCIES actionlib_msgs)
```

Add the following to your package's package.xml: 
```
<depend>actionlib</depend>
<depend>actionlib_msgs</depend>
```

Formatting your Catkin files can be a headache sometimes. It is best to use ROS documentation and online resources, as well as the error messages you get from Catkin, to debug your issues. 

## Implementing an Action/More Information

Please refer the reference #1 for more information on how to create an ActionClient or ActionServer in Python or C++. Reference #2 is great read to learn more about how Actions are implemented "under the hood"; however, it is not required to actually use ROS Actions. 

## References 

1. http://wiki.ros.org/actionlib
2. http://wiki.ros.org/actionlib/DetailedDescription
