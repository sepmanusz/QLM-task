# QLM-homework

### This is a brief description regarding the solved task.

Primarily for testing purposes, I created an **additional Node**, which can be found in the **publisher.cpp** file.
This node sends a message every 1.5 seconds with a Float64 type data to the **/input_numbers topic**. 
This topic will be read by a node named **sender_receiver** (**subscriber.cpp**).

In the publisher Node, I implemented a function necessary for testing, which sends 0 in two consecutive messages to the topic (this is necessary to ensure there is a 0 in the denominator during division).
Therefore, the **sender_receiver** Node would perform division by 0, but due to error handling implementation, it simply skips this division.
In the **sender_receiver** node, I handle data input using a vector array. I load every incoming data into the array and use the last two elements for division. 
Division only occurs when the **received_count_** variable is even (meaning we have at least 2 data).
When this condition is met, the division function is invoked, which performs the operation and publishes the result to the **division_result** topic.

Additionally, I created another function that resets this buffer after a certain array size to prevent overflow.

### Instruction to software building
The necessary files for testing are contained in the folder named "qlm_task".
I personally used three terminals during testing.
Open the terminals in the folder where you placed the downloaded "qlm_task" folder.

Commands for the **first** terminal:
```
colcon build --packages-select qlm_task
. install/setup.bash
ros2 run qlm_task talker
```
**Second** terminal: 'sender_receiver node'

This node will log the input datas and the result of the division.
If an error occured or the buffer resets, it will be also noted.
```. install/setup.bash
ros2 run qlm_task listener
```
**Third** terminal: 'division_result topic'

In this terminal you can see the result.
```
ros2 topic echo division_result
```
### Challenges druing implementation
From my perspective, the task contained many novelties, as I had not previously dealt with ROS2, and building a Linux environment was also new to me.
I ramped up in this topic in 2 days.(I believe I have maximized the topic to demonstrate my ability to learn new concepts.)

During the task description, it was not clear to me whether it was allowed to create a new node, and during error handling,
it occurred to me that the data type could also be checked to ensure that the correct data was coming to the topic.
But as far as I know, when a listener subscribes to a topic, the node in question only listens to one data type,
so theoretically it should not cause any problems.

Furthermore, it could have been simplified to send the data, as it is possible to use the std_msgs/Float64MultiArray Message to send/receive messages not one by one. However, I did not make this simplification due to the task description.
