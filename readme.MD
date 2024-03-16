# `rosCLIPS60`: A ROS node binding for CLIPS 6.0
This repo provides a bridge for connecting CLIPS 6.00 (version of 05/12/93) with ROS.
This CIPS solution is purely written in C and includes no patches, fixes, or anything.

Why did I use this old version?
Don't even dare to ask. I was required to because *bananas*. If you're looking for a newer versuion please do check [rosCLIPS64](https://github.com/kyordhel/rosCLIPS64).

Note: a *banana* is sufficient reason to do something, so imagine many.

# QuickStart

This guide assumes that you have ROS already installed.

## Step 1: Workspace setup and download
1. Source ROS into your shell
```
source /opt/ros/<distro>/setup.bash
```

For example
```
source /opt/ros/noetic/setup.bash
```

2. Create a workspace directory for testing this solution
```
mkdir -p catkin_ws-clips/src
```

3. Move inside the workspace directory and clone this repo as `rosclips` inside `src`
```
cd catkin_ws-clips
git clone https://github.com/kyordhel/rosCLIPS60 src/rosclips
```

4. Build the source (it may take a while)
```
catkin_make
```


## Step 2: Run and test
We will be running 4 terminals for this quickstart:
- `roscore`
- Clips
- An output listener (`rostopic`)
- An input trigger (`rostopic`)

It is strongly advised that you keep Clips and the output listener visible.



1. In all FOUR separate terminals source ROS and the workspace with:
```
source /opt/ros/<distro>/setup.bash
source devel/setup.bash
```

2. In terminal ONE start roscore
```
roscore
```

3. In terminal TWO start the output listener
```
rostopic echo /planner
```

4. In terminal THREE start CLIPS
```
rosrun clips_ros clips_ros
```

5. In terminal FOUR send the `(reset)` signal to CLIPS with the following command:
```
rostopic pub --once /clips_in std_msgs/String "(reset)"
```

Things will start to move. If everything went right in the listener (T2) you should see something like this:

```
data: "move block D to the floor"
---
data: "move block E to the floor"
---
data: "move block A to the floor"
---
data: "move block B to the floor"
---
data: "move block F on top of blockC"
---
```


# Topics:
The CLIPS-ROS bridge will publish to any topic on the fly.
Just use the function `(rospub ?topic ?string)` in CLIPS. 
Only std_msgs/String messages are supported.

Likewise, the CLIPS-ROS bridge will subscribe to any topic on the fly.
Just use the function `(rossub ?topic)` in CLIPS. 
Only std_msgs/String messages are supported.

The CLIPS-ROS bridge will listen to all the input topics the same and assert them as facts.
An exception is messages containing any of the following words (case insensitive):
- CONTINUE: discard message
- FACTS: watch facts
- RULES: watch rules
- UNTRACE: disable tracing
- TRACE: enable tracing
- NUMBER: number of events to trace
- UNWATCH: unwatch rules and facts
- RESET: reset command

On startup the bridge automatically listens to the `clips_in` topic.

# Functions
It is possible to define new functions to trigger different behaviors.
This requires modifying the code.
to achieve this you need to define your user functions in the `user_functions` function in `main.cpp` using the C-CLIPS function `DefineFunction`.
Please reffer to the documentation to understand how this works.
