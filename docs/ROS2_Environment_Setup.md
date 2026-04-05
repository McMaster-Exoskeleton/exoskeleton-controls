**ROS 2 Environment Setup**

\-Written by Muiz Hamzat

**Overview**  
This document contains for building and running tests for a ROS 2 package that defines custom messages for the exoskeleton control system. These messages will be used for communication between the Raspberry Pi and microcontrollers. 

These instructions assume that you have **Ubuntu 22.04** (having it through WSL works too). It is important that the Ubuntu version is 22.04, as the version of ROS 2 being used is for this version of Ubuntu.

It is also assumed that you have ROS 2 installed on Ubuntu. If not, you can install ROS 2 by following these instructions: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

**Instructions**

Open a terminal, then source ROS (load the ROS environment into the shell). If you want to run ROS nodes within a terminal, this must be done at least once within that terminal. Without this, your terminal doesn’t know ROS exists.
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
```

Optional: If you don’t want to source ROS every time you open a new terminal, you can run the following command, and the ROS environment will be loaded automatically every time you open a new terminal.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Make sure you have this repo cloned somewhere on your device (it doesn’t have to be in the ROS 2 workspace):   
[McMaster-Exoskeleton/exoskeleton-controls at ros-msg-test](https://github.com/McMaster-Exoskeleton/exoskeleton-controls/tree/ros-msg-test)

Then within that repo, checkout to the ros-msg-test branch.
```bash
git checkout ros-msg-test
```

Once you’ve done that, you can now create a ROS 2 workspace and copy the packages from the repo into it.
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Copy the packages from github into your src folder
cp -r <path_to_repo>/src/* src/
```

The result should be something like the following:
```bash
ros2_ws/
 ├── src/
 │    ├── exo_msgs
 │    └── exo_test
```


---
**Building and running nodes**

After creating (or in our case cloning in) nodes or after updating code within the ROS packages, we have to build them before we can use them. Running the colcon build  command may take some time depending on the size and content of the packages.  

We also need to source our workspace after it’s been built, for a similar reason we need to source ROS. Sourcing your workspace essentially adds your packages and allows them to be seen by ROS.
```bash
cd ~/ros2_ws
# Build the packages. Building may take about a minute
colcon build --packages-select exo_msgs exo_test

# Source the workspace
source install/setup.bash
```

We can now test the nodes. The terminal you have been using will act as a publisher node (send out messages), while another terminal will act as a subscriber node (will receive those messages). In the current terminal, run the following command:
```bash
ros2 run exo_test test_publisher
```

There should be messages being sent in the terminal.  
While that terminal is running, open a new terminal, source ROS, go to your ROS 2 workspace, then source your workspace.
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 run exo_test test_subscriber
```

The subscriber terminal should be receiving the messages from the publisher terminal.

To stop the publisher terminal from running, you can go to that terminal and press Ctrl \+ C. You will notice that the subscriber node will also stop printing messages. This is because there is no longer anything to receive. Had we stopped the subscriber node first instead of the publisher node, the publisher node would have kept sending messages, as it does not know/care what is subscribed to its messages.  
You can go ahead and stop the subscriber node as well if not done so already.
