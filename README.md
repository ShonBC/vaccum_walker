# vaccum_walker
Implementation of a simple walker algorithm much like a Roomba robot vacuum cleaner. The robot should move forward until it reaches an obstacle (but not colliding), then rotate in place until the way ahead is clear, then move forward again and repeat.

# Prerequisites:
- Ubuntu 18.04
- ROS Melodic
- ROS Beginner tutorials installed:
    sudo apt-get install ros-melodic-ros-tutorials
- Turtlebot3 ROS Package

# Build and Run:
Clone the repo in the src folder of a catkin workspace.

    git clone https://github.com/ShonBC/vaccum_walker.git

Build the packages in the workspace. 
If the catkin workspace was created using catkin_make then navigate to the catkin workspace directory and run:

    catkin_make
    . ~/<catkin_ws_directory_name>/devel/setup.bash

If the workspace was created using catkin build then run:

    catkin build
    . ~/<catkin_ws_directory_name>/devel/setup.bash

# Run the Walker_Node:
By default the turtlebot_walker.launch file will not record a bag file. To run the node and record a bag file run:

    roslaunch vaccum_walker turtlebot_walker.launch record_bag:=true

The bag file will be recorded in the [docs/bag](docs/bag) directory. To examine the bag file first navigate into the [docs/bag](docs/bag) directory in a terminal and run:

    rosbag info walker_bag.bag
    
To play the bag file back and see the published messages first ensure a ROS master is active, in a terminal run:

    roscore

In a new terminal echo the cmd_vel topic:

    rostopic echo /cmd_vel

In a new terminal navigate to the [docs/bag](docs/bag) directory and run:

    roscd vaccum_walker/docs/bag
    rosbag play walker_bag.bag

Generate cppcheck and cpplint results and store in a text file in [/results](results) directory:

    chmod +x run_cpplint.sh
    ./run_cpplint.sh

    chmod +x run_cppcheck.sh
    ./run_cppcheck.sh
