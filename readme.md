# TurtleBot Simulation Project

This project simulates various TurtleBot behaviors within the `turtlesim` environment using ROS.  It demonstrates different control strategies and algorithms, including PID control, predictive pursuit, and handling of noisy sensor data.

## Overview

The project consists of several Python scripts, each focusing on a specific goal:

* **`goal1_control_turtle.py`:** Controls a turtlebot to reach a randomly selected goal position using a PID controller.
* **`goal2_make_grid.py`:** Guides a turtlebot through a grid pattern, incorporating acceleration and deceleration limits for more realistic movement.
* **`goal3_rotate_turtle_in_circle.py`:** Makes a turtlebot move in a continuous circular path, publishing both real and noisy pose data.
* **`goal4_chase_turtle_fast.py`:** Simulates a fast chase between a "Robber Turtle" and a "Police Turtle," with the police turtle using real-time pose information.
* **`goal5_chase_turtle_slow.py`:** Simulates a chase where the police turtle is slower and must predict the robber turtle's future position.


## Running the Simulations

Before running any of the scripts, ensure you have ROS and the `turtlesim` package installed and configured.  You can typically install them using:

bash
sudo apt update
sudo apt install ros-noetic-turtlesim # Replace 'noetic' with your ROS distribution if needed

Then, for each script, follow these steps:

1. **Open a new terminal.**
2. **Source your ROS environment:**  `source /opt/ros/noetic/setup.bash` (replace `/opt/ros/noetic` with your ROS installation path).
3. **Run the script:** `rosrun <your_package_name> <script_name>.py`  (Replace `<your_package_name>` with the name of the ROS package where you've placed these scripts.  If they are not in a ROS package, you'll need to run them directly using `python3 <script_name>.py`).

**Example:** To run `goal1_control_turtle.py`, assuming it's in a package named `turtlebot_simulations`, you would run:

bash
rosrun turtlebot_simulations goal1_control_turtle.py

you need to run the below command to start turtlesim for all the files

bash
rosrun turtlesim turtlesim_node

You can then visualize the turtle's movement in the `turtlesim` window.  Some scripts also publish data to ROS topics that can be visualized using tools like `rqt_plot`.


## Dependencies

* ROS (Robot Operating System)
* `turtlesim` package


## Notes

* The scripts assume a basic understanding of ROS concepts like publishers, subscribers, and services.
* Some scripts use random numbers to generate starting positions or goals, so the behavior might vary slightly on each run.
* For the chase simulations (`goal4_chase_turtle_fast.py` and `goal5_chase_turtle_slow.py`), you might need to adjust parameters to fine-tune the chase behavior.


