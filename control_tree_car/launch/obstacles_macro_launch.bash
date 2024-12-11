#!/bin/bash

# Define the launch file to run and the duration (in seconds) for each execution
LAUNCH_FILE="control_tree_car obstacle_avoidance.launch"
DURATION=1800  # Run each instance for this duration

# Define parameters
PARAMS=(       # Tuple 1: (tree, p_obstacles, full_observability)
    "true 0.1 false src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world"      
    "false 0.1 false src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world"   
    "false 0.1 true src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world"         
    "true 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world"      
    "false 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world"   
    "false 0.25 true src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world" 
)

# Loop through the parameter values
for TUPLE in "${PARAMS[@]}"; do
  # Split the tuple into individual parameters
  tree=$(echo $TUPLE | awk '{print $1}')
  p_obstacle=$(echo $TUPLE | awk '{print $2}')  
  full_observability=$(echo $TUPLE | awk '{print $3}')  
  gz_file=$(echo $TUPLE | awk '{print $4}')  
    
  echo "Running with parameter: tree=$tree, p_obstacle=$p_obstacle, full_observability=$full_observability"
  
  # Launch the file with the parameter
  roslaunch $LAUNCH_FILE tree:=$tree p_obstacle:=$p_obstacle full_observability:=$full_observability &
  
  # Get the process ID of the last background process
  PID_ros=$!
  
  gzserver $gz_file &
  
  # Get the process ID of the last background process
  PID_gz=$!
  
  echo "gzserver launched with PID: $PID_gz"

  # Wait a moment to allow child processes to start
  sleep 2

  # Retrieve all related PIDs
  CHILD_GZ_PIDS=$(pgrep -P $PID_gz)
  ALL_GZ_PIDS=$(pgrep gzserver)

  echo "Main gzserver PID: $PID_gz"
  echo "Child PIDs: $CHILD_GZ_PIDS"
  echo "All gzserver-related PIDs: $ALL_GZ_PIDS"



  
  # Sleep for the fixed duration
  sleep $DURATION
  
  echo "kill gzserver.. $PID_gz"
  
  # Terminate all gzserver-related processes
  for PID in $ALL_GZ_PIDS; do
    kill -INT $PID
    wait $PID
  done

  
  
  # Kill the roslaunch process and wait for the process to terminate cleanly
  rosnode kill -a
  
  echo "kill ros node.."
  kill -INT $PID_ros
  
  echo "wait.."
  wait $PID_ros
    
  echo "all finished.."
done

echo "All runs completed."

