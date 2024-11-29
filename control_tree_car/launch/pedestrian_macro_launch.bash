#!/bin/bash

# Define the launch file to run and the duration (in seconds) for each execution
LAUNCH_FILE="control_tree_car pedestrian.launch"
DURATION=300  # Run each instance for this duration

# Define parameters
PARAMS=(       # Tuple 1: (n_pedestrians, n_branches, p_crossing, full_observability)
    "1 2 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_1.world"      
    "1 1 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_1.world"     
    "1 1 0.05 true  src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_1.world" 
    "1 2 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_1.world" 
    "1 1 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_1.world" 
    "1 1 0.01 true  src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_1.world" 
    "4 5 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"
    "4 4 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"  
    "4 3 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"  
    "4 2 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 1 0.01 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 1 0.01 true  src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 5 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 4 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 3 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 2 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"       
    "4 1 0.05 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"      
    "4 1 0.05 true  src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"    
    "4 5 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 4 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 3 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world" 
    "4 2 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"      
    "4 1 0.25 false src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"     
    "4 1 0.25 true  src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world"  
)

# Loop through the parameter values
for TUPLE in "${PARAMS[@]}"; do
  # Split the tuple into individual parameters
  n_pedestrians=$(echo $TUPLE | awk '{print $1}')
  n_branches=$(echo $TUPLE | awk '{print $2}')  
  p_crossing=$(echo $TUPLE | awk '{print $3}')  
  full_observability=$(echo $TUPLE | awk '{print $4}')  
  gz_file=$(echo $TUPLE | awk '{print $5}')  
    
  echo "Running with parameter: n_pedestrians=$n_pedestrians, n_branches=$n_branches, p_crossing=$p_crossing, full_observability=$full_observability"
  
  # Launch the file with the parameter
  roslaunch $LAUNCH_FILE n_pedestrians:=$n_pedestrians n_branches:=$n_branches p_crossing:=$p_crossing full_observability:=$full_observability &
  
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

