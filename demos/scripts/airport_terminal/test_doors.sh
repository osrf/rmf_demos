#!/bin/bash

echo "Starting basic loop and patrolling scenario."

function spawn_robot {
  ros2 run gazebo_ros spawn_entity.py \
    -database $1 \
    -reference_frame $2_placeholder \
    -entity $2 \
    -z 0.05
}

function send_loop_job {
  ros2 run rmf_demo_tasks delayed_request_loop \
    -s $1 \
    -f $2 \
    -n 100 \
    -r $3 \
    --delay 2
}

echo "Spawning and sending job to robot mir100_0."
spawn_robot MiR100 mir100_0
send_loop_job junction_central_west s08 mir100
