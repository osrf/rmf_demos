# RMF Demo Panel

This will be a replacement of rviz's [rmf_panel](https://github.com/osrf/rmf_schedule_visualizer)

## Installation
Dependencies
 - rmf_core: `feature/task-dispatcher` branch

```bash
colcon build --packages-select rmf_demo_panel
```

# Run 
```
# first terminal
ros2 run rmf_demo_panel dispatcher_gui

# second terminal
ros2 run rmf_task_ros2 rmf_task_dispatcher
```

## Run Sample Tasks

Open http://localhost:5000/ on browser

On the left hand Column, you are able to select a file which consists of scheduled tasks.

The tasks lists are located here `rmf_demos/rmf_demo_panel/task_list`.

Select the desired file according to the world you are runnning, then hit submit! 

