# RMF Demo Panel

This will be a replacement of rviz's [rmf_panel](https://github.com/osrf/rmf_schedule_visualizer)

## Installation
Dependencies
 - rmf_core: `feature/task-dispatcher` branch

Setup `rmf_demo_panel`
```
cd rmf_demo_panel/rmf_demo_panel/static
npm install
npm run build
```

Pkg compilation
```bash
colcon build --packages-select rmf_demo_panel
```


## Run 
Test Run with office world

1. Start Office World
```bash
ros2 launch demos office.launch.xml
```

Run with gazebo simulation
```bash
ros2 launch demos dispatcher.launch.xml
```

## Run Sample Tasks

Open `http://localhost:5000/` on browser

On the left hand column, you are able to select a file which consists of scheduled tasks.

Select tasks for office: `rmf_demos/rmf_demo_panel/task_list/office_tasks.json`.

Once the tasks are populated in the box, hit submit!

Similarly, this works with other worlds
