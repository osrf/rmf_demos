# RMF Demo Panel

This will be a replacement of rviz's [rmf_panel](https://github.com/osrf/rmf_schedule_visualizer)

## Installation
Dependencies
 - rmf_core: [`develop/dispatcher_demo`](https://github.com/osrf/rmf_demos/tree/develop/dispatcher_demo) branch

Setup `rmf_demo_panel`
```bash
cd $ROS2_WS
npm install --prefix src/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/
npm run build --prefix src/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/
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

Take `office.world` as an example:

**Task List***
On the right side column, you are able to select a file which consists of scheduled 
tasks. Select tasks for office: `rmf_demos/rmf_demo_panel/task_list/office_tasks.json`. 
Once the tasks are populated in the box, hit submit!

**Adhoc Task***
User can also submit adhoc task request. This can be done by selecting the 
Request form on the Left panel.

The latest robot states and task summaries will be reflected at the bottom portion of the GUI.

Similarly, Please try out different world.

## Note

- Edit the `dashboard_config.json` to configure the input of the Demo World GUI Task Submission.
The File is located here: `rmf_demo_panel/static/src/components/config/$WORLD`
