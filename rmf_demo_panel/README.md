## Installation
Dependencies
 - rmf_core: [`develop/dispatcher_demo`](https://github.com/osrf/rmf_demos/tree/develop/dispatcher_demo) branch

Setup `rmf_demo_panel`
- Ensure you have node v12 installed (see: [node](https://nodejs.org/en/download/package-manager/) or [nvm](https://github.com/nvm-sh/nvm))

Install Flask
```bash
python3 -m pip install Flask flask-socketio flask-cors
```

Compilation
```bash
nvm use 12 //if you have other versions of node
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

Open `http://localhost:5000/` on browser.

Take `office.world` as an example:

**Task List***
On the right side column, you are able to select a file which consists of scheduled 
tasks. Select tasks for office: `rmf_demos_tasks/rmf_demo_tasks/office_tasks.json`. 
Once the tasks are populated in the box, hit submit!

**Adhoc Task***
User can also submit adhoc task request. This can be done by selecting the 
Request form on the Left panel.

The latest robot states and task summaries will be reflected at the bottom portion of the GUI.

Similarly, Please try out different world.

## Create your own GUI

Internally, there are 2 web-based server running behind the scene, namely:

1. `gui_server` (port `5000`): Providing the static gui to the web client. Non RMF dependent
2. `api_server` (port `8080`): Hosting all endpoints for gui clients to interact with RMF

To create your own customize GUI, you will only require to create your own `CUSTOM_gui_server` 
and interact with the existing `api_server`.

## Note
- Edit the `dashboard_config.json` to configure the input of the Demo World GUI Task Submission.
The dashboard config file is located here: `rmf_dashboard_resources/$WORLD/dashboard_config.json`.
- server ip is configureable via `WEB_SERVER_IP_ADDRESS` in the `dashboard.launch.xml`
- cancel task will not be working. A fully functional cancel will be introduced in a future PR.
