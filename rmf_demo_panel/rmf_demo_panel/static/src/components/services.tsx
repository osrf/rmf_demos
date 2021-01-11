import { showErrorMessage, showSuccessMessage } from './fixed-components/messages';

const API_SERVER_ADD = "http://" + location.hostname + ":8080"

//API endpoints
export const getRobots = async () => {
    try {
        let response = await fetch(API_SERVER_ADD + '/get_robots');
        if(response) {
            let data = await response.json()
            console.log("Populate robots: ", data);
            return data;
        } else {
            return;
        }
    } catch (err) {
        throw new Error(err.message);
    }
}

export const getTasks = async () => {
    try {
        let response = await fetch(API_SERVER_ADD + '/get_task');
        if(response) {
            let data = await response.json()
            console.log("Populate tasks: ", data);
            return data;
        }
    } catch (err) {
        throw new Error(err.message);
    }
}

export const submitRequest = (request: {}, type: string) => {
    try {
        fetch(API_SERVER_ADD + '/submit_task', {
            method: "POST",
            body: JSON.stringify(request),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then(res => res.json())
        .then(data => JSON.stringify(data));
        showSuccessMessage(`${type} Request submitted successfully!`);
      } catch (err) {
        console.log(err);
        showErrorMessage(`Unable to submit ${type} Request`);
      }
}

export const cancelTask = (id: string) => {
    try {
        fetch(API_SERVER_ADD + '/cancel_task', {
            method: "POST",
            body: JSON.stringify({ task_id: id }),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then(res => res.json())
        .then(data => JSON.stringify(data));
        showSuccessMessage(`Task ${id} has been cancelled`);
    } catch (err) {
        console.log(err);
        showErrorMessage(`Unable to cancel Task ${id}`);
    }
}

interface Task {
  task_type: string,
  start_time: number,
  description: string
}

export const submitTaskList = (taskList: string | ArrayBuffer) => {
    let global_list_count = 0;
    let global_task_list: Array<Task> = [];
    let i = 0; // to set time "delay"
    let res = "Task List submitted successfully";
    let tempTaskList: any = taskList; //best to remove 'any
    let jsonTaskList: Array<Task> = JSON.parse(tempTaskList);

    //simulate submission of tasks at intervals
    jsonTaskList.forEach((task) => {
      global_task_list.push(task);
      setTimeout(function(i) {
        try {
          fetch(API_SERVER_ADD + '/submit_task', {
            method: "POST",
            body: JSON.stringify(global_task_list[global_list_count]),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
          })
          .then(res => res.json())
          .then(data => JSON.stringify(data));
          global_list_count++;
        } catch (err) {
          res = "ERROR! " + err;
          showErrorMessage(res);
          console.log('Unable to submit task request');
        }
      }, 900*(++i));
    });
    showSuccessMessage(res);
}

//calling config files
import officeConfig from "./config/office/dashboard_config.json";
import airportConfig from "./config/airport/dashboard_config.json";
import clinicConfig from "./config/clinic/dashboard_config.json";

export const getDefaultConfig = async () => {
    let response = await fetch(officeConfig.toString()).then(resp => resp.json());
    return response;
}

export const getConfigFile = async (folderName: string) => {
    let config: object;

    switch(folderName) {
        case 'Office':
            config = await fetch(officeConfig.toString())
            .then(resp => resp.json());
            return config;

        case 'Airport':
            config = await fetch(airportConfig.toString())
            .then(resp => resp.json());
            return config;

        case 'Clinic':
            config = await fetch(clinicConfig.toString())
            .then(resp => resp.json());
            return config;
    }
}
