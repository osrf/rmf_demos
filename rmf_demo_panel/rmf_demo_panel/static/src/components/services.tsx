import { showErrorMessage, showSuccessMessage } from './fixed-components/messages';
//API endpoints
export const getRobots = async () => {
    try {
        let response = await fetch('/get_robots');
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
        let response = await fetch('/get_task');
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
        fetch('/submit_task', {
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
