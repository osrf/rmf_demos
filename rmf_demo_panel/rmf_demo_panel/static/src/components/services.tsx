//calling API endpoints

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

//calling config files
import officeConfig from "./config/office/dashboard_config.json";
import airportConfig from "./config/airport/dashboard_config.json";
import clinicConfig from "./config/airport/dashboard_config.json";

export const getDefaultConfig = async () => {
    let response = await fetch(officeConfig.toString()).then(resp => resp.json());
    return response;
}

export const getConfigFile = async (folderName: string) => {
    let config;

    switch(folderName) {
        case 'Office':
            config = await fetch(officeConfig.toString())
            .then(resp => resp.json());
            console.log("config file", config);
            return config;

        case 'Airport':
            config = await fetch(airportConfig.toString())
            .then(resp => resp.json());
            console.log("config file", config);
            return config;

        case 'Clinic':
            config = await fetch(clinicConfig.toString())
            .then(resp => resp.json());
            console.log("config file", config);
            return config;
    }
}
