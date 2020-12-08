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