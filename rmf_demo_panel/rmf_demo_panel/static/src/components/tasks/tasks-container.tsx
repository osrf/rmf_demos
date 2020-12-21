import * as React from 'react';
import { Box, Button, Grid, Typography } from '@material-ui/core';
import { TaskCard } from './task-card';
import { useContainerStyles } from '../styles';
import { getTasks } from "../services";
import { Progress } from 'antd';

const TasksContainer = () : React.ReactElement => {
    const classes = useContainerStyles();
    const [taskStates, setTaskStates] = React.useState([]);
    const exampleState  = {
        taskID: 4000,
        description: "zone_x",
        robot_name: "magnus",
        state: "Active",
        task_type: "Clean",
        start_time: 1500,
        end_time: 1540,
        progress: "42"
    };

    const refreshTaskData = async () => {
        let updatedData = await getTasks();
        if(updatedData != taskStates) {
                setTaskStates(updatedData);
        }
    }

    //refresh the tasks summary at intervals of 5 seconds
    React.useEffect(() => {
        const timer = setInterval(() => {
            refreshTaskData();
        }, 5000);
        return () => clearInterval(timer);
    });

    const allTasks = taskStates.map(taskState => {
        return <Grid item><TaskCard taskState={taskState} /></Grid>
    });

    return (
        <Box className={classes.container}>
            <Typography variant="h5">Tasks</Typography>
            <Button variant="outlined" onClick={refreshTaskData}>Refresh</Button>
            <Grid container className={classes.grid}>
                <TaskCard taskState={exampleState} />
                {allTasks}
            </Grid>
        </Box>
    )
}

export default TasksContainer;