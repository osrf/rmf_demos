import * as React from 'react';
import { Box, Button, Grid, Typography } from '@material-ui/core';
import { RobotCard } from './robot-card';
import { useContainerStyles } from '../styles';
import { getRobots } from "../services";

const RobotContainer = () : React.ReactElement => {
    const classes = useContainerStyles();
    const [robotStates, setRobotStates] = React.useState([]);

    //example state can eventually be removed
    const exampleState  = {
            robot_name: "magnus",
            fleet_name: "fleet1",
            assignments: "1003",
            mode: "Idle-0",
            battery_percent: "90",
            level_name: "Level 2"
        };

    const refreshRobotData = async () => {
        let updatedData = await getRobots();
        if(updatedData != robotStates) {
            setRobotStates(updatedData);
        }
    }

    React.useEffect(() => {
        const timer = setInterval(() => {
            refreshRobotData();
        }, 2000);
        return () => clearInterval(timer);
    });

    const allRobots = robotStates.map(robotState => {
        return <Grid item><RobotCard robotState={robotState} /></Grid>
    });

    return (
        <Box className={classes.container}>
            <Typography variant="h5">Robots</Typography>
            <Button variant="outlined" onClick={refreshRobotData}>Refresh</Button>
            <Grid container className={classes.grid} >
                <RobotCard robotState={exampleState} />
                {allRobots}
            </Grid>
        </Box>
    )
}
export default RobotContainer;