import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
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
            battery_percent: "99.99998474121094",
            level_name: "Level 2"
        };

    const exampleStateCharge  = {
            robot_name: "nano",
            fleet_name: "fleet1",
            assignments: "1004",
            mode: "Charging-1",
            battery_percent: "40.04",
            level_name: "Level 1"
        };
    const exampleStateDock  = {
            robot_name: "nano",
            fleet_name: "fleet1",
            assignments: "1004",
            mode: "Dock/Clean-7",
            battery_percent: "70.02",
            level_name: "Level 1"
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
                <RobotCard robotState={exampleStateCharge} />
                <RobotCard robotState={exampleStateDock} />
                {allRobots}
            </Grid>
        </Box>
    )
}
export default RobotContainer;