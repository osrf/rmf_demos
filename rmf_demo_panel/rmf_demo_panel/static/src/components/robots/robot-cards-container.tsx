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
                {allRobots}
            </Grid>
        </Box>
    )
}
export default RobotContainer;