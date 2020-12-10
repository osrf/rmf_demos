import * as React from 'react';
import { Grid, Divider, Typography } from '@material-ui/core';
import RequestForm from "../forms/request-form";
import ScheduledTaskForm from "../forms/scheduled-task-form";
import RobotContainer from "../robots/robot-cards-container";
import TasksContainer from "../tasks/tasks-container";
import { usePanelContainerStyles } from "../styles";

const OfficeDashboard = (): React.ReactElement => {
    const classes = usePanelContainerStyles();

    return (
        <div>
            <Grid className={classes.panels}>
                <Grid container direction="row" alignContent="center" justify="center">
                    <Grid item xs={12}>
                        <Typography className={classes.centered} variant="h4">Request Submission</Typography>
                    </Grid>
                </Grid>
                <Grid container direction="row" justify="space-evenly" alignItems="flex-start" spacing={2}>
                    <Grid item xs={5}>
                        <RequestForm />
                    </Grid>
                    <Grid item xs={5}>
                        <ScheduledTaskForm />
                    </Grid>
                </Grid>
                <Divider variant="middle"/>
                <Grid container direction="row" alignContent="center" justify="center">
                    <Grid item xs={12}>
                        <Typography className={classes.centered} variant="h4">Robots & Tasks Summaries</Typography>
                    </Grid>
                </Grid>
                 <Grid container direction="row" justify="space-evenly" alignItems="flex-start" spacing={2}>
                    <Grid item xs={5}>
                        <RobotContainer />
                    </Grid>
                    <Grid item xs={5}>
                        <TasksContainer />
                    </Grid>
                </Grid>
            </Grid>
        </div>
    )
}

export default OfficeDashboard;
