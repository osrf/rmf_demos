import * as React from "react"
import { Divider, Grid, Typography } from "@material-ui/core";
import ScheduledTaskForm from "./forms/scheduled-task-form";
import RobotContainer from "./robots/robot-cards-container";
import TasksContainer from "./tasks/tasks-container";
import { usePanelContainerStyles } from "./styles";
import RequestForm from "./forms/request-form";

const PanelsContainer = (): React.ReactElement => {
    const classes = usePanelContainerStyles();

    return (
            <Grid className={classes.panels}>
                <Grid container direction="row" alignContent="center" justify="center">
                    <Grid item xs={12}>
                        <Typography className={classes.centered} variant="h4">Task Submissions</Typography>
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
                    <Grid item xs={6}>
                        <TasksContainer />
                    </Grid>
                </Grid>
            </Grid>
    )
}

export default PanelsContainer;