import * as React from 'react';
import { Card, CardContent, Chip, Grid, Typography } from '@material-ui/core';
import { Progress } from 'antd';
import { useTaskCardStyles } from '../styles';

interface TaskCardProps {
    taskState: {
        task_id: string,
        description: string,
        robot_name: string,
        state: 	string,
        task_type: string,
        start_time: number,
        end_time: number,
        progress: string,
    }
}

export const TaskCard = (props: TaskCardProps) : React.ReactElement => {
    const { taskState } = props
    const classes = useTaskCardStyles();

    const returnStateColor = (state: string) => {
      switch(state) {
        case "Failed":
          return "secondary";
        default:
          return "primary";
      }
    }

    return (
        <Card className={classes.root} variant="outlined" role="task-details">
            <CardContent>
              <Grid container>
                  <Grid item xs={12}>
                    <Progress type="dashboard" gapDegree={120} percent={parseInt(taskState.progress)} />
                  </Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      Task ID
                    </Typography>
                  </Grid>
                  <Grid item xs={6}><Typography>{taskState.task_id}</Typography></Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      Details
                    </Typography>
                  </Grid>
                  <Grid item xs={6}><Typography>{taskState.description}</Typography></Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      Robot
                    </Typography>
                  </Grid>
                  <Grid item xs={6}><Typography>{taskState.robot_name}</Typography></Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      Task Type
                    </Typography>
                  </Grid>
                  <Grid item xs={6}><Typography>{taskState.task_type}</Typography></Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      Task State
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>
                    <Chip color={returnStateColor(taskState.state)} label={taskState.state} size='small'/>
                  </Grid>
                  <Grid item xs={3}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      Start: 
                    </Typography>
                  </Grid>
                  <Grid item xs={3}><Typography>{taskState.start_time}</Typography></Grid>
                  <Grid item xs={3}>
                    <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                      End: 
                    </Typography>
                  </Grid>
                  <Grid item xs={3}><Typography>{taskState.end_time}</Typography></Grid>
              </Grid>
            </CardContent>
        </Card>
    );                              
}
