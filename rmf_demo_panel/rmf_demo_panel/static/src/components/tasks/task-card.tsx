import React from 'react';
import Button from '@material-ui/core/Button';
import Card from '@material-ui/core/Card';
import CardContent from '@material-ui/core/CardContent';
import Chip from '@material-ui/core/Chip';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
import Progress from 'antd/lib/progress';
import { useTaskCardStyles } from '../styles';
import { cancelTask } from '../services';

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
                  <Grid item xs={12}><Button variant="outlined" onClick={() => cancelTask(taskState.task_id)}>Cancel Task</Button></Grid>
              </Grid>
            </CardContent>
        </Card>
    );                              
}
