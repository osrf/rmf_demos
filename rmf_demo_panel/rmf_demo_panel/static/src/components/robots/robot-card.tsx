import * as React from 'react';
import { Card, CardContent, Grid, Typography } from '@material-ui/core';
import { Progress } from 'antd';
import { useRobotCardStyles } from '../styles';

interface RobotCardProps {
  robotState: {
    robot_name: string;
    fleet_name: string;
    assignments: string;
    mode: string;
    battery_percent: string;
    level_name: string;
  };
}

export const RobotCard = (props: RobotCardProps) : React.ReactElement => {
    const { robotState } = props
    const classes = useRobotCardStyles();

    return (
        <Card className={classes.root} variant="outlined">
            <CardContent>
              <Grid container>
                <Grid item xs={12}>
                    <Typography variant="subtitle1" color="textSecondary" className={classes.text}>
                      {robotState.robot_name}
                    </Typography>
                  </Grid>
                  <Grid item xs={12}>
                    <Typography variant="subtitle2" color="textSecondary" className={classes.text}>
                      {robotState.fleet_name}
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Battery
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>
                    <Progress percent={parseInt(robotState.battery_percent)} size="small" width={50}/>
                  </Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Task ID
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>{robotState.assignments}</Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Status
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>{robotState.mode}</Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Zone
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>2</Grid>
                  <Grid item xs={6}>
                    <Typography variant="subtitle2" color="textSecondary">
                      Location
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>{robotState.level_name}</Grid>
              </Grid>
            </CardContent>
        </Card>
    );                                                                
}
