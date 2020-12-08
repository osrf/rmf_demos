import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { useFormStyles } from "../styles";

interface Task {
  task_type: string,
  start_time: number,
  description: string
}

const ScheduledTaskForm = (): React.ReactElement => {
  const classes = useFormStyles();
  const [taskList, setTaskList] = React.useState<string | ArrayBuffer>();
  const [message, setMessage] = React.useState<string>('');
  const placeholder = `eg. [
{"task_type":"Clean", "start_time":0, "description":{"cleaning_zone":"zone_1"}},
{"task_type":"Clean", "start_time":10, "description":{"cleaning_zone":"zone_2"}},
{"task_type":"Clean", "start_time":5, "description":{"cleaning_zone":"zone_3"}}
]`

  let global_list_count = 0;
  let global_task_list: Array<Task> = [];

  const submitTaskList = () => {
    global_list_count = 0;
    global_task_list = [];
    let i = 0; // to set time "delay"
    let res = "Task List Submitted";
    let tempTaskList: any = taskList; //best to remove 'any'
    let jsonTaskList: Array<Task> = JSON.parse(tempTaskList);

    //simulate submission of tasks at intervals
    jsonTaskList.forEach((task) => {
      global_task_list.push(task);
      setTimeout(function(i) {
        console.log("Submit Task", global_task_list[global_list_count]);
        try {
          fetch('/submit_task', {
            method: "POST",
            body: JSON.stringify(global_task_list[global_list_count]),
            headers: { 
                "Content-type": "application/json; charset=UTF-8"
            } 
          })
          .then(res => res.json())
          .then(data => JSON.stringify(data));

          global_list_count++;
        } catch (err) {
          res = "ERROR! " + err;
          console.log('Unable to submit task request');
        }
      }, 900*(++i));
      setMessage(res);
    });
  }

  const readTaskFile = (e: React.ChangeEvent<HTMLInputElement>) => {
    let file = e.target.files[0];
    const reader = new FileReader();
    reader.onload = function(event) {
      let result = event.target.result;
      setTaskList(result);
    };
    reader.readAsText(file);
  }

  return (
      <Box className={classes.form}>
        <div className={classes.divForm}>
          <Typography variant="h6">Scheduled Task List</Typography>
          </div>
        <div className={classes.buttonContainer}>
          <Button variant="contained" color="primary" className={classes.button}>
            <input type="file" id="task_file" name="task_file" onChange={readTaskFile} />
          </Button>
        </div>
        <div className={classes.divForm}>
          <TextField
                id="task_list_box"
                multiline
                rows={5}
                placeholder={placeholder}
                variant="outlined"
                fullWidth
                defaultValue={taskList}
                onChange={(e) => setTaskList(e.target.value)}
              />
        </div>
        <div className={classes.buttonContainer}>
          <Button variant="contained" color="primary" onClick={submitTaskList} className={classes.button}>Submit Task List</Button>
        </div>
        <div className={classes.divForm}>
          <Typography variant="h6">{message}</Typography>
        </div>
    </Box>
  )
}

export default ScheduledTaskForm;