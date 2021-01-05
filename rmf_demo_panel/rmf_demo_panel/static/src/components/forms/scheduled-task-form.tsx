import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { showErrorMessage } from "../fixed-components/messages";
import { useFormStyles } from "../styles";

interface ScheduledTaskFormProps {
  submitTaskList: (taskList: string | ArrayBuffer) => void;
}

const ScheduledTaskForm = (props: ScheduledTaskFormProps): React.ReactElement => {
  const { submitTaskList } = props;
  const classes = useFormStyles();
  const [taskList, setTaskList] = React.useState<string | ArrayBuffer>('');
  const placeholder = `eg. [
{"task_type":"Clean", "start_time":0, "description":{"cleaning_zone":"zone_1"}},
{"task_type":"Clean", "start_time":10, "description":{"cleaning_zone":"zone_2"}},
{"task_type":"Clean", "start_time":5, "description":{"cleaning_zone":"zone_3"}}
]`

  const isFormValid = () => {
    if(taskList === "" || taskList === `[]` || taskList === `{}`) {
      showErrorMessage("Unable to submit an empty task list");
      return false;
    }
    return true;
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      submitTaskList(taskList);
      setTaskList('');
    }
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
      <Box className={classes.form} role="scheduled-task-form">
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
                multiline
                rows={5}
                placeholder={placeholder}
                variant="outlined"
                fullWidth
                value={taskList || ''}
                onChange={(e) => setTaskList(e.target.value)}
              />
        </div>
        <div className={classes.buttonContainer}>
          <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Task List</Button>
        </div>
    </Box>
  )
}

export default ScheduledTaskForm;