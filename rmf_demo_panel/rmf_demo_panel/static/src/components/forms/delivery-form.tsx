import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from '../styles';

//declare without any
interface DeliveryFormProps {
  deliveryOptions: any
}

const DeliveryForm = (props: DeliveryFormProps): React.ReactElement => {
  const { deliveryOptions } = props;
  const classes = useFormStyles();
  const [deliveryTask, setDeliveryTask] = React.useState("");
  const [deliveryOption, setDeliveryOption] = React.useState({});
  const [deliveryOptionKeys, setDeliveryOptionKeys] = React.useState([]);
  const [minsFromNow, setMinsFromNow] = React.useState(0);
  const [evaluator, setEvaluator] = React.useState('');

  //errors
  const [timeError, setTimeError] = React.useState("");
  const [taskError, setTaskError] = React.useState("");

  React.useEffect(() => {
    let optionKeys = [];
    for (const key in deliveryOptions) {
        optionKeys.push(key);
    }
    setDeliveryOptionKeys(optionKeys);
  }, [deliveryOptions]);

  React.useEffect(() => {
    {deliveryTask.length > 0 && 
      setDeliveryOption({ option: deliveryTask }) }
  }, [deliveryTask]);

  const isFormValid = () => {
    if(deliveryTask === "") {
      setTaskError("Please select a delivery task");
      return false;
    }
    if(timeError) {
      setTimeError("Start time cannot be negative");
      return false;
    }
    return true;
  }

  const cleanUpForm = () => {
    setDeliveryOption({});
    setDeliveryTask("");
    setTaskError("");
    setTimeError("");
  }
  
  const submitDeliveryRequest = () => {
      let start_time = minsFromNow;
      let description = deliveryOption;
      let request = {};
      if (evaluator.length > 0 ){
        let evaluator_option = evaluator;
        request = { task_type: "Delivery",
                    start_time: start_time,
                    evaluator: evaluator_option,
                    description: description }
      } else {
        request = { task_type: "Delivery",
                    start_time: start_time,
                    description: description }
      }
      console.log("submit task: ", start_time, description);
      console.log("Submitting Task");
      try {
        fetch('/submit_task', {
        method: "POST",
        body: JSON.stringify(request),
        headers: { 
            "Content-type": "application/json; charset=UTF-8"
        } 
      })
        .then(res => res.json())
        .then(data => JSON.stringify(data));
      } catch (err) {
        alert("Unable to submit delivery request");
        console.log('Unable to submit delivery request');
      }
      cleanUpForm();
      console.log("Delivery request submitted");
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      submitDeliveryRequest();
    }
  }

  const evaluators: string[] = ["lowest_delta_cost", "lowest_cost", "quickest_time"];

  return (
    <Box className={classes.form}>
      <div className={classes.divForm}>
        <Typography variant="h6">Schedule a Delivery Request</Typography>
        <Autocomplete
          options={deliveryOptionKeys}
          getOptionLabel={(option: any) => option}
          id="set-delivery-task"
          openOnFocus
          onChange={(_, value) => setDeliveryTask(value)}
          renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select delivery task" variant="outlined" margin="normal" error={!!taskError} helperText={taskError} />}
          value={deliveryTask ? deliveryTask : null}
        />
      </div>
      <div className={classes.divForm}>
        <TextField
          className={classes.input}
          onChange={(e) => {
          setMinsFromNow(e.target.value ? parseInt(e.target.value) : 0);
          }}
          placeholder="Set start time (mins from now)"
          type="number"
          value={minsFromNow || 0}
          label="Set start time (mins from now)"
          variant="outlined"
          id="set-start-time"
          error={!!timeError}
          helperText={timeError}
        />
      </div>
      <div className={classes.divForm}>
        <Autocomplete id="set-evaluator"
          openOnFocus
          options={evaluators}
          getOptionLabel={(evaluator) => evaluator}
          onChange={(_, value) => setEvaluator(value)}
          renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Choose an evaluator (optional)" variant="outlined" margin="normal" />}
        />
      </div>
      <div className={classes.buttonContainer}>
        <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Request</Button>
      </div>
    </Box>
  )
}

export default DeliveryForm;
