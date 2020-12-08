import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from "../styles";

//TODO: implement form validation & getting task status upon submission

export const CleaningForm = (): React.ReactElement => {
  const [zone, setZone] = React.useState('');
  const [evaluator, setEvaluator] = React.useState('');
  const [minsFromNow, setMinsFromNow] = React.useState(0);
  
  const classes = useFormStyles();
  
  const zones: string[] = ["zone_1", "zone_2", "zone_3", "zone_4"];
  const evaluators: string[] = ["lowest_delta_cost", "lowest_cost", "shortest_time"];

  const cleanUpForm = () => {
    setZone('');
    setEvaluator('');
    setMinsFromNow(0);
  }
  
  const submitTaskForm = () => {
      let start_time = minsFromNow;
      let cleaning_zone = zone;
      let evaluator_option = evaluator;
      console.log("submit task: ", start_time, cleaning_zone, evaluator_option );
      console.log("Submitting Task");
      try {
        fetch('/submit_task', {
        method: "POST",
        body: JSON.stringify({
                task_type: 'Clean', 
                start_time: start_time,
                evaluator: evaluator_option,
                description: {'cleaning_zone': cleaning_zone}
              }),
        headers: { 
            "Content-type": "application/json; charset=UTF-8"
        } 
      })
        .then(res => res.json())
        .then(data => JSON.stringify(data));
        
      } catch (err) {
        console.log('Unable to submit task request');
      }
      cleanUpForm();
  }

    return (
        <Box className={classes.form}>
            <div className={classes.divForm}>
            <Typography variant="h6">Schedule an Ad-Hoc Task</Typography>
                <Autocomplete
                options={zones}
                getOptionLabel={(zone) => zone}
                id="set-cleaning-zone"
                openOnFocus
                onChange={(_, value) => setZone(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Pick a zone" variant="outlined" margin="normal" />}
                />
            </div>
            <div className={classes.divForm}>
                <Autocomplete id="set-evaluator"
                openOnFocus
                options={evaluators}
                getOptionLabel={(evaluator) => evaluator}
                onChange={(_, value) => setEvaluator(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Choose an evaluator" variant="outlined" margin="normal" />}
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
                />
            </div>
            <div className={classes.buttonContainer}>
                <Button variant="contained" color="primary" onClick={submitTaskForm} className={classes.button}>Submit Task</Button>
            </div>
        </Box>
    );
} 

export default CleaningForm;