import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from "../styles";
import { showErrorMessage, showSuccessMessage } from "../fixed-components/messages";

interface CleaningFormProps {
  cleaningZones: string[]
}

export const CleaningForm = (props: CleaningFormProps): React.ReactElement => {
  const { cleaningZones } = props;
  const [allZones, setZones] = React.useState(cleaningZones);
  const [targetZone, setTargetZone] = React.useState('');
  const [evaluator, setEvaluator] = React.useState('');
  const [minsFromNow, setMinsFromNow] = React.useState(0);

  //errors
  const [timeError, setTimeError] = React.useState("");
  const [zoneError, setZoneError] = React.useState("");
  
  const classes = useFormStyles();
  const evaluators: string[] = ["lowest_delta_cost", "lowest_cost", "quickest_time"];
  
  React.useEffect(() => {
    setZones(cleaningZones);
  }, [cleaningZones]);

  const cleanUpForm = () => {
    setTargetZone('');
    setEvaluator('');
    setMinsFromNow(0);
  }

  const isFormValid = () => {
    if(minsFromNow < 0) {
      setTimeError("Start time cannot be negative");
      return false;
    }
    if(targetZone.length === 0) {
      setZoneError("Cleaning zone cannot be an empty field");
      return false;
    }
    return true;
  }

  const createRequest = () => {
    let start_time = minsFromNow;
    let cleaning_zone = targetZone;
    let request = {};
    if (evaluator.length > 0 ){
      let evaluator_option = evaluator;
      request = { task_type: "Clean",
                  start_time: start_time,
                  evaluator: evaluator_option,
                  description: {'cleaning_zone': cleaning_zone} }
    } else {
      request = { task_type: "Clean",
                  start_time: start_time,
                  description: {'cleaning_zone': cleaning_zone} }
      }
    return request;
  }
  
  const submitCleaningRequest = () => {
      const request = createRequest();
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
        showSuccessMessage("Cleaning Request submitted successfully!");
      } catch (err) {
        console.log(err);
        showErrorMessage("Unable to submit cleaning request");
      }
      cleanUpForm();
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      submitCleaningRequest();
    }
  }

    return (
        <Box className={classes.form} role="cleaning-form">
            <div className={classes.divForm}>
            <Typography variant="h6">Schedule an Ad-Hoc Task</Typography>
                <Autocomplete
                options={allZones}
                getOptionLabel={(zone) => zone}
                id="set-cleaning-zone"
                openOnFocus
                onChange={(_, value) => setTargetZone(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Pick a zone" variant="outlined" margin="normal" helperText={zoneError} error={!!zoneError}/>}
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
                helperText={timeError}
                error={!!timeError}
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
                <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Task</Button>
            </div>
        </Box>
    );
} 

export default CleaningForm;