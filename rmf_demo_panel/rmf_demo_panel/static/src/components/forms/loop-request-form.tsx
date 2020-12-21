import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from '../styles';

interface LoopDescription {
  num_loops: number,
  start_name: string,
  finish_name: string
}

interface LoopFormProps {
  availablePlaces: string[]
}

const LoopRequestForm = (props: LoopFormProps): React.ReactElement => {
  const { availablePlaces } = props;
  const classes = useFormStyles();
  const [startLocation, setStartLocation] = React.useState("");
  const [endLocation, setEndLocation] = React.useState("");
  const [places, setPlaces] = React.useState(availablePlaces);
  const [numLoops, setNumLoops] = React.useState(1);
  const [minsFromNow, setMinsFromNow] = React.useState(0);
  const [evaluator, setEvaluator] = React.useState('');

  //errors
  const [timeError, setTimeError] = React.useState("");
  const [numLoopsError, setNumLoopsError] = React.useState("");
  const [locationError, setLocationError] = React.useState("");

  React.useLayoutEffect(() => {
    setPlaces(availablePlaces);
  }, [availablePlaces]);

  const isFormValid = (): boolean => {
    let isValid = true;
    if(startLocation == endLocation) {
      setLocationError("Start and end locations cannot be the same");
      isValid = false;
    }
    if(startLocation == '' || endLocation == '') {
      setLocationError('Please select a location');
      isValid = false;
    }
    if(numLoops <= 0) {
      setNumLoopsError("Number of loops can only be > 0");
      isValid = false;
    }
    if(minsFromNow < 0) {
      setTimeError("Start time can only be >= 0");
      isValid = false;
    }
    return isValid;
  }

  const cleanUpForm = () => {
    setStartLocation("");
    setEndLocation("");
    setNumLoops(1);
    setMinsFromNow(0);
    cleanUpError();
  }

  const cleanUpError = () => {
      setLocationError('');
      setNumLoopsError('');
      setTimeError('');
    };

  const submitLoopRequest = () => {
    let description: LoopDescription = {
      num_loops: numLoops,
      start_name: startLocation,
      finish_name: endLocation,
    }
    let start_time = minsFromNow;
    let request = {};
    if (evaluator.length > 0 ){
        let evaluator_option = evaluator;
        request = { task_type: "Loop",
                    start_time: start_time,
                    evaluator: evaluator_option,
                    description: description }
      } else {
        request = { task_type: "Loop",
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
        alert("Unable to submit loop request");
        console.log('Unable to submit loop request');
      }
      cleanUpForm();
      console.log("loop request submitted");
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      submitLoopRequest();
    }
  }

  const evaluators: string[] = ["lowest_delta_cost", "lowest_cost", "quickest_time"];

  return (
        <Box className={classes.form}>
            <div className={classes.divForm}>
            <Typography variant="h6">Schedule a Loop Request</Typography>
                <Autocomplete
                options={places}
                getOptionLabel={(place) => place}
                id="set-start-location"
                openOnFocus
                onChange={(_, value) => setStartLocation(value)}
                renderInput={(params: AutocompleteRenderInputParams) => 
                  <TextField {...params} 
                    label="Select start location" 
                    variant="outlined" 
                    margin="normal"
                    error={!!locationError}
                    helperText={locationError} 
                  />}
                value={startLocation ? startLocation : null}
                />
            </div>
            <div className={classes.divForm}>
                <Autocomplete id="set-end-location"
                openOnFocus
                options={places}
                getOptionLabel={(place) => place}
                onChange={(_, value) => setEndLocation(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select end location" variant="outlined" margin="normal" error={!!locationError} helperText={locationError}/>}
                value={endLocation ? endLocation : null}
                />
            </div>
            <div className={classes.divForm}>
                <TextField
                  className={classes.input}
                  onChange={(e) => {
                  setNumLoops(e.target.value ? parseInt(e.target.value) : 0);
                  }}
                  placeholder="Set number of loops"
                  type="number"
                  value={numLoops || ''}
                  label="Number of Loops"
                  variant="outlined"
                  id="set-num-loops"
                  error={!!numLoopsError}
                  helperText={numLoopsError}
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
    );
}

export default LoopRequestForm;
