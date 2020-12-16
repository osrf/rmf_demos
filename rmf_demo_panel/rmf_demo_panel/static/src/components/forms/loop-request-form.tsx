import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from '../styles';

interface LoopFormProps {
  availablePlaces: string[]
}

const LoopRequestForm = (props: LoopFormProps): React.ReactElement => {
  const { availablePlaces } = props;
  const [startLocation, setStartLocation] = React.useState("");
  const [endLocation, setEndLocation] = React.useState("");
  const [places, setPlaces] = React.useState(availablePlaces);
  const [numLoops, setNumLoops] = React.useState(1);
  const [minsFromNow, setMinsFromNow] = React.useState(0);
  const [errorMessage, setErrorMessage] = React.useState("");

  const classes = useFormStyles();
  const isFormValid = () => {
    //TODO: check that form inputs are valid before submitting
    if(startLocation == endLocation) {
      setErrorMessage("Start and end locations cannot be the same");
      return false;
    } else if(numLoops <= 0) {
      setErrorMessage("Number of loops can only be > 0")
      return false;
    }
    return true;
  }

  const clearForm = () => {
    setStartLocation("");
    setEndLocation("");
    setNumLoops(1);
    setErrorMessage('');
  }

  const submitLoopRequest = () => {
    //TODO: submission of request
    console.log("loop request submitted");
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    //TOOD: submit loop task (make use of romi-js-core-interface lib?)
    ev.preventDefault();
    if(isFormValid()) {
      submitLoopRequest();
      clearForm();
    }
  }

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
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select start location" variant="outlined" margin="normal" />}
                />
            </div>
            <div className={classes.divForm}>
                <Autocomplete id="set-end-location"
                openOnFocus
                options={places}
                getOptionLabel={(place) => place}
                onChange={(_, value) => setEndLocation(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select end location" variant="outlined" margin="normal" />}
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
                value={numLoops || 0}
                label="Number of Loops"
                variant="outlined"
                id="set-num-loops"
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
                <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Request</Button>
            </div>
            <Typography variant="h6">{errorMessage}</Typography>
        </Box>
    );
}

export default LoopRequestForm;
