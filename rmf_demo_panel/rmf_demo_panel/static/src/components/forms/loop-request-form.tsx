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
  const [numLoops, setNumLoops] = React.useState(0);

  const classes = useFormStyles();
  const validateForm = () => {
    //TODO: check that form inputs are valid before submitting
  }

  const submitLoopRequest = () => {
    //TOOD: submit loop task (make use of romi-js-core-interface lib?)
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
            <div className={classes.buttonContainer}>
                <Button variant="contained" color="primary" onClick={submitLoopRequest} className={classes.button}>Submit Request</Button>
            </div>
        </Box>
    );
}

export default LoopRequestForm;
