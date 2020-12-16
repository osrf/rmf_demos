import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from '../styles';

interface DeliveryFormProps {
  deliveryOptions: {}
}

const DeliveryForm = (props: DeliveryFormProps): React.ReactElement => {
  const { deliveryOptions } = props;
  const classes = useFormStyles();
  const [deliveryTask, setDeliveryTask] = React.useState("");
  const [deliveryOptionKeys, setDeliveryOptionKeys] = React.useState([]);
  const [minsFromNow, setMinsFromNow] = React.useState(0);
  const [errorMessage, setErrorMessage] = React.useState("");

  React.useEffect(() => {
    let optionKeys = [];
    for (const key in deliveryOptions) {
        optionKeys.push(key);
    }
    setDeliveryOptionKeys(optionKeys);
  }, []);

  const isFormValid = () => {
    if(deliveryTask === "") {
      setErrorMessage("Please select a delivery task");
      return false;
    }
    return true;
  }
  
  const submitDeliveryRequest = () => {
    //TODO: submission of request
    console.log("delivery request submitted");
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    //TOOD: submit loop task (make use of romi-js-core-interface lib?)
    ev.preventDefault();
    if(isFormValid()) {
      submitDeliveryRequest();
      setDeliveryTask("");
      setErrorMessage("");
    }
  }

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
          renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select delivery task" variant="outlined" margin="normal" />}
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
  )
}

export default DeliveryForm;
