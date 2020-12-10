import * as React from "react";
import { Box, Button, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from '../styles';

interface DeliveryFormProps {
  deliveryOptions: {
    coke: {
      pickup_place_name: string,
      pickup_dispenser: string,
      dropoff_place_name: string,
      dropoff_ingestor: string
    },
    mop: {
      pickup_place_name: string,
      pickup_dispenser: string,
      dropoff_place_name: string,
      dropoff_ingestor: string
    }
  }
}

const DeliveryForm = (props: DeliveryFormProps): React.ReactElement => {
  const { deliveryOptions } = props;
  const classes = useFormStyles();
  const [deliveryTask, setDeliveryTask] = React.useState('');
  const [deliveryOptionKeys, setDeliveryOptionKeys] = React.useState([]);

  React.useEffect(() => {
    let optionKeys = [];
    for (const key in deliveryOptions) {
        optionKeys.push(key);
    }
    setDeliveryOptionKeys(optionKeys);
  }, []);

  const submitDeliveryRequest = () => { 
    //TODO: implement submission of deliveries
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
          renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select start location" variant="outlined" margin="normal" />}
        />
      </div>
      <div className={classes.buttonContainer}>
        <Button variant="contained" color="primary" onClick={submitDeliveryRequest} className={classes.button}>Submit Request</Button>
      </div>
    </Box>
  )
}

export default DeliveryForm;
