import * as React from 'react';
import { Box, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from "../styles";
import CleaningForm from './cleaning-form';
import LoopRequestForm from './loop-request-form';
import DeliveryForm from './delivery-form';
import { WorldContext } from '../fixed-components/app-context';

const RequestForm = (): React.ReactElement => {
    const { config } = React.useContext(WorldContext);
    console.log("req form config", config);
    const [requestTypes, setRequestTypes] = React.useState(config.valid_task);
    const [formType, setFormType] = React.useState('');
    const [loopPlaces, setLoopPlaces] = React.useState([]);
    const [deliveryOptions, setDeliveryOptions] = React.useState({});
    const [cleaningZones, setCleaningZones] = React.useState([]);
    
    React.useEffect(() => {
        if(Object.keys(config).length > 0) {
            setRequestTypes(config.valid_task);
            setLoopPlaces(config.task.Loop.places);
            setDeliveryOptions(config.task.Delivery.option);
            setCleaningZones(config.task.Clean.option);
        } else {
            setRequestTypes([]);
            setFormType('');
            setLoopPlaces([]);
            setDeliveryOptions({});
            setCleaningZones([]);
        }
    }, [config]);

    const returnFormType = (formType: string) => {
        switch (formType) {
            case "Loop":
                return <LoopRequestForm availablePlaces={loopPlaces} />
            case "Delivery": 
                return <DeliveryForm deliveryOptions={deliveryOptions} />
            case "Clean":
                return <CleaningForm cleaningZones={cleaningZones}/>
        }
    }
  
    const classes = useFormStyles();
    
    return (
        <Box className={classes.form}>
            <div className={classes.divForm}>
            <Typography variant="h6">Requests</Typography>
                <Autocomplete
                options={requestTypes}
                getOptionLabel={(requestType) => requestType}
                id="set-form-type"
                openOnFocus
                defaultValue={formType}
                onChange={(_, value) => setFormType(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select a request type" variant="outlined" margin="normal" />}
                />
            </div>
            <div className={classes.divForm}>
                {formType && returnFormType(formType)}
            </div>
        </Box>
    )
}

export default RequestForm;
