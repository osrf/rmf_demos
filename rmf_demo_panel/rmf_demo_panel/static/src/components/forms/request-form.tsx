import * as React from 'react';
import { Box, TextField, Typography } from "@material-ui/core";
import { Autocomplete, AutocompleteRenderInputParams } from '@material-ui/lab';
import { useFormStyles } from "../styles";
import CleaningForm from './cleaning-form';
import LoopRequestForm from './loop-request-form';
import DeliveryForm from './delivery-form';
import dashboardConfig from '../../../dashboard_config';

const RequestForm = (): React.ReactElement => {
    const [formType, setFormType] = React.useState('loop request');
    const loopPlaces = dashboardConfig.task.Loop.places;

    const returnFormType = (formType: string) => {
        switch (formType) {
            case "loop request":
                return <LoopRequestForm availablePlaces={loopPlaces}/>
            case "delivery": 
                return <DeliveryForm />
            case "cleaning":
                return <CleaningForm />
        }
    }
  
    const classes = useFormStyles();
    
    const requestTypes: string[] = ["loop request", "delivery", "cleaning"];

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
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Pick a zone" variant="outlined" margin="normal" />}
                />
            </div>
            <div className={classes.divForm}>
                {formType && returnFormType(formType)}
            </div>
        </Box>
    )
}

export default RequestForm;