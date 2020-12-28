import React from 'react';
import { Box, Typography } from '@material-ui/core';
import { socket } from '../socket';

const RostimeClock = () => {
    const [time, setTime] = React.useState('');

    React.useEffect(() => {
        socket.on("ros_time", msg => {
            const timeData = Math.floor(msg / 3600) + ":" +
            (Math.floor(msg / 60) % 60) + ":" +
            (msg % 60);
            setTime(timeData);
        });
    });

    return (
        <Box>
            <Typography variant='overline' align="center" gutterBottom>Time is: {time}</Typography>
        </Box>
    )
}

export default RostimeClock;
