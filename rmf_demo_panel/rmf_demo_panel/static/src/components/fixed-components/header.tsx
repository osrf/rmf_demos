import * as React from "react";
import { AppBar, Typography, Toolbar } from '@material-ui/core';

const Header = () : React.ReactElement => {

    return (
        <div>
            <AppBar id="appbar" position="sticky">
                <Toolbar>
                    <Typography variant="h4">RMF Panel</Typography>
                </Toolbar>
            </AppBar>
        </div>
    )
}

export default Header;