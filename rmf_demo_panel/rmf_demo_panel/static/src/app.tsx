import * as React from 'react';
import { Divider } from '@material-ui/core';
import Header from "./components/fixed-components/header";
import PanelsContainer from './components/panels-container';
import Footer from './components/fixed-components/footer';
import NavTabs from './components/fixed-components/tabs';

export default function App(): React.ReactElement {
    return (
        <div>
            <Header />
            <NavTabs />
            <Divider variant="middle" />
            <PanelsContainer />
            <Divider variant="middle"/>
            <Footer />
        </div>
    )
}  