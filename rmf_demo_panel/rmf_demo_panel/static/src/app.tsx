import * as React from 'react';
import { Divider } from '@material-ui/core';
import Header from "./components/fixed-components/header";
import PanelsContainer from './components/panels-container';
import Footer from './components/fixed-components/footer';
import NavTabs from './components/fixed-components/tabs';
import { WorldContext, World } from './components/fixed-components/app-context';

export default function App(): React.ReactElement {
    const currWorld = React.useContext(WorldContext);
    const [currentWorld, setCurrentWorld] = React.useState(currWorld);
    console.log("The current map is the", World[currentWorld.currentWorld]);

    return (
        <div>
            <WorldContext.Provider value={currentWorld}>
                <Header />
                <NavTabs handleWorldChange={(world) => setCurrentWorld(world)}/>
                <Divider variant="middle" />
                <PanelsContainer />
                <Divider variant="middle"/>
                <Footer />
            </WorldContext.Provider>
        </div>
    )
}
