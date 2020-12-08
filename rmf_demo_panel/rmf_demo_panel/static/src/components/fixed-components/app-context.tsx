import * as React from 'react';

export const Worlds = {
    0: "Office",
    1: "Airport",
    2: "Clinic"
}

export enum World {
    Office = 0,
    Airport,
    Clinic,
}

export type WorldContextType = {
    currentWorld: World;
    setWorld: React.Dispatch<React.SetStateAction<World>>
}

export const WorldContext = React.createContext<WorldContextType>({ currentWorld: World.Office, setWorld: newWorld => console.warn('no world provider')});
