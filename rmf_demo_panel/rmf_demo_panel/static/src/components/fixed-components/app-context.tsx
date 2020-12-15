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
    map: World,
    config: any
}

export const WorldContext = React.createContext<WorldContextType>({ map: World.Office, config: {} });
