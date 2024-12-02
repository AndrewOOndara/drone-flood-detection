import React, { useState } from 'react';
import './index.css';
import EditCoverageMap from './EditCoverageMap';
import { AppSettings, DEFAULT_SETTINGS, Drone } from '../types';
import { getSettings } from '../api';


function Admin() {
    const [settings, setSettings] = useState<AppSettings>(DEFAULT_SETTINGS);
    React.useEffect(() => {
        getSettings().then(s => {
            setSettings(s);
        });
    }, []);
    const updateSettings = React.useCallback((ps: Partial<AppSettings>) => {
        setSettings(s => ({ ...s, ...ps }));
    }, [setSettings])
    return <React.Fragment>
        <div className="admin-sections-container">
            <Section name="Coverage">
                <EditCoverageMap
                    edges_to_visit={settings.edges_to_visit}
                    map_lat_lon={settings.map_lat_lon}
                    map_radius={settings.map_radius}
                    updateSettings={updateSettings}
                />
            </Section>
            <Section name="Drones">
                <DronesList drones={settings.drones} updateSettings={updateSettings} />
            </Section>
        </div>
    </React.Fragment>
}

function Section(props: { name: string, children: React.ReactNode }) {
    return (
        <div className="admin-section shadowed">
            <h2 className="admin-section-header">{props.name}</h2>
            {props.children}
        </div>
    )
}

function DronesList({ drones, updateSettings }: Pick<AppSettings, "drones"> & { updateSettings: (s: Partial<AppSettings>) => void }) {
    const updateDrone = React.useCallback((drone_id: number, changes: Partial<Drone>) => {
        updateSettings({
            drones: drones.map(d => (
                d.drone_id == drone_id ? { ...d, ...changes } : d
            ))
        })
    }, [updateSettings]);
    return (
        <ol className="admin-drones-list">
            {drones.map(d => (
                <DroneItem
                    key={d.drone_id}
                    updateDrone={updateDrone}
                    {...d}
                />
            ))}
        </ol>
    )
}

const DroneItem = React.memo(function (props: Drone & { updateDrone: (drone_id: number, d: Partial<Drone>) => void }) {
    return <div className="admin-drone-box">
        <span>ID: {props.drone_id}</span>
    </div>
})

export default Admin;