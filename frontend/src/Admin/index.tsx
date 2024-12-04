import React, { useState } from 'react';
import './index.css';
import { AppSettings, DEFAULT_SETTINGS } from '../types';
import { apiGetSettings, saveSettings } from '../api';
import MapComponent from './MapComponent';
import { useCoverage } from './coverage';
import { useDrone } from './drone';
import { usePlan } from './plan';

type UpdateSettings = React.Dispatch<React.SetStateAction<AppSettings>>;

const Admin = React.memo(function (props: {}) {
    const [settings, setSettings] = useState<AppSettings>(DEFAULT_SETTINGS);
    const requestRunning = React.useRef(false);
    React.useEffect(() => {
        if (requestRunning.current) return;
        requestRunning.current = true;
        apiGetSettings().then(s => {
            console.debug("Got settings");
            console.debug(s);
            setSettings(s);
        }).finally(() => {
            requestRunning.current = false;
        });
    }, [setSettings]);
    const save = React.useCallback(() => {
        if (requestRunning.current) return;
        requestRunning.current = true;
        saveSettings(settings).finally(() => {
            requestRunning.current = false;
        });
    }, [settings]);

    const [covController, covUi] = useCoverage(settings, setSettings);
    const [droController, droUi] = useDrone(settings, setSettings);
    const [plaController, plaUi] = usePlan(settings);

    return <>
        <div className="app-title-row">
            <h1 className="app-title">HPE Admin Controls</h1>
            {plaUi}
            <button onClick={save}>💾</button>
        </div>
        <div className="admin-container">
            <section className="admin-map-container">
                <MapComponent
                    center={settings.map_lat_lon}
                    radius={settings.map_radius}
                >
                    {covController}
                    {droController}
                    {plaController}
                </MapComponent>
            </section>
            <div className="admin-sections-container">
                <Section name="Coverage">
                    {covUi}
                </Section>
                <Section name="Drones">
                    {droUi}
                </Section>
            </div>
        </div>
    </>
});

function Section(props: { name: string, children: React.ReactNode }) {
    return (
        <div className="admin-section">
            <h2 className="admin-section-header">{props.name}</h2>
            {props.children}
        </div>
    )
}

export default Admin;