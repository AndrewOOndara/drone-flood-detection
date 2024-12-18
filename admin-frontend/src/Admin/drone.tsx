import React from "react";
import { AppSettings, Drone } from "../types";
import { CircleMarker, useMapEvents } from "react-leaflet";
import { getNodeFromCoords } from "../api";
type UpdateSettings = React.Dispatch<React.SetStateAction<AppSettings>>;

export function useDrone(settings: AppSettings, updateSettings: UpdateSettings): [React.ReactNode, React.ReactNode] {
    const [picking, setPicking] = React.useState<number | null>(null);
    let ui = null;
    let controller = null;
    controller = <DroneLocationsMarker drones={settings.drones} picking={picking} setPicking={setPicking} updateSettings={updateSettings} />
    ui = <DronesList drones={settings.drones} map_lat_lon={settings.map_lat_lon} updateSettings={updateSettings} picking={picking} setPicking={setPicking} />
    return [controller, ui];
}

type DronePickerProps = {
    picking: number | null,
    setPicking: (droneId: number | null) => void,
    updateSettings: UpdateSettings
}

const DroneLocationsMarker = React.memo(function (props: { drones: AppSettings["drones"] } & DronePickerProps) {
    return <>
        {props.drones.map(d => (
            <CircleMarker
                key={d.drone_id}
                center={d.base_station_node.position}
                color='violet'
                radius={10}
            />
        ))}
        {props.picking !== null && (
            <DroneLocationClickController picking={props.picking} setPicking={props.setPicking} updateSettings={props.updateSettings} />
        )}
    </>
});
const DroneLocationClickController = React.memo(function (props: DronePickerProps) {
    useMapEvents({
        click: async (e: L.LeafletMouseEvent) => {
            const node = await getNodeFromCoords([e.latlng.lat, e.latlng.lng]);
            props.setPicking(null);
            props.updateSettings(s => ({
                ...s,
                drones: s.drones.map(d => ((d.drone_id === props.picking) ? {
                    ...d,
                    base_station_node: node
                } : d))
            }));
        }
    })
    return null;
})

const DronesList = React.memo(function ({ drones, map_lat_lon, updateSettings, picking, setPicking }: Pick<AppSettings, "drones" | "map_lat_lon"> & DronePickerProps) {
    const addDrone = React.useCallback(async () => {
        const base_station_node = await getNodeFromCoords(map_lat_lon);
        updateSettings(s => {
            return {
                ...s,
                drones: [...s.drones, {
                    drone_id: (s.drones.length === 0) ? 1 : (Math.max(...s.drones.map(d => d.drone_id)) + 1),
                    base_station_node,
                    battery_life_m: 2500,
                }]
            }
        })
    }, [map_lat_lon, updateSettings])
    return (
        <>
            <ol className="admin-drones-list">
                {drones.map(d => (
                    <DroneItem
                        key={d.drone_id}
                        updateSettings={updateSettings}
                        setPicking={setPicking}
                        beingPicked={picking === d.drone_id}
                        {...d}
                    />
                ))}
            </ol>
            <div className="flex-horizontal flex-spread">
                <button onClick={addDrone}>Add a new drone</button>
            </div>
        </>
    )
})

const DroneItem = React.memo(function (props: Drone & { beingPicked: boolean } & Pick<DronePickerProps, "setPicking" | "updateSettings">) {
    return <div className="admin-drone-box">
        <span>ID: {props.drone_id}</span>
        <label className="flex-horizontal">
            Range (m):
            <input className="flex-one" type="number" value={props.battery_life_m} onChange={(ev) => {
                const number = parseFloat(ev.target.value);
                props.updateSettings(s => ({
                    ...s,
                    drones: s.drones.map(d => (d.drone_id === props.drone_id) ? ({
                        ...d,
                        battery_life_m: number
                    }) : d)
                }))
            }} />
        </label>
        {props.beingPicked ?
            <>
                <span>Click on the map to set base station.</span>
                <button onClick={() => { props.setPicking(null) }}>Cancel</button>
            </>
            :
            <div className="flex-horizontal">
                <button className="flex-one" onClick={() => { props.setPicking(props.drone_id) }}>Change Location</button>
                <button onClick={() => {
                    props.updateSettings(s => ({
                        ...s,
                        drones: s.drones.filter(d => d.drone_id !== props.drone_id)
                    }))
                }}>🚮</button>
            </div>
        }
    </div>
})

