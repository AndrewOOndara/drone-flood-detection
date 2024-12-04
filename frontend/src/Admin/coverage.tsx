import { CircleMarker, Polyline, Rectangle, useMap, useMapEvents } from "react-leaflet";
import { AppSettings, EdgeInfo, LatLon } from "../types";
import React, { useState } from "react";
import { getEdgeFromCoords } from "../api";
type UpdateSettings = React.Dispatch<React.SetStateAction<AppSettings>>;
type CoverageControllerProps = {
    state: CoverageState,
    setState: React.Dispatch<React.SetStateAction<CoverageState>>,
    updateSettings: UpdateSettings
}
const AddedEdges = React.memo(function ({ edges }: { edges: EdgeInfo[] }) {
    return <>
        {edges.map(edge => (
            <EdgeLine key={`${edge.fromNodeId}:${edge.toNodeId}`} edge={edge} />
        ))}
    </>
})
const EdgeLine = React.memo(function ({ edge }: { edge: EdgeInfo }) {
    return <Polyline
        positions={edge.positions}
        color="purple"
        weight={3}
        opacity={0.7}
    />
})

type CoverageState = {
    mode: "nothing",
} | {
    mode: "adding",
    points: LatLon[]
} | {
    mode: "removing"
}
export function useCoverage(settings: AppSettings, updateSettings: UpdateSettings): [React.ReactNode, React.ReactNode] {
    const [state, setState] = React.useState<CoverageState>({ mode: "nothing" });
    let controller = null;
    let ui = null;
    if (state.mode === "nothing") {
        ui = <div className="flex-horizontal flex-spread">
            <button onClick={() => setState({ mode: "adding", points: [] })}>Add Coverage</button>
            <button onClick={() => setState({ mode: "removing" })}>Delete Coverage</button>
        </div>;
    }
    if (state.mode === "adding") {
        controller = <AddCoverageController state={state} setState={setState} updateSettings={updateSettings} />;
        ui = <>
            <span>Click on the map to add keypoints.</span>
            <button onClick={() => {
                setState({ mode: "nothing" });
            }}>Finish</button>
        </>;
    }
    if (state.mode === "removing") {
        controller = <RemoveCoverageController state={state} setState={setState} updateSettings={updateSettings} />
        ui = <>
            <span>Right click and drag over the paths you want to remove.</span>
            <button onClick={() => setState({ mode: "nothing" })}>Done</button>
        </>
    }
    controller = <>
        {controller}
        <AddedEdges edges={settings.edges_to_visit} />
    </>;
    return [controller, ui];
}
const AddCoverageController = React.memo(function ({ state, setState, updateSettings }: CoverageControllerProps) {
    useMapEvents({
        click: (e: L.LeafletMouseEvent) => {
            if (state.mode !== "adding") return;
            const { lat, lng } = e.latlng;
            if (state.points.length > 0) {
                const lastPoint = state.points[state.points.length - 1];
                getEdgeFromCoords(lastPoint, [lat, lng]).then(e => {
                    updateSettings(s => ({
                        ...s,
                        edges_to_visit: [...s.edges_to_visit, e]
                    }));
                });
            }
            setState({
                ...state,
                points: [...state.points, [lat, lng]]
            });
        },
    });
    if (state.mode === "adding") {
        return <>
            {state.points.map((pt, i) => (
                <CircleMarker
                    key={i}
                    center={pt}
                    radius={12}
                    color="purple"
                />
            ))}
        </>
    }
    else return null;
});
const RemoveCoverageController = React.memo(function ({ state, setState, updateSettings }: CoverageControllerProps) {
    const map = useMap();
    React.useEffect(() => {
        map.dragging.disable();
        return () => { map.dragging.enable(); };
    }, [map]);
    const [rect, setRect] = useState<[[number, number], [number, number]] | null>(null);
    const rectStart = React.useRef<[number, number] | null>(null);
    useMapEvents({
        mousedown: (e) => {
            if (e.originalEvent.buttons === 1) rectStart.current = [e.latlng.lat, e.latlng.lng];
        },
        mousemove: (e) => {
            if (e.originalEvent.buttons === 1) {
                if (rectStart.current) {
                    setRect([
                        rectStart.current,
                        [e.latlng.lat, e.latlng.lng]
                    ]);
                }
            };
        },
        mouseup: (e) => {
            if (!rect) return;
            const [ylow, yhigh] = [rect[0][0], rect[1][0]].toSorted((a, b) => a - b);
            const [xlow, xhigh] = [rect[0][1], rect[1][1]].toSorted((a, b) => a - b);
            updateSettings(s => {
                const edges = s.edges_to_visit.filter(ed => {
                    return !ed.positions.every(([y, x]) => (
                        (ylow <= y && y <= yhigh && xlow <= x && x <= xhigh)
                    ));
                });
                if (edges.length === s.edges_to_visit.length) return s;
                else return {
                    ...s,
                    edges_to_visit: edges
                };
            });
            rectStart.current = null;
            setRect(null);
        },
    });

    if (state.mode === "removing") {
        if (rect) {
            return <Rectangle
                bounds={rect}
                color="red"
            />;
        }
    }
    else return null;
});

