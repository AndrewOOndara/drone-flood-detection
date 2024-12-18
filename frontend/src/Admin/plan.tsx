import React from "react";
import { AppSettings, PlanningOutput } from "../types";
import { Polyline } from "react-leaflet";
import { getPlan } from "../api";

export function usePlan(settings: AppSettings): [React.ReactNode, React.ReactNode] {
    const [plan, setPlan] = React.useState<PlanningOutput>({});
    const doPlan = async () => {
        setPlan(await getPlan());
    };
    let controller = null;
    let ui = null;
    const COLORS: React.CSSProperties['color'][] = ['red', 'green', 'blue', 'orange', 'brown'];
    if (plan) {
        controller = <>
            {Object.entries(plan).map(([k, v], i) => (
                <Polyline
                    key={k}
                    positions={v.positions}
                    color={COLORS[i%COLORS.length]}
                />
            ))}
        </>;
    }
    ui = <>
        <button onClick={doPlan}>Plan</button>
        <button onClick={() => setPlan({})}>Clear</button>
    </>
    return [controller, ui];
}