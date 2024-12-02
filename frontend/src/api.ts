import { AppSettings } from "./types";

const API_BASE = "http://localhost:5000/api/v1"

export async function getSettings(): Promise<AppSettings> {
    const req = fetch(
        API_BASE + "/settings",
        {
            method: 'GET'
        }
    );
    const res = await req;
    const out = res.json();
    return out;
}