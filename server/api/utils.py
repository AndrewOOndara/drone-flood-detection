import networkx
import osmnx
import geopandas
from typing import Optional
import shapely


def remove_consecutive_dup(l):
    prev = None
    return [prev:=a for a in l if prev!=a]

def fill_route(egdf: Optional[geopandas.GeoDataFrame], G: networkx.MultiDiGraph, paths: list[list[int]]) -> list[list[tuple[float, float]]]:

    if egdf is None:
        egdf = osmnx.convert.graph_to_gdfs(G, nodes=False, edges=True).droplevel(2)
        egdf = egdf[~egdf.index.duplicated()]

    lines = dict(egdf.reindex(list(set(
        [(p1, p2) for path in paths for (p1, p2) in zip(path[:-1], path[1:])] + 
        [(p2, p1) for path in paths for (p1, p2) in zip(path[:-1], path[1:])]
    )))["geometry"])

    return [
        remove_consecutive_dup([
            p
            for e in zip(path[:-1], path[1:])
            for p in ([(y, x) for (x, y) in lines[e].coords] if lines[e] is not None else [])
        ])
        for path in paths
    ]


# for p in ([(y, x) for (x, y) in lines[e].coords] if G[e] is not None else [])
