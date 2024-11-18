from typing import TypeAlias, Optional
from dataclasses import dataclass

import networkx
import osmnx
import rtree

from PIL import Image

import matplotlib.pyplot as plt

from io import BytesIO
import pickle
import os

# (left, bottom, right, top)
RectCoords: TypeAlias = tuple[float, float, float, float]

def create_map_graph(cache_path: str, lat_lon: tuple[float, float], **osmnx_kwargs) -> networkx.MultiDiGraph:
    if os.path.exists(cache_path):
        with open(cache_path, 'rb') as f:
            loaded = pickle.load(f)
            if loaded['lat_lon'] == lat_lon and loaded['kwargs'] == osmnx_kwargs: return loaded['map']

    loaded = dict(
        lat_lon=lat_lon,
        kwargs=osmnx_kwargs,
        map=osmnx.graph_from_point(lat_lon, **osmnx_kwargs)
    )
    with open(cache_path, 'wb') as f:
        return pickle.dump(loaded, f)
    return loaded['map']

@dataclass
class ZoneDbEntry:
    time: int
    drone_id: int
    flooded: bool
    confidence: float

PATH_CHECK_AREA_THRESHOLD: float = 1e-4
PATH_ZERO_THRESHOLD: float = 1e-7

class FloodDatabase:
    def __init__(self, map_graph: networkx.MultiDiGraph):
        self.zones: rtree.Index = rtree.Index()
        self.map_graph: networkx.MultiDiGraph = map_graph
        self.id_counter: int = 0
        self.entries_map: dict[int, ZoneDbEntry] = dict()

        # objects older than this time are considered irrelevant
        self.outdated_threshold_time: int = 0

    def _delete_one(self, item: rtree.index.Item):
        self.zones.delete(item.id, item.bbox)
        self.entries_map.pop(item.id)

    def _add_one(self, bbox: RectCoords, ent: ZoneDbEntry):
        self.id_counter += 1
        this_id = self.id_counter
        self.zones.insert(this_id, bbox)
        self.entries_map[this_id] = ent

    # Remove outdated zones from the list and return the remaining ones.
    def _clean_list(self, l: list[rtree.index.Item]) -> list[rtree.index.Item]:
        remaining = []
        for item in l:
            dbe = self.entries_map[item.id]
            if dbe.time < self.outdated_threshold_time:
                self._delete_one(item)
            else:
                remaining.append(item)
        return remaining

    # Update the database with a new rectangle.
    def update_zone(self, rect: RectCoords, ent: ZoneDbEntry):
        intersections = list(self.zones.intersection(rect, True))
        intersections = self._clean_list(intersections)

        for item in intersections:
            dbe = self.entries_map[item.id]
            self._delete_one(item)
            for sp in _rect_split(rect, item.bbox):
                self._add_one(sp, dbe)

        self._add_one(rect, ent)

    def is_path_wet(self, path: list[tuple[float, float]]) -> bool:
        path_x = [x for (x, y) in path]
        path_y = [y for (x, y) in path]
        box = (min(path_x), min(path_y), max(path_x), max(path_y))
        if _rect_area(box) > PATH_CHECK_AREA_THRESHOLD:
            lp = len(path)
            if lp > 2:
                mididx = lp // 2
                return self.is_path_wet(path[:mididx+1]) or self.is_path_wet(path[mididx:])
            else:
                (px, py) = path[0]
                (qx, qy) = path[1]
                midpoint = ((px+qx)/2, (py+qy)/2)
                return self.is_path_wet([path[0], midpoint]) or self.is_path_wet([midpoint, path[1]])

        intersections = list(self.zones.intersection(box, True))
        intersections = self._clean_list(intersections)
        for item in intersections:
            dbe = self.entries_map[item.id]
            if dbe.flooded:
                for p1, p2 in zip(path[:-1], path[1:]):
                    if _rect_intersects_line(item.bbox, p1, p2): return True
        return False

    def _plot(self, ax: plt.Axes):
        osmnx.plot_graph(self.map_graph, ax=ax, show=False)
        bounds = self.zones.get_bounds()
        if len(self.entries_map) > 0:
            for item in self.zones.intersection(bounds, True):
                (x0, y0, x1, y1) = item.bbox
                dbe = self.entries_map[item.id]
                ec = 'tab:blue' if dbe.flooded else 'tab:gray'
                ax.add_patch(plt.Rectangle((x0, y0), x1-x0, y1-y0, facecolor=ec, edgecolor='tab:red'))
                ax.annotate(f"{item.id}", ((x0+x1)/2, (y0+y1)/2), ha='center', va='center', color='white', size=9)

def _rect_intersects_line(r: RectCoords, p: tuple[float, float], q: tuple[float, float]) -> bool:
    (bx0, by0, bx1, by1) = r
    (px, py) = p
    (qx, qy) = q
    if (abs(px - qx) < PATH_ZERO_THRESHOLD):
        return (bx0 <= px <= bx1) and (min(py, qy) <= by1) and (max(py, qy) >= by0)
    if (abs(py - qy) < PATH_ZERO_THRESHOLD):
        return (by0 <= py <= by1) and (min(px, qx) <= bx1) and (max(px, qx) >= bx0)
    sy = (qy-py)/(qx-px)
    sx = (qx-px)/(qy-py)
    return (
        (by0 <= (sy * (bx0-px) + py) <= by1) or
        (by0 <= (sy * (bx1-px) + py) <= by1) or
        (bx0 <= (sx * (by0-py) + px) <= bx1) or
        (bx0 <= (sx * (by1-py) + px) <= bx1)
    )

def _rect_area(r: RectCoords) -> float:
    (x0, y0, x1, y1) = r
    return (x1 - x0) * (y1 - y0)

def _rect_contains(big: RectCoords, small: RectCoords) -> bool:
    (bx0, by0, bx1, by1) = big
    (sx0, sy0, sx1, sy1) = small
    return (bx0 <= sx0 and by0 <= sy0 and bx1 >= sx1 and by1 >= sy1)

def _rect_split(splitter: RectCoords, splittee: RectCoords) -> list[RectCoords]:
    (bx0, by0, bx1, by1) = splitter
    (sx0, sy0, sx1, sy1) = splittee
    output_rects = []
    if bx0 > sx0 and bx0 < sx1:
        output_rects.append((sx0, sy0, bx0, sy1))
        sx0 = bx0
    if bx1 > sx0 and bx1 < sx1:
        output_rects.append((bx1, sy0, sx1, sy1))
        sx1 = bx1
    if by0 > sy0 and by0 < sy1:
        output_rects.append((sx0, sy0, sx1, by0))
        sy0 = by0
    if by1 > sy0 and by1 < sy1:
        output_rects.append((sx0, by1, sx1, sy1))
        sy1 = by1
    return output_rects

