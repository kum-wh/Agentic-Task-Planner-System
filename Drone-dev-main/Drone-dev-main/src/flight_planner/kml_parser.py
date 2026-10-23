from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from .models import Coordinate, MissionInput, MissionRegion


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _coordinates(element: ET.Element) -> list[Coordinate]:
    text = element.text or ""
    result = []
    for raw in text.split():
        values = raw.split(",")
        if len(values) >= 2:
            result.append(Coordinate(float(values[0]), float(values[1]), float(values[2]) if len(values) > 2 else 0.0))
    return result


def parse_kml(path: str | Path) -> MissionInput:
    root = ET.parse(path).getroot()
    placemarks: dict[str, ET.Element] = {}
    for placemark in root.iter():
        if _local_name(placemark.tag) != "Placemark":
            continue
        name = next((child.text or "" for child in placemark if _local_name(child.tag) == "name"), "").strip()
        placemarks[name.lower()] = placemark

    def find(name: str) -> ET.Element:
        try:
            return placemarks[name.lower()]
        except KeyError as exc:
            available = ", ".join(sorted(placemarks))
            raise ValueError(f"KML is missing {name}; found: {available}") from exc

    def point_region(name: str) -> Coordinate:
        placemark = find(name)
        for element in placemark.iter():
            if _local_name(element.tag) == "coordinates":
                points = _coordinates(element)
                if points:
                    return points[0]
        raise ValueError(f"KML placemark {name} has no coordinates")

    def polygon_region(name: str) -> MissionRegion:
        placemark = find(name)
        for element in placemark.iter():
            if _local_name(element.tag) == "outerBoundaryIs":
                for child in element.iter():
                    if _local_name(child.tag) == "coordinates":
                        points = _coordinates(child)
                        if len(points) >= 3:
                            return MissionRegion(name, points)
        raise ValueError(f"KML placemark {name} has no polygon outer boundary")

    return MissionInput(point_region("GCSlocation"), polygon_region("GeoFence"), polygon_region("OpsArea"))
