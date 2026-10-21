#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import re


def get_vehicles(filename):
    root = ET.parse(filename).getroot()

    result = []

    for vehicle in root.findall(".//vehicle"):
        vid = vehicle.get("id")
        vtype = vehicle.get("type")
        depart = vehicle.get("depart")

        route = vehicle.find("route")
        edges = route.get("edges").split() if route is not None else []

        result.append({
            "id": vid,
            "type": vtype,
            "depart": depart,
            "route": edges,
        })

    return result


def get_tls(filename):
    text = open(filename, "r", encoding="utf-8").read()

    ids = re.findall(
        r'<junction\b[^>]*id="([^"]+)"[^>]*type="traffic_light"',
        text
    )

    return sorted(set(ids))


ambulances = get_vehicles("ambulances.rou.xml")
cars = get_vehicles("traffic.rou.xml")
tls = get_tls("network.net.xml")

print("=" * 60)
print("CITY NETWORK INVENTORY")
print("=" * 60)

print(f"\nAmbulances : {len(ambulances)}")
for a in ambulances:
    print(
        f"  {a['id']:8s} "
        f"depart={a['depart']:>6} "
        f"route={a['route']}"
    )

print(f"\nCars       : {len(cars)}")

if cars:
    print("First 10 cars:")
    for c in cars[:10]:
        print(
            f"  {c['id']:8s} "
            f"depart={c['depart']:>6}"
        )

print(f"\nSignals    : {len(tls)}")
for tls_id in tls:
    print(f"  {tls_id}")

print("\nSuggested NR-V2X capacity:")
print(f"  {len(ambulances)} ambulances")
print(f"  {len(cars)} cars")
print(f"  {len(tls)} intersections")
print(f"  Total required nodes = "
      f"{len(ambulances) + len(cars) + len(tls)}")
print("  Recommended MAX_UE = 600")
