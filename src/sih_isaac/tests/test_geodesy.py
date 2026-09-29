"""Check the dependency-free lat/lon -> UTM against pyproj.

pyproj lives only in .demoenv (nothing at runtime imports it), so run this with:

    PYTHONPATH=$PWD/src/sih_isaac .demoenv/bin/python \
        src/sih_isaac/tests/test_geodesy.py
"""
import json
import pathlib
import random
import sys

from pyproj import Transformer

from sih_isaac.geodesy import epsg_for, latlon_to_utm

WORLD = pathlib.Path(__file__).resolve().parents[1] / "generated/world.json"
TOL_M = 0.002          # 2 mm; the series is good to ~1 mm worldwide


def main() -> int:
    bad = 0
    datum = json.loads(WORLD.read_text())["site_datum"]

    # the site datum must reproduce the anchor baked into world.json
    e, n, zone, north = latlon_to_utm(datum["lat"], datum["lon"])
    assert epsg_for(zone, north) == datum["epsg"], (zone, north, datum["epsg"])
    de, dn = e - datum["anchor_e"], n - datum["anchor_n"]
    print(f"site datum: dE {de*1000:+.6f} mm  dN {dn*1000:+.6f} mm")
    if max(abs(de), abs(dn)) > 1e-6:
        print("FAIL: site datum does not reproduce the baked anchor")
        bad += 1

    random.seed(7)
    for name, lon_range, count in (("zone 44", (78.1, 84.0), 5000),
                                   ("global", (-179.9, 179.9), 5000)):
        worst = 0.0
        for _ in range(count):
            lat = random.uniform(-79.5, 83.5)
            lon = random.uniform(*lon_range)
            e, n, zone, north = latlon_to_utm(lat, lon)
            tr = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg_for(zone, north)}",
                                      always_xy=True)
            pe, pn = tr.transform(lon, lat)
            worst = max(worst, abs(e - pe), abs(n - pn))
        print(f"{name} sweep ({count} pts): worst axis error {worst*1000:.4f} mm")
        if worst > TOL_M:
            print(f"FAIL: {name} exceeds {TOL_M*1000:.0f} mm")
            bad += 1

    print("OK" if not bad else f"{bad} failure(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
