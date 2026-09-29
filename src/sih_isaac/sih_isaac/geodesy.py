"""WGS84 lat/lon -> UTM, in pure Python (no pyproj, no runtime deps).

The operator may type a goal as lat/lon instead of UTM easting/northing. That
conversion is plain spheroidal arithmetic on numbers typed at a keyboard --
there is still no receiver, no NavSatFix, nothing that measures a position.

Standard Snyder/USGS transverse-Mercator series (the same one EPSG:326xx
implements); agrees with pyproj to well under a millimetre, which
tests/test_geodesy.py checks against the site datum.
"""
import math

A = 6378137.0                      # WGS84 semi-major axis
F = 1.0 / 298.257223563            # WGS84 flattening
E2 = F * (2.0 - F)                 # first eccentricity squared
EP2 = E2 / (1.0 - E2)              # second eccentricity squared
K0 = 0.9996                        # UTM scale factor on the central meridian


def utm_zone(lon_deg: float) -> int:
    return int(math.floor((lon_deg + 180.0) / 6.0)) % 60 + 1


def latlon_to_utm(lat_deg: float, lon_deg: float, zone: int | None = None):
    """(lat, lon) in degrees -> (easting, northing, zone, north_hemisphere)."""
    if not -80.0 <= lat_deg <= 84.0:
        raise ValueError(f"latitude {lat_deg} is outside the UTM band (-80..84)")
    if not -180.0 <= lon_deg <= 180.0:
        raise ValueError(f"longitude {lon_deg} is outside -180..180")

    zone = utm_zone(lon_deg) if zone is None else zone
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat, tan_lat = math.sin(lat), math.cos(lat), math.tan(lat)

    n = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = EP2 * cos_lat * cos_lat
    # keep the longitude difference in -pi..pi so a zone override still works
    a_ = (math.remainder(lon - lon0, 2.0 * math.pi)) * cos_lat

    m = A * ((1 - E2 / 4 - 3 * E2**2 / 64 - 5 * E2**3 / 256) * lat
             - (3 * E2 / 8 + 3 * E2**2 / 32 + 45 * E2**3 / 1024) * math.sin(2 * lat)
             + (15 * E2**2 / 256 + 45 * E2**3 / 1024) * math.sin(4 * lat)
             - (35 * E2**3 / 3072) * math.sin(6 * lat))

    easting = K0 * n * (a_ + (1 - t + c) * a_**3 / 6
                        + (5 - 18 * t + t * t + 72 * c - 58 * EP2) * a_**5 / 120) + 500000.0
    northing = K0 * (m + n * tan_lat
                     * (a_**2 / 2 + (5 - t + 9 * c + 4 * c * c) * a_**4 / 24
                        + (61 - 58 * t + t * t + 600 * c - 330 * EP2) * a_**6 / 720))
    north = lat_deg >= 0.0
    if not north:
        northing += 10000000.0
    return easting, northing, zone, north


def epsg_for(zone: int, north: bool) -> int:
    return (32600 if north else 32700) + zone
