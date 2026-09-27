"""Georeferencing layer for the SIH26126 demo: UTM <-> map frame, and the
prior-terrain GeoTIFF.

GPS-DENIED BY CONSTRUCTION. Nothing here reads a receiver. The only external
geodetic input is a *surveyed site datum* -- one lat/lon fixed at survey time
and baked into the mission file -- exactly like a benchmark pillar on a test
range. The rover never measures its own lat/lon; it measures its pose in the
map frame with VSLAM, and this module converts that to UTM for reporting and
converts the operator's UTM goal into the map frame for planning.

    surveyed datum (lat, lon)  --pyproj-->  UTM anchor (E0, N0)   [offline]
    operator goal (E, N)       --utm_to_map-->  map (x, y)        [online]
    VSLAM pose (x, y)          --map_to_utm-->  UTM (E, N)        [reporting]
"""
from __future__ import annotations

import pathlib

import numpy as np
from pyproj import CRS, Transformer

# --- Surveyed site datum -----------------------------------------------------
# Open test range, Avadi, Tamil Nadu. Surveyed once, offline; a constant of the
# mission, not a measurement made by the vehicle.
SITE_LAT = 13.114700
SITE_LON = 80.109800

WGS84 = CRS.from_epsg(4326)
UTM_44N = CRS.from_epsg(32644)      # WGS 84 / UTM zone 44N -- correct for 78-84 E

_TO_UTM = Transformer.from_crs(WGS84, UTM_44N, always_xy=True)
_TO_WGS = Transformer.from_crs(UTM_44N, WGS84, always_xy=True)


def utm_zone_for_lon(lon: float) -> int:
    return int((lon + 180.0) // 6.0) + 1


def latlon_to_utm(lat: float, lon: float) -> tuple[float, float]:
    """WGS84 geographic -> UTM 44N easting/northing (metres)."""
    easting, northing = _TO_UTM.transform(lon, lat)
    return float(easting), float(northing)


def utm_to_latlon(easting: float, northing: float) -> tuple[float, float]:
    lon, lat = _TO_WGS.transform(easting, northing)
    return float(lat), float(lon)


# The map frame origin, in UTM. Computed once from the surveyed datum.
ANCHOR_E, ANCHOR_N = latlon_to_utm(SITE_LAT, SITE_LON)


def utm_to_map(easting: float, northing: float) -> tuple[float, float]:
    """UTM 44N -> local ENU map frame (metres, origin at the surveyed anchor).

    UTM easting/northing are already an ENU-aligned grid, so over a 150 m site
    this is a pure translation; grid convergence (~0.2 deg here) is absorbed by
    treating the map frame as grid-north rather than true-north.
    """
    return float(easting - ANCHOR_E), float(northing - ANCHOR_N)


def map_to_utm(x: float, y: float) -> tuple[float, float]:
    """Local ENU map frame -> UTM 44N easting/northing (metres)."""
    return float(x + ANCHOR_E), float(y + ANCHOR_N)


def map_to_latlon(x: float, y: float) -> tuple[float, float]:
    return utm_to_latlon(*map_to_utm(x, y))


# --- Prior terrain GeoTIFF ---------------------------------------------------

def write_geotiff(dem: np.ndarray, path: str | pathlib.Path, world_size: float) -> dict:
    """Write a north-up, UTM-44N-georeferenced float32 DEM.

    `dem[0]` is the NORTH edge (row 0 = +y), matching the heightmap convention
    already used by sih_sim.heightmap_generator.
    """
    import rasterio
    from rasterio.transform import from_origin

    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    n = dem.shape[0]
    res = world_size / (n - 1)
    # Upper-left corner of the upper-left *pixel*, in UTM.
    ul_e = ANCHOR_E - world_size / 2.0 - res / 2.0
    ul_n = ANCHOR_N + world_size / 2.0 + res / 2.0
    transform = from_origin(ul_e, ul_n, res, res)

    with rasterio.open(
        path, "w", driver="GTiff", height=n, width=n, count=1,
        dtype="float32", crs=UTM_44N, transform=transform,
        compress="deflate", tiled=True, blockxsize=256, blockysize=256,
    ) as dst:
        dst.write(dem.astype(np.float32), 1)
        dst.set_band_description(1, "terrain elevation above site datum (m)")
        dst.update_tags(
            SITE_LAT=f"{SITE_LAT:.6f}", SITE_LON=f"{SITE_LON:.6f}",
            ANCHOR_E=f"{ANCHOR_E:.3f}", ANCHOR_N=f"{ANCHOR_N:.3f}",
            SOURCE="SIH26126 prior survey DEM",
        )

    return dict(path=str(path), size_px=n, res_m=res,
                crs=UTM_44N.to_string(), transform=tuple(transform)[:6])


class PriorDEM:
    """A GeoTIFF read back off disk and sampled in the map frame.

    This is the *prior map* the global planner uses: it is read from the file,
    not from the simulator, so the demo genuinely exercises the GeoTIFF path.
    """

    def __init__(self, path: str | pathlib.Path):
        import rasterio

        self.path = str(path)
        with rasterio.open(self.path) as src:
            self.dem = src.read(1).astype(np.float64)
            self.transform = src.transform
            self.crs = src.crs
            self.bounds = src.bounds
            self.res = float(src.res[0])
            self.tags = src.tags()
        self.n = self.dem.shape[0]

    def describe(self) -> str:
        return (f"{pathlib.Path(self.path).name}  {self.n}x{self.n} px @ "
                f"{self.res:.3f} m  {self.crs.to_string()}")

    def _pixel_of_map(self, x, y):
        """Map (x, y) -> fractional (row, col). Row 0 is the north edge."""
        e = np.asarray(x, dtype=np.float64) + ANCHOR_E
        n = np.asarray(y, dtype=np.float64) + ANCHOR_N
        col = (e - self.transform.c) / self.res - 0.5
        row = (self.transform.f - n) / self.res - 0.5
        return row, col

    def elevation(self, x, y):
        """Bilinearly sampled elevation (m) at map coords, clamped at the edge."""
        row, col = self._pixel_of_map(x, y)
        r0 = np.clip(np.floor(row).astype(int), 0, self.n - 2)
        c0 = np.clip(np.floor(col).astype(int), 0, self.n - 2)
        fr = np.clip(row - r0, 0.0, 1.0)
        fc = np.clip(col - c0, 0.0, 1.0)
        d = self.dem
        top = d[r0, c0] * (1 - fc) + d[r0, c0 + 1] * fc
        bot = d[r0 + 1, c0] * (1 - fc) + d[r0 + 1, c0 + 1] * fc
        return top * (1 - fr) + bot * fr

    def slope_deg(self, x, y, eps: float = 1.0):
        """Terrain slope magnitude in degrees, by central difference."""
        zx = (self.elevation(x + eps, y) - self.elevation(x - eps, y)) / (2 * eps)
        zy = (self.elevation(x, y + eps) - self.elevation(x, y - eps)) / (2 * eps)
        return np.degrees(np.arctan(np.hypot(zx, zy)))
