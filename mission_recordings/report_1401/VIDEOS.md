# Mission videos - report_1401

No video carries on-screen words: pictures only.

| file | length | what it shows |
|---|---|---|
| `QSLAM_highlight.mp4` | 1:52.9 | **Final highlight reel** - the whole mission in at most two minutes: the input, the drive, inside the stack, arrival |
| `isaac_onboard.mp4` | 14:54.7 | **Rover camera** - the mast-mounted ZED camera, 1080p, rendered in Isaac Sim |
| `isaac_behind.mp4` | 14:50.0 | **Chase camera** - third-person follow view from behind the rover |
| `isaac_side.mp4` | 14:50.3 | **Side camera** - follow view from the rover's right flank |
| `isaac_top.mp4` | 14:57.0 | **Top-down camera** - bird's-eye follow view, 16 m above the rover |
| `rviz.mp4` | 25:59.3 | **RViz 3D view** - the operator's live view -- prior map, costmaps, plans, lidar, EKF pose -- cropped from the screen capture (full grab, with its panels and terminals, in with_text_originals/) |
| `costmap.mp4` | 14:17.0 | **Costmaps and planning** - global costmap (GeoTIFF slope prior + lidar + inflation), Smac A* route, rolling local costmap with MPPI candidates |
| `ekf.mp4` | 14:17.0 | **EKF state estimation** - EKF estimate vs ground truth vs wheel-only dead reckoning, with the error and heading histories |
| `slam_graph.mp4` | 14:17.0 | **Mapping and the live ROS 2 graph** - map built from lidar with the pose graph, beside the node/topic graph whose links light up while their topics carry traffic |
| `geo_pipeline.mp4` | 0:52.9 | **GeoTIFF -> DEM -> UTM sequence** - the GeoTIFF on its UTM grid, the DEM, slope, occupancy grid, UTM and map frames, and this run's goal and route |

## Mission

- commanded goal: UTM 44N goal E 403562.52 N 1450024.12 -> map (56.00, 34.00)   [anchor E 403506.52 N 1449990.12]
- outcome: GOAL REACHED
- driven 187.5 m, finished 0.17 m from the goal
- EKF error at arrival 0.73 m (RMS 0.33 m)
- no GNSS receiver is read at any point
