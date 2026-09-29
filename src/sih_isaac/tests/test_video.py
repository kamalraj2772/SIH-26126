"""Check the mission-video bookkeeping: goals, mission marks, timebase, no words.

    python3 src/sih_isaac/tests/test_video.py

No ROS, no ffmpeg, no recorded run needed: every case builds its own tiny run
directory. Runs under the system python3 or .demoenv alike.
"""
import pathlib
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                    # noqa: E402

W = video.world()
AE, AN = W["site_datum"]["anchor_e"], W["site_datum"]["anchor_n"]
BERM = W["berm"]


def run_dir(**files):
    d = pathlib.Path(tempfile.mkdtemp(prefix="vidtest_"))
    for name, text in files.items():
        (d / name.replace("__", ".")).write_text(text)
    return d


def log(rows):
    head = "t,gt_x,gt_y,gt_yaw,ekf_x,ekf_y,cmd_v,cmd_w\n"
    return head + "".join(",".join(f"{v}" for v in r) + "\n" for r in rows)


def test_goal_sources():
    # lat/lon, as utm_goal.py logs it
    g = video.goal_info(run_dir(goal__log=(
        "[utm_goal] WGS84 goal lat 13.115009 lon 80.109338 -> UTM 44N "
        "E 403456.58 N 1450024.49 -> map (-49.94, 34.37)\n"
        "[utm_goal] finished with status 4  -- GOAL REACHED\n")))
    assert (g["lat"], g["lon"]) == (13.115009, 80.109338)
    assert (g["map_x"], g["map_y"]) == (-49.94, 34.37)
    assert g["reached"] and g["status"] == 4
    # UTM easting/northing
    g = video.goal_info(run_dir(goal__log=(
        "[utm_goal] UTM 44N goal E 403562.52 N 1450024.12 -> map (56.00, "
        "34.00)   [anchor E 403506.52 N 1449990.12]\n")))
    assert (g["easting"], g["map_x"]) == (403562.52, 56.0) and g["lat"] is None
    # map frame
    g = video.goal_info(run_dir(goal__log=(
        "[utm_goal] map goal (12.50, -3.25) = UTM E 403519.02 N 1449986.87\n")))
    assert (g["map_x"], g["map_y"], g["northing"]) == (12.5, -3.25, 1449986.87)
    # a goal clicked in RViz: only the recorder's note of the plan's end
    g = video.goal_info(run_dir(goal_seen__txt="20.0 5.0\n"))
    assert (g["map_x"], g["map_y"]) == (20.0, 5.0)
    assert abs(g["easting"] - (20.0 + AE)) < 1e-6
    # nothing at all: the committed default goal
    g = video.goal_info(run_dir())
    assert (g["map_x"], g["map_y"]) == (W["goal_default"]["x"],
                                        W["goal_default"]["y"])


def test_marks_follow_the_berm():
    rows = []
    for i in range(400):                      # 80 s at 5 Hz
        t = 10.0 + i * 0.2
        x = -20.0 + max(0.0, t - 20.0) * 0.8  # still until t = 20, then 0.8 m/s
        v = 0.8 if t >= 20.0 else 0.0
        rows.append((round(t, 2), round(x, 3), 25.0, 0.0, round(x, 3), 25.0,
                     v, 0.0))
    d = run_dir(mission_log__csv=log(rows))
    m = video.marks(d)
    assert abs(m["move"] - 20.0) < 0.21
    assert abs(m["tunnel_in"] - (20.0 + (BERM["x0"] + 20.0) / 0.8)) < 0.3
    assert abs(m["tunnel_out"] - (20.0 + (BERM["x1"] + 20.0) / 0.8)) < 0.3


def test_log_tolerates_nan_and_junk():
    d = run_dir(mission_log__csv=log(
        [(1.0, 0, 0, 0, "nan", "nan", 0, 0), (1.2, 0.1, 0, 0, 0.1, 0, 0.3, 0)])
        + "garbage,line\n")
    assert video.read_log(d).shape == (2, 8)
    st = video.stats(d)
    assert abs(st["loc_err"]) < 1e-9 and abs(st["path_len"] - 0.1) < 1e-9


def test_rviz_mapper():
    d = run_dir(timebase__csv="sim_t,wall_t\n100,1000\n110,1020\n120,1040\n",
                rviz_start__txt="990\n")
    f = video.rviz_mapper(d)
    assert abs(f(100) - 10.0) < 1e-9       # sim runs at 0.5x wall here
    assert abs(f(115) - 40.0) < 1e-9
    assert f(0) >= 0.0                      # clamped, never negative
    assert video.rviz_mapper(run_dir())(42.0) == 42.0    # no table: identity


def test_strip_words_leaves_only_numbers():
    fig = video.new_figure(640, 360)
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ax.plot([0, 1], [0, 1], label="line")
    ax.set_title("title")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend()
    ax.text(0.5, 0.5, "words")
    ax.annotate("note", (0.2, 0.2))
    sec = ax.secondary_xaxis("top")
    sec.set_xlabel("utm")
    fig.text(0.5, 0.95, "caption")
    video.strip_words(fig)
    video.fig_rgb(fig)                                  # draws without error
    shown = [t.get_text() for t in fig.findobj(
        lambda a: hasattr(a, "get_text") and a.get_visible())
        if t.get_text().strip()]
    for word in shown:                                  # tick numbers only
        assert word.replace(".", "").replace("\u2212", "").replace("-", "")\
            .strip().isdigit(), word


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"{name}: ok")
    print("OK")
