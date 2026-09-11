"""The menu's path handling: repo paths must reach WSL through the ~/utopia symlink."""

import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("tiltlab_menu", REPO / "scripts" / "tiltlab_menu.py")
menu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(menu)


def test_bash_path_keeps_home_expandable():
    # a quoted "~" would be a literal directory name in bash, so it has to come out as $HOME
    assert menu.bash_path("~/utopia/wsl/x.sh") == '"$HOME"/utopia/wsl/x.sh'
    assert menu.bash_path("~/a dir/x") == '"$HOME"' + "'/a dir/x'"  # spaces still quoted
    assert menu.bash_path("/dev/ttyACM0") == "/dev/ttyACM0"


def test_repo_paths_map_onto_the_wsl_symlink():
    harness = REPO / "exports" / "gazebo_hitl" / "x_hitl"
    assert menu.to_wsl(harness) == "~/utopia/vibe-coded/exports/gazebo_hitl/x_hitl"
    assert menu.to_wsl(REPO) == "~/utopia/vibe-coded"


def test_every_menu_entry_is_callable():
    assert len(menu.MENU) == 8
    assert all(callable(action) for _, action in menu.MENU)
