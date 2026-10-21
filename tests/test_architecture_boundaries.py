"""Structural regression for authority, imports and the sole formal app."""
import ast
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT / "ros2_ws/src"


def imports(path):
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.ImportFrom) and node.module and not node.level:
            yield node.module.split(".")[0]
        elif isinstance(node, ast.Import):
            yield from (item.name.split(".")[0] for item in node.names)


def test_actual_package_dependencies_are_declared_and_acyclic():
    packages = {path.name: path for path in WORKSPACE.iterdir() if (path / "package.xml").is_file()}
    assert "companionbot_ros" not in packages
    graph = {}
    for name, path in packages.items():
        xml = ET.parse(path / "package.xml").getroot()
        declared = {node.text for node in xml if node.tag in ("depend", "exec_depend", "build_depend")}
        actual = set()
        for file in path.rglob("*.py"):
            if "test" in file.parts:
                continue
            for imported in imports(file):
                owner = "companionbot_interfaces" if imported == "companionbot_contracts" else imported
                if owner in packages and owner != name:
                    actual.add(owner)
        assert actual <= declared, (name, actual - declared)
        graph[name] = declared & packages.keys()
    def visit(name, trail):
        assert name not in trail, trail + [name]
        for dependency in graph[name]:
            visit(dependency, trail + [name])
    for name in graph:
        visit(name, [])


def test_control_and_follow_import_without_ros_or_simulator():
    # Fresh interpreter proves that a previous import has not hidden coupling.
    program = """
import sys
import companionbot_control.core.fixed_lqr
import companionbot_control.estimation.longitudinal_estimation
import companionbot_control.estimation.slope_estimation
import companionbot_control.runtime.mcu
import companionbot_follow.core.rolling_reference
import companionbot_follow.core.follow_governor
import companionbot_follow.core.radial_velocity_kf
import companionbot_follow.runtime.reference_producer
assert not {'mujoco', 'rclpy', 'torch', 'cv2'} & sys.modules.keys()
"""
    result = subprocess.run([sys.executable, "-B", "-c", program], cwd=ROOT,
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr


def test_behavior_has_no_control_planner_or_plant_import():
    package = WORKSPACE / "companionbot_behavior"
    forbidden = {"companionbot_control", "companionbot_follow", "companionbot_simulation", "mujoco"}
    for path in (package / package.name).rglob("*.py"):
        assert not set(imports(path)) & forbidden, path
    xml = ET.parse(package / "package.xml").getroot()
    assert not {node.text for node in xml if node.tag.endswith("depend")} & forbidden


def test_formal_script_and_ros_entry_delegate_to_one_application():
    expected = "companionbot_behavior.app"
    for path in [ROOT / "scripts/demo_stage8_interaction.py",
                 WORKSPACE / "companionbot_behavior/companionbot_behavior/ros/entry.py"]:
        tree = ast.parse(path.read_text())
        owners = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
                  and any(alias.name == "main" for alias in node.names)}
        assert owners == {expected}
    script = ast.parse((ROOT / "scripts/demo_stage8_interaction.py").read_text())
    assert not any(isinstance(node, (ast.ClassDef, ast.FunctionDef)) for node in script.body)
    launch = WORKSPACE / "companionbot_bringup/launch/interaction.launch.py"
    tree = ast.parse(launch.read_text())
    owners = [keyword.value.value for node in ast.walk(tree)
              if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "Node"
              for keyword in node.keywords if keyword.arg == "package"]
    assert owners.count("companionbot_behavior") == 1
    assert set(owners) <= {"companionbot_behavior", "companionbot_perception"}
