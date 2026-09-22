#!/usr/bin/env python3
"""Offline structural validation; it does not require ROS to be installed."""

from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    required = [
        "CMakeLists.txt", "package.xml",
        "msg/DynamicObject.msg", "msg/DynamicObjectArray.msg",
        "msg/SemanticDetection2D.msg", "msg/SemanticDetection2DArray.msg",
        "launch/complete_system.launch", "launch/perception.launch",
        "launch/avoidance.launch", "config/dynamic_detector.yaml",
        "config/dynamic_cluster.yaml", "config/dynamic_risk.yaml",
        "config/static_risk.yaml", "config/static_map.yaml",
        "src/fastlio_bridge.cpp", "src/cloud_bridge.cpp",
        "src/dynamic_detector.cpp", "src/dynamic_cluster.cpp",
        "src/collision_risk.cpp", "src/static_collision_risk.cpp",
        "src/avoidance_fusion.cpp", "src/static_map_builder.cpp",
        "src/system_watchdog.cpp",
    ]
    for relative in required:
        require((ROOT / relative).is_file(), f"missing {relative}")

    ET.parse(ROOT / "package.xml")
    for launch in (ROOT / "launch").glob("*.launch"):
        ET.parse(launch)

    dynamic_msg = (ROOT / "msg/DynamicObject.msg").read_text()
    for field in ("id", "pose", "twist", "size", "confidence",
                  "semantic_class", "predicted"):
        require(re.search(rf"\b{field}\b", dynamic_msg),
                f"DynamicObject.msg missing {field}")

    complete_launch = (ROOT / "launch/complete_system.launch").read_text()
    require('name="enable_control" default="false"' in complete_launch,
            "real control must default to false")

    cmake = (ROOT / "CMakeLists.txt").read_text()
    sources = re.findall(r"add_fastlio_node\([^ ]+ ([^)]+)\)", cmake)
    for source in sources:
        require((ROOT / "src" / source.strip()).is_file(),
                f"CMake source missing: {source}")

    # Threshold order is safety critical and is checked without PyYAML.
    static_risk = (ROOT / "config/static_risk.yaml").read_text()
    values = {}
    for key in ("uav_radius", "emergency_clearance", "safe_clearance"):
        match = re.search(rf"^{key}:\s*([0-9.]+)", static_risk, re.MULTILINE)
        require(match, f"missing threshold {key}")
        values[key] = float(match.group(1))
    require(values["uav_radius"] > 0.0, "uav radius must be positive")
    require(values["emergency_clearance"] < values["safe_clearance"],
            "emergency_clearance must be below safe_clearance")

    # Topic ownership is intentional: cloud_bridge publishes a current scan,
    # while static_map_builder is the sole owner of the accumulated global map.
    # Making both topics equal creates a positive feedback/self-subscription loop.
    static_map = (ROOT / "config/static_map.yaml").read_text()
    scan_match = re.search(r"^cloud_topic:\s*(\S+)", static_map, re.MULTILINE)
    global_match = re.search(r"^output_topic:\s*(\S+)", static_map, re.MULTILINE)
    require(scan_match and global_match, "static map topics are missing")
    require(scan_match.group(1) == "/uav1/fastlio/registered_scan",
            "static map input must be the current registered scan")
    require(global_match.group(1) == "/uav1/fastlio/cloud_map",
            "global map must own /uav1/fastlio/cloud_map")
    require(scan_match.group(1) != global_match.group(1),
            "static map input and output must differ to prevent feedback")
    require(re.search(r"^free_space_output_topic:\s*/uav1/local_free_space\s*$",
                      static_map, re.MULTILINE),
            "local free-space output is required for stale obstacle clearing")
    require(re.search(r"^enable_free_space_clearing:\s*true\s*$",
                      static_map, re.MULTILINE),
            "negative occupancy clearing must be enabled")

    for config_name in ("dynamic_detector.yaml", "static_risk.yaml"):
        config = (ROOT / "config" / config_name).read_text()
        require(re.search(
            r"^cloud_topic:\s*/uav1/fastlio/registered_scan\s*$",
            config, re.MULTILINE),
            f"{config_name} must consume the current scan, not global map")

    planner = (ROOT / "config/path_planner.yaml").read_text()
    require(re.search(
        r"^local_free_space_topic:\s*/uav1/local_free_space\s*$",
        planner, re.MULTILINE),
        "planner must consume local free-space evidence")
    require(re.search(r"^local_replan_to_final_goal:\s*true\s*$",
                      planner, re.MULTILINE),
            "corrected planner must re-optimize to the final goal")

    print("PASS: package structure, XML, messages, targets and safety defaults")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
