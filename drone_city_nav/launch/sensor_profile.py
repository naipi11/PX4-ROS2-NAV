#!/usr/bin/env python3
"""The navigation sensor profiles: what the vehicle carries and whose
observability the speed law and the gaze answer to. The stereo pair with six
8 x 8 time-of-flight sensors (up, down, front, back, left and right) is the
default; the 3D navigation lidar remains available on request."""

from __future__ import annotations


CAMERA_PROFILES = ("none", "stereo_tof")
NAVIGATION_SENSOR_PROFILES = ("lidar", "stereo_tof")
DEFAULT_CAMERA_PROFILE = "stereo_tof"
DEFAULT_NAVIGATION_SENSOR_PROFILE = "stereo_tof"

# What the stereo profile guarantees to see, measured in roadmap item 14: the
# pair recovers depth it can stand behind out to 6.4 m (disparity error 0.45 px
# at p90, a quarter-metre voxel) inside 60 degrees either side of the heading
# and 52.4 degrees above and below the horizon (120 degrees on a 4:3 imager);
# the up/down time-of-flight pair sees 2.8 m inside 22.5 degrees of the
# vertical. The four horizontal ToFs also feed obstacle memory. A motion the
# vehicle does not face answers to what memory has observed along it; a
# faced motion between the two fields is held to the unobserved speed. A
# vertical approach keeps 1.0 m: the body's half height (0.35 m), the vertical
# estimate error, a voxel and the tracking error, where the 2.0 m of a
# horizontal one is sized by the 0.82 m envelope.
# A climb or a descent with no heading to face surveys the walls it passes at
# 1.5 rad/s: the pair's 120 degree field leaves 240 degrees to sweep, 2.8 s at
# that rate, 2.8 m of climb at the 1 m/s the cone admits, inside the 3.9 m of
# wall the pair's vertical half-angle covers 1.5 m away. Measured before it on
# the camera flights r607 and r600 (8f6d248f, f0d64d86): one or two of eight
# heading sectors faced per half metre of a 12 m shaft, then 20 s of probing
# exits at the top through 39 routes; the estimator stayed healthy through
# the 1988 degrees the vehicle turned there (tracked features p50 170).
STEREO_TOF_OBSERVABILITY = {
    "guaranteed_lidar_detection_range_m": 6.4,
    "forward_detection_vertical_half_angle_deg": 52.4,
    "forward_detection_horizontal_half_angle_deg": 60.0,
    "vertical_detection_range_m": 2.8,
    "vertical_detection_cone_half_angle_deg": 22.5,
    "vertical_sensor_braking_physical_margin_m": 1.0,
    "unobserved_motion_speed_mps": 1.0,
    "gaze_follows_motion": True,
    "gaze_survey_yaw_rate_radps": 1.5,
    # The rays the depth node would publish if every pixel it samples matched:
    # a 1280 x 960 pair sampled every fourth pixel both ways. The share the
    # frame actually matched is what the braking contract reads as the range
    # the frame stands behind (roadmap item 17 stage 0).
    "sensor_expected_beam_count": (1280 // 4) * (960 // 4),
}

# What makes an obstacle memory a vision memory: the returns of the depth node,
# each a ray of its own, from the left camera on the nose mount (body
# forward-right-down). Two hits make a voxel occupied, not one: a matcher's
# outliers beside depth edges do not repeat from frame to frame the way a
# surface does (r481: 4.6 percent of the vision-occupied voxels had no lidar
# voxel within two and 3 percent of those were truly occupied; with two hits
# 0.7 percent, r483).
VISION_MEMORY_OVERRIDES = {
    "lidar_3d_hit_only_returns": True,
    "lidar_surface_interpolation_enabled": False,
    "lidar_3d_minimum_range_m": 0.3,
    "lidar_extrinsic_translation_body_frd_m": [0.32, -0.10, -0.02],
    "hit_weight": 2,
}

# K27's vertical allowance on the lidar profile.
#
# The figure belongs to the estimator that produces the height, not to the
# airframe: the 0.5 m the law was built with is the camera estimator's own
# measured error (0.3 to 0.6 m, 0.57 m by the end of an ordinary flight,
# r1183 to r1184), and the lidar profile inherited it although the one
# measured lidar-inertial height error is the 0.34 m it sat at near r3180's
# contact. That is why the figure is a parameter at all.
#
# It is kept at 0.5 here, unchanged from before the parameter existed, and the
# measurement alone was not enough to move it. Tried at 0.34 on 2026-11-07, the
# second lidar flight of that series crashed: the approach to the contact was
# clearance-limited, and the wider body clearance admitted more speed there
# (the ticks before the contact read a `clearance` limit of 1.18 to 1.81 m/s
# where the narrower band prices the same geometry at the contact floor). The
# allowance is a speed law in exactly the rooms a narrow band would open up,
# and the crash chain it entered is the one r3180 showed, so the change needs
# its own validated series before it can ship; until then the profile keeps the
# figure every accepted lidar flight was flown with. The evidence is under
# log/tools/vio/depth_visibility_20261103/r3800_lidar_crash_20261107/.
LIDAR_VERTICAL_ERROR_OVERRIDES = {
    "estimate_vertical_error_m": 0.5,
}


# The localization profile a flight flies when none is asked for: the
# estimator the navigation sensors feed, and never GNSS.
DEFAULT_LOCALIZATION_PROFILES = {
    "stereo_tof": "visual_inertial",
    "lidar": "lidar_inertial",
}


def validate_sensor_profiles(camera_profile: str, navigation_sensor_profile: str):
    """Return the two profiles, normalized; the stereo navigation profile needs
    the camera set mounted."""
    cameras = camera_profile.strip().lower()
    navigation = navigation_sensor_profile.strip().lower()
    if cameras not in CAMERA_PROFILES:
        raise ValueError(
            f"camera profile must be one of {', '.join(CAMERA_PROFILES)}, "
            f"got '{camera_profile}'"
        )
    if navigation not in NAVIGATION_SENSOR_PROFILES:
        raise ValueError(
            "navigation sensor profile must be one of "
            f"{', '.join(NAVIGATION_SENSOR_PROFILES)}, got "
            f"'{navigation_sensor_profile}'"
        )
    if navigation == "stereo_tof" and cameras != "stereo_tof":
        raise ValueError(
            "navigation_sensor_profile=stereo_tof requires camera_profile=stereo_tof"
        )
    return cameras, navigation


def stereo_tof_topics(
    world_name: str,
    model_name: str,
    vehicle_prefix: str = "",
    image_topic_suffix: str = "",
):
    """How one vehicle's sensor set reaches its depth node: (bridge arguments,
    bridge remaps, parameters of the `gazebo_stereo_depth_node` process). All
    six time-of-flight clouds cross the ROS-Gazebo bridge as PointCloud2 on
    `<vehicle_prefix>/tof/{up,down,front,back,left,right}/points`. The pair's
    images are taken directly from Gazebo inside the depth node's process,
    because bridging two 1280 x 960 streams stalled the simulator. The returns
    topic combines stereo depth and ToF rays for obstacle memory and, when
    enabled, the visual-inertial estimator's map alignment. A suffix names the
    topics an evaluation relay republishes the images on, the camera driver's
    input then."""
    sensor_prefix = (
        f"/world/{world_name}/model/{model_name}/link/stereo_tof_link/sensor"
    )
    bridge_arguments = []
    remappings = []
    depth_parameters = {"returns_topic": f"{vehicle_prefix}/stereo_depth/points"}
    for side in ("left", "right"):
        depth_parameters[f"{side}_gazebo_image_topic"] = (
            f"{sensor_prefix}/stereo_{side}/image{image_topic_suffix}"
        )
        depth_parameters[f"{side}_image_topic"] = (
            f"{vehicle_prefix}/stereo/{side}/image"
        )
    # The vehicle's barometer, relayed by the same process for the camera
    # estimator's hold through a dark stretch (roadmap item 17 stage 5).
    depth_parameters["gazebo_air_pressure_topic"] = (
        f"/world/{world_name}/model/{model_name}/link/base_link/sensor/"
        "air_pressure_sensor/air_pressure"
    )
    depth_parameters["air_pressure_topic"] = f"{vehicle_prefix}/barometer/air_pressure"
    for side in ("up", "down", "front", "back", "left", "right"):
        gz_topic = f"{sensor_prefix}/tof_{side}/scan/points"
        ros_topic = f"{vehicle_prefix}/tof/{side}/points"
        bridge_arguments.append(
            f"{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        )
        remappings.extend(["-r", f"{gz_topic}:={ros_topic}"])
        depth_parameters[f"tof_{side}_topic"] = ros_topic
    return bridge_arguments, remappings, depth_parameters
