#!/bin/bash
rosservice call /$UAV_NAME/trajectory_generation/path "path:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: 'fcu_untilted'
  input_id: 0
  use_heading: true
  fly_now: false 
  stop_at_waypoints: false
  loop: false
  override_constraints: false
  override_max_velocity_horizontal: 0.0
  override_max_acceleration_horizontal: 0.0
  override_max_jerk_horizontal: 0.0
  override_max_velocity_vertical: 0.0
  override_max_acceleration_vertical: 0.0
  override_max_jerk_vertical: 0.0
  relax_heading: false
  points:

  - position: {x: 2.5, y: 2.5, z: 0.0}
    heading: 0.0
  - position: {x: 2.5, y: -2.5, z: 0.0}
    heading: 0.0
  - position: {x: -2.5, y: -2.5, z: 0.0}
    heading: 0.0
  - position: {x: -2.5, y: 2.5, z: 0.0}
    heading: 0.0
  - position: {x: 2.5, y: 2.5, z: 0.0}
    heading: 0.0"
