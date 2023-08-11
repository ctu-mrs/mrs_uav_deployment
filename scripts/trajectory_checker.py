#!/usr/bin/python3

# #{ imports

import rospy
import rosnode
import subprocess
import math
import os
import numpy as np
from math import atan2
from numpy import cos, sin
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import mpl_toolkits.mplot3d
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
import tf

# #} end of imports

# #{ HELPER FUNCTIONS

def boolToString(state):
    return "OK" if state else "FAILED"

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc. (https://stackoverflow.com/questions/13685386/
    matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to)

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def trajectoryToPathMsg(trajectory):
    path = Path()
    path.header.frame_id = "map"
    for k in range(len(trajectory.poses)):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position = Point(trajectory.poses[k].x, trajectory.poses[k].y, trajectory.poses[k].z)
        path.poses.append(pose_stamped)

    return path

def trajectoryToOdometryMsg(trajectory, trajectory_idx):
    odom = Odometry()
    odom.header.frame_id = "map"
    idx = min(trajectory_idx, len(trajectory.poses) - 1)
    odom.pose.pose.position = Point(trajectory.poses[idx].x, trajectory.poses[idx].y, trajectory.poses[idx].z)
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, trajectory.poses[idx].heading)
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]
    odom.twist.twist.linear = Vector3(trajectory.velocities[idx].x, trajectory.velocities[idx].y, trajectory.velocities[idx].z)
    odom.twist.twist.angular = Vector3(0.0, 0.0, trajectory.velocities[idx].heading)

    return odom

# #} end of HELPER FUNCTIONS

# #{ class Vector4d

class Vector4d:
    def __init__(self, x, y, z, heading):
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading

# #} end of class Vector4d

# #{ class DynamicConstraint

class DynamicConstraint:
    def __init__(self, speed, acceleration, jerk, snap):
        self.speed = speed
        self.acceleration = acceleration
        self.jerk = jerk
        self.snap = snap

# #} end of class DynamicConstraint

# #{ class Constraints

class Constraints:
    def __init__(self, horizontal, ascending, descending, heading):
        self.horizontal = horizontal
        self.ascending = ascending
        self.descending = descending
        self.heading = heading

# #} end of class Constraints

# #{ class Trajectory

class Trajectory:
    def __init__(self, waypoint_list, dt, trajectory_name):
        self.poses = waypoint_list
        self.dt = dt
        self.trajectory_name = trajectory_name
        self.velocities = self.getDerivation(self.poses, self.dt, True)
        self.accelerations = self.getDerivation(self.velocities, self.dt, False)
        self.jerks = self.getDerivation(self.accelerations, self.dt, False)
        self.snaps = self.getDerivation(self.jerks, self.dt, False)

    def getDerivation(self, vector, dt, wrap_heading):
        derivatives = []
        derivatives.append(Vector4d(0.0, 0.0, 0.0, 0.0))

        for k in range(1, len(vector)):
            dx = (vector[k].x - vector[k-1].x)/dt
            dy = (vector[k].y - vector[k-1].y)/dt
            dz = (vector[k].z - vector[k-1].z)/dt
            dheading = self.getHeadingDiff(vector[k-1].heading, vector[k].heading) if wrap_heading else (vector[k].heading - vector[k-1].heading)/dt
            derivatives.append(Vector4d(dx, dy, dz, dheading))

        return derivatives

    def getHeadingDiff(self, h1, h2):
        h1n = atan2(sin(h1), cos(h1))
        h2n = atan2(sin(h2), cos(h2))
        diff = h2n - h1n
        diffn = diff if abs(diff) < math.pi else np.sign(diff) * 2*math.pi - diff
        return diffn

# #} end of class Trajectory 

# #{ class TrajectoryChecker

class TrajectoryChecker:

    # #{ __init__()

    def __init__(self):

        rospy.init_node('trajectory_checker', anonymous=True)

        trajectory_folder = rospy.get_param('~trajectory_folder')
        trajectory_files = rospy.get_param('~trajectory_files')
        process_all = rospy.get_param('~process_all')
        visualization_trajectory = rospy.get_param('~visualization_python/trajectories')
        visualization_dynamics = rospy.get_param('~visualization_python/dynamics')
        visualization_mutual_dist = rospy.get_param('~visualization_python/mutual_distance')
        visualization_rviz = rospy.get_param('~visualization_rviz/use')
        playback_speed = rospy.get_param('~visualization_rviz/playback_speed')
        trajectory_dt = rospy.get_param('~trajectory_dt')
        minimum_mutual_distance = rospy.get_param('~min_mutual_distance')
        minimum_height = rospy.get_param('~safety_area/min_height')
        maximum_height = rospy.get_param('~safety_area/max_height')
        rviz_config = rospy.get_param('~rviz_config')
        self.print_info = rospy.get_param('~print_info')

        # #{ LOAD CONSTRAINTS

        constraint_type = rospy.get_param('~constraints')

        # ends if constraints cannot be loaded

        h_speed = rospy.get_param('~' + constraint_type + '/horizontal/speed')
        h_acc = rospy.get_param('~' + constraint_type + '/horizontal/acceleration')
        h_jerk = rospy.get_param('~' + constraint_type + '/horizontal/jerk')
        h_snap = rospy.get_param('~' + constraint_type + '/horizontal/snap')

        va_speed = rospy.get_param('~' + constraint_type + '/vertical/ascending/speed')
        va_acc = rospy.get_param('~' + constraint_type + '/vertical/ascending/acceleration')
        va_jerk = rospy.get_param('~' + constraint_type + '/vertical/ascending/jerk')
        va_snap = rospy.get_param('~' + constraint_type + '/vertical/ascending/snap')

        vd_speed = rospy.get_param('~' + constraint_type + '/vertical/descending/speed')
        vd_acc = rospy.get_param('~' + constraint_type + '/vertical/descending/acceleration')
        vd_jerk = rospy.get_param('~' + constraint_type + '/vertical/descending/jerk')
        vd_snap = rospy.get_param('~' + constraint_type + '/vertical/descending/snap')

        heading_speed = rospy.get_param('~' + constraint_type + '/heading/speed')
        heading_acc = rospy.get_param('~' + constraint_type + '/heading/acceleration')
        heading_jerk = rospy.get_param('~' + constraint_type + '/heading/jerk')
        heading_snap = rospy.get_param('~' + constraint_type + '/heading/snap')

        self.constraints = Constraints(DynamicConstraint(h_speed, h_acc, h_jerk, h_snap), DynamicConstraint(va_speed, va_acc, va_jerk, va_snap), DynamicConstraint(vd_speed, vd_acc, vd_jerk, vd_snap), DynamicConstraint(heading_speed, heading_acc, heading_jerk, heading_snap))

        # #} end of LOAD CONSTRAINTS


        # TODO: LOAD SAFETY AREA

        if process_all:
            trajectory_files = [f for f in os.listdir(trajectory_folder) if os.path.isfile(os.path.join(trajectory_folder, f))]

        rospy.loginfo("[TrajectoryChecker] Starting trajectory checker.")

        self.trajectories = self.loadTrajectories(trajectory_folder, trajectory_files, trajectory_dt)

        if len(self.trajectories) == len(trajectory_files):
            rospy.loginfo("[TrajectoryChecker] All trajectories loaded successfully. Number of trajectories = %lu.", len(self.trajectories))
        else:
            rospy.logwarn("[TrajectoryChecker] Only %lu out of %lu were loaded.", len(self.trajectories), len(trajectory_files))

        if len(self.trajectories) == 0:
            rospy.logwarn("[TrajectoryChecker] No valid trajectory loaded. Nothing to check.")
            return

        # initialize path and odometry publishers
        path_publishers = []
        odometry_publishers = []
        idx = 0;

        rospy.loginfo("[TrajectoryChecker] ---------- Trajectory indices (used in rviz visualization): ----------")
        for trajectory in self.trajectories:
            path_publishers.append(rospy.Publisher('/trajectory_checker/paths/path_' + str(idx), Path, queue_size=1))
            odometry_publishers.append(rospy.Publisher('/trajectory_checker/odometry/odom_' + str(idx), Odometry, queue_size=1))
            rospy.loginfo("[TrajectoryChecker] Trajectory %d: %s", idx, trajectory.trajectory_name)
            idx += 1


        mutual_distances = self.getMutualDistances(self.trajectories)

        mutual_dists_ok_list = self.checkMutualDistances(mutual_distances, minimum_mutual_distance)

        dynamic_constraints_ok_list = self.checkDynamicConstraints(self.trajectories, self.constraints)

        safety_area_ok_list = self.checkSafetyArea(self.trajectories, False, minimum_height, maximum_height)

        overall_status = True
        for k in range(len(self.trajectories)):
            rospy.loginfo("[TrajectoryChecker] ---------- Trajectory %s check list: ----------", self.trajectories[k].trajectory_name)
            trajectory_status = mutual_dists_ok_list[k] and dynamic_constraints_ok_list[k] and safety_area_ok_list[k]
            rospy.loginfo("[TrajectoryChecker] Overall status: %s (mutual distances: %s, dynamic constraints: %s, safety area: %s)", boolToString(trajectory_status), boolToString(mutual_dists_ok_list[k]), boolToString(dynamic_constraints_ok_list[k]), boolToString(safety_area_ok_list[k]))
            overall_status = overall_status and trajectory_status

        if overall_status:
            rospy.loginfo("[TrajectoryChecker] ---------- Overall status: OK, all checks passed. ----------")
        else:
            rospy.logerr("[TrajectoryChecker] ---------- Overall status: FAILED, some checks failed, see script's output for details. ----------")

        if (visualization_trajectory):
            self.plotPaths(self.trajectories)

        if (visualization_mutual_dist):
            self.plotMutualDistances(mutual_distances, trajectory_dt, minimum_mutual_distance)

        if (visualization_dynamics):
            self.plotDynamics(self.trajectories, self.constraints, trajectory_dt)

        if (visualization_rviz):
            rviz_proc = subprocess.Popen(['rviz', '-d', rviz_config], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.Rate(1.0).sleep()
            self.publishPaths(self.trajectories, path_publishers)

            rate = rospy.Rate(0.2)

            while not rospy.is_shutdown():
                self.publishPaths(self.trajectories, path_publishers)
                self.runTrajectoryPlayback(self.trajectories, odometry_publishers, playback_speed, trajectory_dt)
                rate.sleep()

    # #} end of __init__()

    # #{ loadTrajectories()

    def loadTrajectories(self, folder, filenames, dt):

        trajectories = []
        for filename in filenames:
            if os.path.exists(os.path.join(folder, filename)):
                trajectories.append(self.loadTrajectory(os.path.join(folder, filename), filename.rpartition('.')[0], dt))
            else:
                rospy.logerr('[TrajectoryChecker] Trajectory file %s not found. Excluding file from checking.', os.path.join(folder, filename))

        return trajectories

    # #} end of loadTrajectories()

    # #{ loadTrajectory()

    def loadTrajectory(self, filepath, trajectory_name, dt):
        f = open(filepath, 'r')
        lines = f.readlines()
        waypoints = []
        ext = filepath.split(".")[-1]
        ext = filepath.split(".")[-1]
        for line in lines:
            x, y, z, heading = line.split(',' if ',' in line else ' ')
            waypoints.append(Vector4d(float(x), float(y), float(z), float(heading)))

        return Trajectory(waypoints, dt, trajectory_name)

    # #} end of loadTrajectory()

    # #{ checkDynamicConstraints()

    def checkDynamicConstraints(self, trajectories, constraints):

        results = []

        for k in range(len(trajectories)):
            vels_xy = []
            accs_xy = []
            jerks_xy = []
            snaps_xy = []
            for m in range(len(trajectories[k].velocities)):
                vels_xy.append(np.sqrt(trajectories[k].velocities[m].x**2 + trajectories[k].velocities[m].y**2))
                accs_xy.append(np.sqrt(trajectories[k].accelerations[m].x**2 + trajectories[k].accelerations[m].y**2))
                jerks_xy.append(np.sqrt(trajectories[k].jerks[m].x**2 + trajectories[k].jerks[m].y**2))
                snaps_xy.append(np.sqrt(trajectories[k].snaps[m].x**2 + trajectories[k].snaps[m].y**2))

            vels_z = [vel.z for vel in trajectories[k].velocities]
            accs_z = [acc.z for acc in trajectories[k].accelerations]
            jerks_z = [jerk.z for jerk in trajectories[k].jerks]
            snaps_z = [snap.z for snap in trajectories[k].snaps]
            vels_heading = [vel.heading for vel in trajectories[k].velocities]
            accs_heading = [acc.heading for acc in trajectories[k].accelerations]
            jerks_heading = [jerk.heading for jerk in trajectories[k].jerks]
            snaps_heading = [snap.heading for snap in trajectories[k].snaps]

            # #{ LIMITING VALUES

            max_vel_xy = max(abs(np.array(vels_xy)))
            max_vel_desc = abs(min(np.array(vels_z)))
            max_vel_asc = abs(max(np.array(vels_z)))
            max_vel_heading = max(abs(np.array(vels_heading)))

            max_acc_xy = max(abs(np.array(accs_xy)))
            max_acc_desc = abs(min(np.array(accs_z)))
            max_acc_asc = abs(max(np.array(accs_z)))
            max_acc_heading = max(abs(np.array(accs_heading)))

            max_jerk_xy = max(abs(np.array(jerks_xy)))
            max_jerk_desc = abs(min(np.array(jerks_z)))
            max_jerk_asc = abs(max(np.array(jerks_z)))
            max_jerk_heading = max(abs(np.array(jerks_heading)))

            max_snap_xy = max(abs(np.array(snaps_xy)))
            max_snap_desc = abs(min(np.array(snaps_z)))
            max_snap_asc = abs(max(np.array(snaps_z)))
            max_snap_heading = max(abs(np.array(snaps_heading)))

            # #} end of LIMITING VALUES

            # #{ CONSTRAINTS CHECK

            # check constraints
            vel_xy_ok = max_vel_xy < constraints.horizontal.speed
            vel_desc_ok = max_vel_desc < constraints.descending.speed
            vel_asc_ok = max_vel_asc < constraints.ascending.speed
            vel_heading_ok = max_vel_heading < constraints.heading.speed

            acc_xy_ok = max_acc_xy < constraints.horizontal.acceleration
            acc_desc_ok = max_acc_desc < constraints.descending.acceleration
            acc_asc_ok = max_acc_asc < constraints.ascending.acceleration
            acc_heading_ok = max_acc_heading < constraints.heading.acceleration

            jerk_xy_ok = max_jerk_xy < constraints.horizontal.jerk
            jerk_desc_ok = max_jerk_desc < constraints.descending.jerk
            jerk_asc_ok = max_jerk_asc < constraints.ascending.jerk
            jerk_heading_ok = max_jerk_heading < constraints.heading.jerk

            snap_xy_ok = max_snap_xy < constraints.horizontal.snap
            snap_desc_ok = max_snap_desc < constraints.descending.snap
            snap_asc_ok = max_snap_asc < constraints.ascending.snap
            snap_heading_ok = max_snap_heading < constraints.heading.snap

            # #} end of CONSTRAINTS CHECK

            # #{ COMMAND LINE OUTPUTS

            rospy.loginfo("[TrajectoryChecker] ---------- Constraints check for trajectory %s: ----------", trajectories[k].trajectory_name)

            if not vel_xy_ok:
                rospy.logwarn("[TrajectoryChecker] Horizontal speed constraint violated. Maximum speed = %.2f m/s, limit = %.2f m/s.", max_vel_xy, constraints.horizontal.speed)

            if not vel_desc_ok:
                rospy.logwarn("[TrajectoryChecker] Descending speed constraint violated. Maximum speed = %.2f m/s, limit = %.2f m/s.", max_vel_desc, constraints.descending.speed)

            if not vel_asc_ok:
                rospy.logwarn("[TrajectoryChecker] Ascending speed constraint violated. Maximum speed = %.2f m/s, limit = %.2f m/s.", max_vel_asc, constraints.ascending.speed)

            if not vel_heading_ok:
                rospy.logwarn("[TrajectoryChecker] Heading speed constraint violated. Maximum speed = %.2f rad/s, limit = %.2f rad/s.", max_vel_heading, constraints.heading.speed)

            if not acc_xy_ok:
                rospy.logwarn("[TrajectoryChecker] Horizontal acceleration constraint violated. Maximum acceleration = %.2f m/s^2, limit = %.2f m/s^2.", max_acc_xy, constraints.horizontal.acceleration)

            if not acc_desc_ok:
                rospy.logwarn("[TrajectoryChecker] Descending acceleration constraint violated. Maximum acceleration = %.2f m/s^2, limit = %.2f m/s^2.", max_acc_desc, constraints.descending.acceleration)

            if not acc_asc_ok:
                rospy.logwarn("[TrajectoryChecker] Ascending acceleration constraint violated. Maximum acceleration = %.2f m/s^2, limit = %.2f m/s^2.", max_acc_asc, constraints.ascending.acceleration)

            if not acc_heading_ok:
                rospy.logwarn("[TrajectoryChecker] Heading acceleration constraint violated. Maximum acceleration = %.2f rad/s^2, limit = %.2f rad/s^2.", max_acc_heading, constraints.heading.acceleration)

            if not jerk_xy_ok:
                rospy.logwarn("[TrajectoryChecker] Horizontal jerk constraint violated. Maximum jerk = %.2f m/s^3, limit = %.2f m/s^3.", max_jerk_xy, constraints.horizontal.jerk)

            if not jerk_desc_ok:
                rospy.logwarn("[TrajectoryChecker] Descending jerk constraint violated. Maximum jerk = %.2f m/s^3, limit = %.2f m/s^3.", max_jerk_desc, constraints.descending.jerk)

            if not jerk_asc_ok:
                rospy.logwarn("[TrajectoryChecker] Ascending jerk constraint violated. Maximum jerk = %.2f m/s^3, limit = %.2f m/s^3.", max_jerk_asc, constraints.ascending.jerk)

            if not jerk_heading_ok:
                rospy.logwarn("[TrajectoryChecker] Heading jerk constraint violated. Maximum jerk = %.2f rad/s^3, limit = %.2f rad/s^3.", max_jerk_heading, constraints.heading.jerk)

            if not snap_xy_ok:
                rospy.logwarn("[TrajectoryChecker] Horizontal snap constraint violated. Maximum snap = %.2f m/s^4, limit = %.2f m/s^4.", max_snap_xy, constraints.horizontal.snap)

            if not snap_desc_ok:
                rospy.logwarn("[TrajectoryChecker] Descending snap constraint violated. Maximum snap = %.2f m/s^4, limit = %.2f m/s^4.", max_snap_desc, constraints.descending.snap)

            if not snap_asc_ok:
                rospy.logwarn("[TrajectoryChecker] Ascending snap constraint violated. Maximum snap = %.2f m/s^4, limit = %.2f m/s^4.", max_snap_asc, constraints.ascending.snap)

            if not snap_heading_ok:
                rospy.logwarn("[TrajectoryChecker] Heading snap constraint violated. Maximum snap = %.2f rad/s^4, limit = %.2f rad/s^4.", max_snap_heading, constraints.heading.snap)


            # #} end of COMMAN LINE OUTPUTS

            constraints_check_successful = vel_xy_ok and vel_asc_ok and vel_desc_ok and vel_heading_ok and acc_xy_ok and acc_asc_ok and acc_desc_ok and acc_heading_ok and jerk_xy_ok and jerk_asc_ok and jerk_desc_ok and jerk_heading_ok and snap_xy_ok and snap_asc_ok and snap_desc_ok and snap_heading_ok

            if constraints_check_successful:
                rospy.loginfo("[TrajectoryChecker] ##### Constraints for trajectory %s not violated. #####", trajectories[k].trajectory_name)
            else:
                rospy.logerr("[TrajectoryChecker] ##### FAILED: Constraints for trajectory %s violated. #####", trajectories[k].trajectory_name)

            if self.print_info:
                rospy.loginfo("[TrajectoryChecker] --------- Dynamics of trajectory %s: ----------", trajectories[k].trajectory_name)
                rospy.loginfo("[TrajectoryChecker] Max speed: horizontal = %.2f (m/s), descending = %.2f (m/s), ascending = %.2f (m/s), heading = %.2f (rad/s)", max_vel_xy, max_vel_desc, max_vel_asc, max_vel_heading)
                rospy.loginfo("[TrajectoryChecker] Max acceleration: horizontal = %.2f (m/s^2), descending = %.2f (m/s^2), ascending = %.2f (m/s^2), heading = %.2f (rad/s^2)", max_acc_xy, max_acc_desc, max_acc_asc, max_acc_heading)

            results.append(constraints_check_successful)

        return results

    # #} end of checkDynamicConstraints()

    # #{ checkSafetyArea()

    def checkSafetyArea(self, trajectories, safety_area, minimum_height, maximum_height):
        results = []

        for k in range(len(trajectories)):

            rospy.loginfo("[TrajectoryChecker] ---------- Min and max height check for trajectory %s: ----------", trajectories[k].trajectory_name)

            z_list = [pose.z for pose in trajectories[k].poses]

            z_min = min(np.array(z_list))
            z_max = max(np.array(z_list))

            result = True

            if z_min > minimum_height:
                rospy.loginfo("[TrajectoryChecker] Minimum height constraints for trajectory %s not violated.", trajectories[k].trajectory_name)
            else:
                rospy.logerr("[TrajectoryChecker] ##### FAILED: Minimum height constraints for trajectory %s violated. #####", trajectories[k].trajectory_name)
                result = False

            if z_max < maximum_height:
                rospy.loginfo("[TrajectoryChecker] Maximum height constraints for trajectory %s not violated.", trajectories[k].trajectory_name)
            else:
                rospy.logerr("[TrajectoryChecker] ##### FAILED: Maximum height constraints for trajectory %s violated. #####", trajectories[k].trajectory_name)
                result = False

            results.append(result)

            if self.print_info:
                rospy.loginfo("[TrajectoryChecker] Height limits of trajectory %s:", trajectories[k].trajectory_name)
                rospy.loginfo("[TrajectoryChecker] Min height = %.2f m, max height = %.2f m", z_min, z_max)

        return results

    # #} end of checkSafetyArea()

    # #{ checkMutualDistances()

    def checkMutualDistances(self, min_dists_list, min_dist_allowed):

        result = []

        if min_dists_list == []:
            result.append(True)
            return result 

        for k in range(len(min_dists_list)):

            rospy.loginfo("[TrajectoryChecker] ---------- Mutual distance check for trajectory %s: ----------", self.trajectories[k].trajectory_name)

            min_d = min(min_dists_list[k], key = lambda t: t[0])

            rospy.loginfo("[TrajectoryChecker] Minimum distance of trajectory %s from other trajectories is %.2f (closest trajectory: %s).", self.trajectories[k].trajectory_name, min_d[0], self.trajectories[min_d[1]].trajectory_name)

            if (min_d[0] < min_dist_allowed):
                rospy.logerr("[TrajectoryChecker] FAILED: Minimum distance of trajectory %s is below specified threshold (%.2f).", self.trajectories[k].trajectory_name, min_dist_allowed)
                result.append(False)
            else:
                result.append(True)

        return result

    # #} end of checkMutualDistances()

    # #{ getMutualDistances()

    def getMutualDistances(self, trajectories):

        min_dists_list = []

        if len(trajectories) < 2: 
            return min_dists_list
        
        for t in range(len(trajectories)):
            min_dists = []
            for k in range(len(trajectories[t].poses)):
                min_dist = 1e6
                min_idx = -1
                for t_r in range(len(trajectories)):
                    if t == t_r:
                        continue

                    idx = min(k, len(trajectories[t_r].poses) - 1)
                    dist = self.getTransitionPointDist(trajectories[t].poses[k], trajectories[t_r].poses[idx])
                    if dist < min_dist:
                        min_dist = dist
                        min_idx = t_r

                min_dists.append((min_dist, min_idx))

            min_dists_list.append(min_dists)

        return min_dists_list

    # #} end of getMutualDistances()

    # #{ getTransitionPointDist()

    def getTransitionPointDist(self, point1, point2):
        return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z -point2.z)**2)

    # #} end of getTransitionPointDist()

    # #{ plotDynamics()

    def plotDynamics(self, trajectories, constraints, dt):

        for k in range(len(trajectories)):
            vels_xy = []
            accs_xy = []
            jerks_xy = []
            snaps_xy = []
            for m in range(len(trajectories[k].velocities)):
                vels_xy.append(np.sqrt(trajectories[k].velocities[m].x**2 + trajectories[k].velocities[m].y**2))
                accs_xy.append(np.sqrt(trajectories[k].accelerations[m].x**2 + trajectories[k].accelerations[m].y**2))
                jerks_xy.append(np.sqrt(trajectories[k].jerks[m].x**2 + trajectories[k].jerks[m].y**2))
                snaps_xy.append(np.sqrt(trajectories[k].snaps[m].x**2 + trajectories[k].snaps[m].y**2))

            vels_z = [vel.z for vel in trajectories[k].velocities]
            accs_z = [acc.z for acc in trajectories[k].accelerations]
            jerks_z = [jerk.z for jerk in trajectories[k].jerks]
            snaps_z = [snap.z for snap in trajectories[k].snaps]
            vels_heading = [vel.heading for vel in trajectories[k].velocities]
            accs_heading = [acc.heading for acc in trajectories[k].accelerations]
            jerks_heading = [jerk.heading for jerk in trajectories[k].jerks]
            snaps_heading = [snap.heading for snap in trajectories[k].snaps]

            max_time = len(trajectories[k].poses) * dt

            fig, axs = plt.subplots(3, 4, sharex=False, sharey=False)
            fig.suptitle("Dynamics: " + trajectories[k].trajectory_name)
            axs[0, 0].grid(True, linestyle='-.')
            axs[0, 0].plot(np.arange(0, len(trajectories[k].poses))*dt, vels_xy, 'b')
            axs[0, 0].plot([0.0, max_time], [constraints.horizontal.speed, constraints.horizontal.speed], 'r')
            axs[0, 0].set_title("Speed horizontal (m/s)")

            axs[0, 1].grid(True, linestyle='-.')
            axs[0, 1].plot(np.arange(0, len(trajectories[k].poses))*dt, accs_xy, 'b')
            axs[0, 1].plot([0.0, max_time], [constraints.horizontal.acceleration, constraints.horizontal.acceleration], 'r')
            axs[0, 1].set_title("Acceleration horizontal (m/s^2)")

            axs[0, 2].grid(True, linestyle='-.')
            axs[0, 2].plot(np.arange(0, len(trajectories[k].poses))*dt, jerks_xy, 'b')
            axs[0, 2].plot([0.0, max_time], [constraints.horizontal.jerk, constraints.horizontal.jerk], 'r')
            axs[0, 2].set_title("Jerk horizontal (m/s^3)")

            axs[0, 3].grid(True, linestyle='-.')
            axs[0, 3].plot(np.arange(0, len(trajectories[k].poses))*dt, snaps_xy, 'b')
            axs[0, 3].plot([0.0, max_time], [constraints.horizontal.snap, constraints.horizontal.snap], 'r')
            axs[0, 3].set_title("Snap horizontal (m/s^4)")

            axs[1, 0].grid(True, linestyle='-.')
            axs[1, 0].plot(np.arange(0, len(trajectories[k].poses))*dt, vels_z, 'b')
            axs[1, 0].plot([0.0, max_time], [constraints.ascending.speed, constraints.ascending.speed], 'r')
            axs[1, 0].plot([0.0, max_time], [-constraints.descending.speed, -constraints.descending.speed], 'r')
            axs[1, 0].set_title("Speed vertical (m/s)")

            axs[1, 1].grid(True, linestyle='-.')
            axs[1, 1].plot(np.arange(0, len(trajectories[k].poses))*dt, accs_z, 'b')
            axs[1, 1].plot([0.0, max_time], [constraints.ascending.acceleration, constraints.ascending.acceleration], 'r')
            axs[1, 1].plot([0.0, max_time], [-constraints.descending.acceleration, -constraints.descending.acceleration], 'r')
            axs[1, 1].set_title("Acceleration vertical (m/s^2)")

            axs[1, 2].grid(True, linestyle='-.')
            axs[1, 2].plot(np.arange(0, len(trajectories[k].poses))*dt, jerks_z, 'b')
            axs[1, 2].plot([0.0, max_time], [constraints.ascending.jerk, constraints.ascending.jerk], 'r')
            axs[1, 2].plot([0.0, max_time], [-constraints.descending.jerk, -constraints.descending.jerk], 'r')
            axs[1, 2].set_title("Jerk vertical (m/s^3)")

            axs[1, 3].grid(True, linestyle='-.')
            axs[1, 3].plot(np.arange(0, len(trajectories[k].poses))*dt, snaps_z, 'b')
            axs[1, 3].plot([0.0, max_time], [constraints.ascending.snap, constraints.ascending.snap], 'r')
            axs[1, 3].plot([0.0, max_time], [-constraints.descending.snap, -constraints.descending.snap], 'r')
            axs[1, 3].set_title("Snap vertical (m/s^4)")

            axs[2, 0].grid(True, linestyle='-.')
            axs[2, 0].plot(np.arange(0, len(trajectories[k].poses))*dt, vels_heading, 'b')
            axs[2, 0].plot([0.0, max_time], [constraints.heading.speed, constraints.heading.speed], 'r')
            axs[2, 0].plot([0.0, max_time], [-constraints.heading.speed, -constraints.heading.speed], 'r')
            axs[2, 0].set_title("Speed heading (rad/s)")

            axs[2, 1].grid(True, linestyle='-.')
            axs[2, 1].plot(np.arange(0, len(trajectories[k].poses))*dt, accs_heading, 'b')
            axs[2, 1].plot([0.0, max_time], [constraints.heading.acceleration, constraints.heading.acceleration], 'r')
            axs[2, 1].plot([0.0, max_time], [-constraints.heading.acceleration, -constraints.heading.acceleration], 'r')
            axs[2, 1].set_title("Acceleration heading (rad/s^2)")

            axs[2, 2].grid(True, linestyle='-.')
            axs[2, 2].plot(np.arange(0, len(trajectories[k].poses))*dt, jerks_heading, 'b')
            axs[2, 2].plot([0.0, max_time], [constraints.heading.jerk, constraints.heading.jerk], 'r')
            axs[2, 2].plot([0.0, max_time], [-constraints.heading.jerk, -constraints.heading.jerk], 'r')
            axs[2, 2].set_title("Jerk heading (rad/s^3)")

            axs[2, 3].grid(True, linestyle='-.')
            axs[2, 3].plot(np.arange(0, len(trajectories[k].poses))*dt, snaps_heading, 'b')
            axs[2, 3].plot([0.0, max_time], [constraints.heading.snap, constraints.heading.snap], 'r')
            axs[2, 3].plot([0.0, max_time], [-constraints.heading.snap, -constraints.heading.snap], 'r')
            axs[2, 3].set_title("Snap heading (rad/s^4)")

            mng = plt.get_current_fig_manager()
            mng.resize(*mng.window.maxsize())
            plt.show()

    # #} end of plotDynamics()

    # #{ plotPaths()
    def plotPaths(self, trajectories):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_title("Trajectories")
        for trajectory in trajectories:
            xs = [pose.x for pose in trajectory.poses]
            ys = [pose.y for pose in trajectory.poses]
            zs = [pose.z for pose in trajectory.poses]
            ax.plot(xs, ys, zs)
            idx_middle = math.floor(len(xs)*0.05)
            ax.scatter([xs[0], xs[idx_middle]], [ys[0], ys[idx_middle]], [zs[0], zs[idx_middle]], marker='o')
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_zlabel("z (m)")

        set_axes_equal(ax)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

    # #} end of plotPaths()

    # #{ plotMutualDistances()

    def plotMutualDistances(self, min_dists_list, dt, minimum_mutual_distance):


        if len(min_dists_list) < 2:
            rospy.loginfo('[TrajectoryChecker] Single trajectory present. Mutual distances cannot be plotted.')
            return
        elif len(min_dists_list) == 2:
            n_rows = 1
            n_cols = 2
        elif len(min_dists_list) == 4:
            n_rows = 2
            n_cols = 2
        elif len(min_dists_list) <= 9:
            n_rows = math.ceil(len(min_dists_list) / 3.0)
            n_cols = 3
        else:
            n_rows = math.ceil(len(min_dists_list) / 4.0)
            n_cols = 4

        fig, axs = plt.subplots(n_rows, n_cols, sharex=False, sharey=False)
        fig.suptitle("Minimum mutual distance (m)")

        for k in range(len(min_dists_list)):
            row_idx = math.floor(k / n_cols)
            col_idx = k % n_cols
            max_time = len(min_dists_list[k])*dt
            if n_rows == 1:
                axs[col_idx].grid(True, linestyle='-.')
                axs[col_idx].plot(np.arange(0, len(min_dists_list[k]))*dt, [p[0] for p in min_dists_list[k]], 'b')
                axs[col_idx].plot([0.0, max_time], [minimum_mutual_distance, minimum_mutual_distance], 'r')
                axs[col_idx].set_title("Trajectory: " + self.trajectories[k].trajectory_name)
            else: 
                axs[row_idx, col_idx].grid(True, linestyle='-.')
                axs[row_idx, col_idx].plot(np.arange(0, len(min_dists_list[k]))*dt, [p[0] for p in min_dists_list[k]], 'b')
                axs[row_idx, col_idx].plot([0.0, max_time], [minimum_mutual_distance, minimum_mutual_distance], 'r')
                axs[row_idx, col_idx].set_title("Trajectory: " + self.trajectories[k].trajectory_name)

        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

    # #} end of plotMutualDistances()

    # #{ runTrajectoryPlayback()

    def runTrajectoryPlayback(self, trajectories, odometry_publishers, playback_speed, dt):
        rospy.loginfo_once("[TrajectoryChecker] Running trajectory playback.")
        playback_rate = rospy.Rate(playback_speed/dt)
        max_len = max(np.array([len(trajectory.poses) for trajectory in trajectories]))
        trajectory_idx = 0
        while trajectory_idx < max_len:
            self.publishOdometry(trajectories, odometry_publishers, trajectory_idx)
            trajectory_idx += 1
            playback_rate.sleep()

    # #} end of runTrajectoryPlayback()

    # #{ publishOdometry()

    def publishOdometry(self, trajectories, odometry_publishers, trajectory_idx):

        for k in range(len(trajectories)):
            odom_msg = trajectoryToOdometryMsg(trajectories[k], trajectory_idx)
            odometry_publishers[k].publish(odom_msg)

    # #} end of publishOdometry()

    # #{ publishPaths()

    def publishPaths(self, trajectories, path_publishers):
        rospy.loginfo_once("[TrajectoryChecker] Publishing paths.")
        for k in range(len(trajectories)):
            path_k = trajectoryToPathMsg(trajectories[k])
            path_publishers[k].publish(path_k)

    # #} end of publishPaths()

# #} end of TRAJECTORY CHECKER

if __name__ == '__main__':
    try:
        trajectory_checker = TrajectoryChecker()
    except rospy.ROSInterruptException:
        pass
