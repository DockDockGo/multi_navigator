from enum import Enum
import time
from random import randint

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import AssistedTeleop, BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import (
    # FollowPath,
    # FollowWaypoints,
    # NavigateThroughPoses,
    NavigateToPose,
)

# from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class MultiNavigator(Node):
    def __init__(self, namespaces=[], node_name="multi_navigator"):
        super().__init__(node_name=node_name)

        self.goal_handle = {}
        self.result_future = {}
        self.feedback = None
        self.status = {}
        self.namespaces = namespaces
        self.time_taken = []
        self.nav_to_pose_clients = {}
        self.ENVIRONMENT = os.environ.get("MAP_NAME", "svd_demo")
        self.past_poses = []

        # for namespace in namespaces:
        #     self.nav_to_pose_clients[namespace] = ActionClient(
        #         self, NavigateToPose, "/" + namespace + "/navigate_to_pose"
        #     )

        # self.dt = {}
        # self.compute_path_to_pose_clients = {}
        # for namespace in namespaces:
        #     self.compute_path_to_pose_clients[namespace] = ActionClient(
        #         self, ComputePathToPose, "/" + namespace + "/compute_path_to_pose"
        #     )

        self.send_goal_publisher = {}
        for namespace in namespaces:
            self.send_goal_publisher[namespace] = self.create_publisher(
                PoseStamped, "/" + namespace + "/cbs_path/goal_pose", 10
            )

        # self.pose_list = self.init_pose_config_workspace_0()
        self.pose_list = self.init_pose_config()

        # timer_period = 60  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_pose_config(self):
        pose_list = []

        if self.ENVIRONMENT == "neo_track1":
            initial_pose = (5.0, 6.0)
            for i in range(0, 6):
                for j in range(0, 8):
                    pose_list.append(
                        (initial_pose[0] - j * 2.0, initial_pose[1] - i * 2.0)
                    )

        if self.ENVIRONMENT == "aws":
            initial_pose = (12.5, 2.0)
            for i in range(0, 6):
                for j in range(0, 3):
                    pose_list.append(
                        (initial_pose[0] - i * 2.0, initial_pose[1] - j * 2.0)
                    )

            pose_list.append((0.0, -3.0))
            pose_list.append((-3.0, -3.0))

            initial_pose = (12.5, -7.0)

            for i in range(0, 8):
                pose_list.append((initial_pose[0] - i * 2.0, initial_pose[1]))

        if self.ENVIRONMENT == "neo_workshop":
            initial_pose = (5.0, 3.0)
            for j in range(0, 4):
                pose_list.append((initial_pose[0], initial_pose[1] - 2.0 * j))
            for j in range(0, 5):
                pose_list.append((initial_pose[0] - 2.0 * j, initial_pose[1]))
            for j in range(0, 5):
                pose_list.append((initial_pose[0] - 2.0 * j, initial_pose[1] - 6.0))

            initial_pose = (-1.5, -6.0)

            for i in range(0, 2):
                for j in range(0, 2):
                    pose_list.append(
                        (initial_pose[0] - 2.5 * i, initial_pose[1] - 2.0 * j)
                    )

        if self.ENVIRONMENT == "workspace_0":
            initial_pose = (-3.0, 1.5)
            for j in range(0, 3):
                pose_list.append((initial_pose[0] + 0.5, initial_pose[1] - 1.5 * j))
            for j in range(0, 3):
                pose_list.append((initial_pose[0] + 3, initial_pose[1] - 1.5 * j))
            for j in range(0, 3):
                pose_list.append((initial_pose[0] + 6.5, initial_pose[1] - 1.5 * j))

            for i in range(0, 2):
                pose_list.append((6.0, -1.5 * i))

            # for i in range(0, 2):
            pose_list.append((-5, -3))

        if self.ENVIRONMENT == "svd_demo":
            # pose_list.append((-1.5, 1.5))
            # pose_list.append((5.5, -0.5))
            pose_list.append((2.5, 0.0))

            pose_list.append((4.0, 0.0))

            pose_list.append((2.5, -2.0))

            pose_list.append((4.0, -2.0))
            # pose_list.append((5.5, -0.5))
            pose_list.append((-2.0, -2.0))

            pose_list.append((1.0, -2.0))

            pose_list.append((6.0, -2.0))

            # # -----------------------
            # pose_list.append((-1.5, 1.5))
            # pose_list.append((-0.5, 1.5))

            # pose_list.append((5.5, 2.0))
            # pose_list.append((5.5, 1.0))
            # pose_list.append((5.5, 0.0))
            # pose_list.append((5.5, -1.0))
            # pose_list.append((5.5, -2.0))

            # pose_list.append((0.0, 0.0))
            # pose_list.append((1.0, 0.0))
            # pose_list.append((2.0, 0.0))
            # pose_list.append((3.0, 0.1))
            # pose_list.append((4.0, 0.0))

            # pose_list.append((0.0, -1.3))
            # pose_list.append((5.0, -1.3))
            # pose_list.append((6.0, -1.3))
            # pose_list.append((7.0, -1.3))
            # pose_list.append((8.0, -1.3))
            # pose_list.append((9.0, -1.3))
            # pose_list.append((10.0, -1.3))

            # pose_list.append((0.0, -2.2))
            # pose_list.append((1.0, -2.2))
            # pose_list.append((2.0, -2.2))
            # pose_list.append((3.0, -2.2))
            # pose_list.append((4.0, -2.2))
            # pose_list.append((5.0, -2.2))
            # pose_list.append((6.0, -2.2))
            # pose_list.append((7.0, -2.2))
            # pose_list.append((8.0, -2.2))
            # pose_list.append((9.0, -2.2))
            # pose_list.append((10.0, -2.2))

        return pose_list

    def timer_callback(self):
        # This function calls goToPose for every namespace
        #
        for namespace in self.namespaces:
            pose = self.computeRandomPoses(namespace)
            self.goToPose(pose, namespace)

    def computeRandomPoses(self, namespace):
        self.info("Computing random pose for namespace " + namespace)
        print("len of pose list = ", len(self.pose_list))

        pose = self.pose_list[randint(0, len(self.pose_list) - 1)]
        if len(self.past_poses) == 4:
            self.past_poses = []
        while pose in self.past_poses:
            self.debug("finding new pose")
            pose = self.pose_list[randint(0, len(self.pose_list) - 1)]

        self.past_poses.append(pose)

        self.info("pose for " + namespace + " is " + str(pose) + "...")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = namespace
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0] * 1.0
        goal_pose.pose.position.y = pose[1] * 1.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        return goal_pose

    def goToPose(self, pose, namespace, behavior_tree=""):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")

        # while not self.nav_to_pose_clients[namespace].wait_for_server(timeout_sec=1.0):
        #     self.info("'NavigateToPose' action server not available, waiting...")

        # self.cancelTask(namespace)
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = pose
        # goal_msg.behavior_tree = behavior_tree

        pose.header.frame_id = namespace

        self.info(
            "Navigating to goal for namespace "
            + namespace
            + " : "
            + str(pose.pose.position.x)
            + " "
            + str(pose.pose.position.y)
            + "..."
        )

        self.send_goal_publisher[namespace].publish(pose)

        # # send_goal_future = self.nav_to_pose_clients[namespace].send_goal_async(
        # #     goal_msg, self._feedbackCallback
        # # )
        # # rclpy.spin_until_future_complete(self, send_goal_future)
        # # self.goal_handle[namespace] = send_goal_future.result()

        # # if not self.goal_handle[namespace].accepted:
        # #     self.error(
        # #         "Goal to "
        # #         + str(pose.pose.position.x)
        # #         + " "
        # #         + str(pose.pose.position.y)
        # #         + " was rejected!"
        # #     )
        # #     return False

        # self.result_future[namespace] = self.goal_handle[namespace].get_result_async()
        return True

    def benchmark_callback(self):
        # This function calls computePathCb for every namespace
        past_poses = []
        for namespace in self.namespaces:
            pose = self.computeRandomPoses(past_poses, namespace)
            self.dt[namespace] = self.get_clock().now()
            path = self.computePathCb(pose, namespace)
            # self.info(path)

    def computePathCb(self, goal_pose, namespace):
        self.debug("Waiting for 'ComputePathToPose' action server")

        while not self.compute_path_to_pose_clients[namespace].wait_for_server(
            timeout_sec=1.0
        ):
            self.info("'ComputePathToPose' action server not available, waiting...")

        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.position.z = 0.0
        start.pose.orientation.w = 0.0

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal_pose
        goal_msg.planner_id = "GridBased"
        goal_msg.use_start = False

        self.info(
            "Computing path to goal for namespace "
            + namespace
            + " : "
            + str(goal_pose.pose.position.x)
            + " "
            + str(goal_pose.pose.position.y)
            + "..."
        )
        send_goal_future = self.compute_path_to_pose_clients[namespace].send_goal_async(
            goal_msg
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle[namespace] = send_goal_future.result()
        if not self.goal_handle[namespace].accepted:
            self.error(
                "Goal to "
                + str(goal_pose.pose.position.x)
                + " "
                + str(goal_pose.pose.position.y)
                + " was rejected!"
            )
            return None

        _result_future = self.goal_handle[namespace].get_result_async()
        rclpy.spin_until_future_complete(self, _result_future)
        self.status = _result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f"Getting path failed with status code: {self.status}")
            return None
        else:
            self.time_taken.append(
                (self.get_clock().now() - self.dt[namespace]).nanoseconds / 1e6
            )
            self.info("Path computed for namespace " + str(self.time_taken[-1]))  # ms

        # self.warn("here")

        return _result_future.result().result

        # self.warn("here2")
        # self.result_future[namespace] = self.goal_handle[namespace].get_result_async()
        # self.result_future[namespace].add_done_callback(
        #     lambda future: self._pathCallback(future, namespace)
        # )
        # self.warn("here2")
        # # time.sleep(10)
        # rclpy.spin_until_future_complete(self, self.result_future[namespace])
        # self.warn("here3")
        # self.status[namespace] = self.result_future[namespace].result().status
        # if self.status[namespace] != GoalStatus.STATUS_SUCCEEDED:
        #     self.warn(f"Getting path failed with status code: {self.status[namespace]}")
        #     return None
        # self.warn("here4")
        # self.info(
        #     "Path computed for namespace "
        #     + str(self.get_clock().now() - self.dt[namespace])
        # )

        # return self.result_future[namespace].result().result

        # if not self.goal_handle[namespace].accepted:
        #     self.error(
        #         "Goal to "
        #         + str(goal_pose.pose.position.x)
        #         + " "
        #         + str(goal_pose.pose.position.y)
        #         + " was rejected!"
        #     )
        #     return False

        # self.result_future[namespace] = self.goal_handle[namespace].get_result_async()

        # self.result_future[namespace].add_done_callback(
        #     lambda future: self._pathCallback(future, namespace)
        # )

    # def _pathCallback(self, future, namespace):
    #     self.info(
    #         "Path computed for namespace "
    #         + str(self.dt[namespace] - self.get_clock().now())
    #     )

    def destroyNode(self):
        self.destroy_node()

    def destroy_node(self):
        for namespace in self.namespaces:
            self.nav_to_pose_clients[namespace].destroy()
            self.compute_path_to_pose_clients[namespace].destroy()

        # self.follow_waypoints_client.destroy()
        super().destroy_node()

    # def followWaypoints(self, poses):
    #     """Send a `FollowWaypoints` action request."""
    #     self.debug("Waiting for 'FollowWaypoints' action server")
    #     while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'FollowWaypoints' action server not available, waiting...")

    #     goal_msg = FollowWaypoints.Goal()
    #     goal_msg.poses = poses

    #     self.info(f"Following {len(goal_msg.poses)} goals....")
    #     send_goal_future = self.follow_waypoints_client.send_goal_async(
    #         goal_msg, self._feedbackCallback
    #     )
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error(f"Following {len(poses)} waypoints request was rejected!")
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    # def spin(self, spin_dist=1.57, time_allowance=10):
    #     self.debug("Waiting for 'Spin' action server")
    #     while not self.spin_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'Spin' action server not available, waiting...")
    #     goal_msg = Spin.Goal()
    #     goal_msg.target_yaw = spin_dist
    #     goal_msg.time_allowance = Duration(sec=time_allowance)

    #     self.info(f"Spinning to angle {goal_msg.target_yaw}....")
    #     send_goal_future = self.spin_client.send_goal_async(
    #         goal_msg, self._feedbackCallback
    #     )
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Spin request was rejected!")
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    # def backup(self, backup_dist=0.15, backup_speed=0.025, time_allowance=10):
    #     self.debug("Waiting for 'Backup' action server")
    #     while not self.backup_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'Backup' action server not available, waiting...")
    #     goal_msg = BackUp.Goal()
    #     goal_msg.target = Point(x=float(backup_dist))
    #     goal_msg.speed = backup_speed
    #     goal_msg.time_allowance = Duration(sec=time_allowance)

    #     self.info(f"Backing up {goal_msg.target.x} m at {goal_msg.speed} m/s....")
    #     send_goal_future = self.backup_client.send_goal_async(
    #         goal_msg, self._feedbackCallback
    #     )
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Backup request was rejected!")
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    # def assistedTeleop(self, time_allowance=30):
    #     self.debug("Wainting for 'assisted_teleop' action server")
    #     while not self.assisted_teleop_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'assisted_teleop' action server not available, waiting...")
    #     goal_msg = AssistedTeleop.Goal()
    #     goal_msg.time_allowance = Duration(sec=time_allowance)

    #     self.info("Running 'assisted_teleop'....")
    #     send_goal_future = self.assisted_teleop_client.send_goal_async(
    #         goal_msg, self._feedbackCallback
    #     )
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Assisted Teleop request was rejected!")
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    # def followPath(self, path, controller_id="", goal_checker_id=""):
    #     """Send a `FollowPath` action request."""
    #     self.debug("Waiting for 'FollowPath' action server")
    #     while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'FollowPath' action server not available, waiting...")

    #     goal_msg = FollowPath.Goal()
    #     goal_msg.path = path
    #     goal_msg.controller_id = controller_id
    #     goal_msg.goal_checker_id = goal_checker_id

    #     self.info("Executing path...")
    #     send_goal_future = self.follow_path_client.send_goal_async(
    #         goal_msg, self._feedbackCallback
    #     )
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Follow path was rejected!")
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    def cancelTask(self, namespace=""):
        """Cancel pending task request of any type."""
        self.info("Canceling current task.")
        if namespace != "" and self.result_future[namespace]:
            future = self.goal_handle[namespace].cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self, namespace=""):
        """Check if the task request of any type is complete yet."""
        if namespace != "" and self.result_future[namespace]:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if namespace != "" and self.result_future[namespace].result():
            self.status[namespace] = self.result_future[namespace].result().status
            if self.status[namespace] != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f"Task with failed with status code: {self.status}")
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug("Task succeeded!")
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self, namespace=""):
        """Get the pending action result message."""
        if self.status[namespace] == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status[namespace] == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status[namespace] == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    # def waitUntilNav2Active(self, navigator="bt_navigator", localizer="amcl"):
    #     """Block until the full navigation system is up and running."""
    #     self._waitForNodeToActivate(localizer)
    #     # if localizer == "amcl":
    #     #     self._waitForInitialPose()
    #     self._waitForNodeToActivate(navigator)
    #     self.info("Nav2 is ready for use!")
    #     return

    # def _getPathImpl(self, start, goal, planner_id="", use_start=False):
    #     """
    #     Send a `ComputePathToPose` action request.
    #     Internal implementation to get the full result, not just the path.
    #     """
    #     self.debug("Waiting for 'ComputePathToPose' action server")
    #     while not self.compute_path_to_pose_clients.wait_for_server(timeout_sec=1.0):
    #         self.info("'ComputePathToPose' action server not available, waiting...")

    #     goal_msg = ComputePathToPose.Goal()
    #     goal_msg.start = start
    #     goal_msg.goal = goal
    #     goal_msg.planner_id = planner_id
    #     goal_msg.use_start = use_start

    #     self.info("Getting path...")
    #     send_goal_future = self.compute_path_to_pose_clients.send_goal_async(goal_msg)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Get path was rejected!")
    #         return None

    #     self.result_future = self.goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, self.result_future)
    #     self.status = self.result_future.result().status

    #     return self.result_future.result().result

    # def getPath(self, start, goal, planner_id="", use_start=False):
    #     """Send a `ComputePathToPose` action request."""
    #     rtn = self._getPathImpl(start, goal, planner_id="", use_start=False)

    #     if self.status != GoalStatus.STATUS_SUCCEEDED:
    #         self.warn(f"Getting path failed with status code: {self.status}")
    #         return None

    #     if not rtn:
    #         return None
    #     else:
    #         return rtn.path

    # def _getPathThroughPosesImpl(self, start, goals, planner_id="", use_start=False):
    #     """
    #     Send a `ComputePathThroughPoses` action request.
    #     Internal implementation to get the full result, not just the path.
    #     """
    #     self.debug("Waiting for 'ComputePathThroughPoses' action server")
    #     while not self.compute_path_through_poses_client.wait_for_server(
    #         timeout_sec=1.0
    #     ):
    #         self.info(
    #             "'ComputePathThroughPoses' action server not available, waiting..."
    #         )

    #     goal_msg = ComputePathThroughPoses.Goal()
    #     goal_msg.start = start
    #     goal_msg.goals = goals
    #     goal_msg.planner_id = planner_id
    #     goal_msg.use_start = use_start

    #     self.info("Getting path...")
    #     send_goal_future = self.compute_path_through_poses_client.send_goal_async(
    #         goal_msg
    #     )
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Get path was rejected!")
    #         return None

    #     self.result_future = self.goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, self.result_future)
    #     self.status = self.result_future.result().status

    #     return self.result_future.result().result

    # def getPathThroughPoses(self, start, goals, planner_id="", use_start=False):
    #     """Send a `ComputePathThroughPoses` action request."""
    #     rtn = self.__getPathThroughPosesImpl(
    #         start, goals, planner_id="", use_start=False
    #     )

    #     if self.status != GoalStatus.STATUS_SUCCEEDED:
    #         self.warn(f"Getting path failed with status code: {self.status}")
    #         return None

    #     if not rtn:
    #         return None
    #     else:
    #         return rtn.path

    # def _smoothPathImpl(
    #     self, path, smoother_id="", max_duration=2.0, check_for_collision=False
    # ):
    #     """
    #     Send a `SmoothPath` action request.
    #     Internal implementation to get the full result, not just the path.
    #     """
    #     self.debug("Waiting for 'SmoothPath' action server")
    #     while not self.smoother_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'SmoothPath' action server not available, waiting...")

    #     goal_msg = SmoothPath.Goal()
    #     goal_msg.path = path
    #     goal_msg.max_smoothing_duration = rclpyDuration(seconds=max_duration).to_msg()
    #     goal_msg.smoother_id = smoother_id
    #     goal_msg.check_for_collisions = check_for_collision

    #     self.info("Smoothing path...")
    #     send_goal_future = self.smoother_client.send_goal_async(goal_msg)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error("Smooth path was rejected!")
    #         return None

    #     self.result_future = self.goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, self.result_future)
    #     self.status = self.result_future.result().status

    #     return self.result_future.result().result

    # def smoothPath(
    #     self, path, smoother_id="", max_duration=2.0, check_for_collision=False
    # ):
    #     """Send a `SmoothPath` action request."""
    #     rtn = self._smoothPathImpl(
    #         path, smoother_id="", max_duration=2.0, check_for_collision=False
    #     )

    #     if self.status != GoalStatus.STATUS_SUCCEEDED:
    #         self.warn(f"Getting path failed with status code: {self.status}")
    #         return None

    #     if not rtn:
    #         return None
    #     else:
    #         return rtn.path

    # def changeMap(self, map_filepath):
    #     """Change the current static map in the map server."""
    #     while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
    #         self.info("change map service not available, waiting...")
    #     req = LoadMap.Request()
    #     req.map_url = map_filepath
    #     future = self.change_maps_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     status = future.result().result
    #     if status != LoadMap.Response().RESULT_SUCCESS:
    #         self.error("Change map request failed!")
    #     else:
    #         self.info("Change map request was successful!")
    #     return

    # def clearAllCostmaps(self):
    #     """Clear all costmaps."""
    #     self.clearLocalCostmap()
    #     self.clearGlobalCostmap()
    #     return

    # def clearLocalCostmap(self):
    #     """Clear local costmap."""
    #     while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
    #         self.info("Clear local costmaps service not available, waiting...")
    #     req = ClearEntireCostmap.Request()
    #     future = self.clear_costmap_local_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return

    # def clearGlobalCostmap(self):
    #     """Clear global costmap."""
    #     while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
    #         self.info("Clear global costmaps service not available, waiting...")
    #     req = ClearEntireCostmap.Request()
    #     future = self.clear_costmap_global_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return

    # def getGlobalCostmap(self):
    #     """Get the global costmap."""
    #     while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
    #         self.info("Get global costmaps service not available, waiting...")
    #     req = GetCostmap.Request()
    #     future = self.get_costmap_global_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().map

    # def getLocalCostmap(self):
    #     """Get the local costmap."""
    #     while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
    #         self.info("Get local costmaps service not available, waiting...")
    #     req = GetCostmap.Request()
    #     future = self.get_costmap_local_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().map

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info("Starting up lifecycle nodes based on lifecycle_manager.")
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
                self.info(f"Starting up {srv_name}")
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f"{srv_name} service not available, waiting...")
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info("Nav2 is ready for use!")
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info("Shutting down lifecycle nodes based on lifecycle_manager.")
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == "nav2_msgs/srv/ManageLifecycleNodes":
                self.info(f"Shutting down {srv_name}")
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f"{srv_name} service not available, waiting...")
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f"Waiting for {node_name} to become active..")
        node_service = f"{node_name}/get_state"
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            self.debug(f"Getting {node_name} state...")
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f"Result of get_state: {state}")
            time.sleep(2)
        return

    # def _waitForInitialPose(self):
    # while not self.initial_pose_received:
    #     self.info("Setting initial pose")
    #     self._setInitialPose()
    #     self.info("Waiting for amcl_pose to be received")
    #     rclpy.spin_once(self, timeout_sec=1.0)
    # return

    # def _amclPoseCallback(self, msg):
    #     self.debug("Received amcl pose")
    #     self.initial_pose_received = True
    #     return

    def _feedbackCallback(self, msg):
        self.debug("Received action feedback message")
        self.feedback = msg.feedback
        return

    # def _setInitialPose(self):
    #     msg = PoseWithCovarianceStamped()
    #     msg.pose.pose = self.initial_pose.pose
    #     msg.header.frame_id = self.initial_pose.header.frame_id
    #     msg.header.stamp = self.initial_pose.header.stamp
    #     self.info("Publishing Initial Pose")
    #     self.initial_pose_pub.publish(msg)
    #     return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(args=None):
    rclpy.init(args=args)

    namespaces = []
    num_robots = os.environ.get("Number_of_Robots", "2")

    for i in range(int(num_robots)):
        namespaces.append("robot" + str(i))

    multi_navigator = MultiNavigator(namespaces)

    multi_navigator.info("Starting demo!")
    # Spin in a separate thread
    # thread = threading.Thread(target=rclpy.spin, args=(multi_navigator,), daemon=True)
    # thread.start()

    # rate = multi_navigator.create_rate(0.01666)

    # multi_navigator.run_demo()
    try:
        while rclpy.ok():
            multi_navigator.timer_callback()

            # multi_navigator.benchmark_callback()
            # if multi_navigator.time_taken:
            #     print(
            #         "Average time taken = "
            #         + str(
            #             sum(multi_navigator.time_taken)
            #             / len(multi_navigator.time_taken)
            #         )
            #         + " ms seconds"
            #     )

            #     print(
            #         "Rate "
            #         + str(
            #             1000.0
            #             / (
            #                 sum(multi_navigator.time_taken)
            #                 / len(multi_navigator.time_taken)
            #             )
            #         )
            #         + "Hz"
            #     )
            time.sleep(45)
    except KeyboardInterrupt:
        pass

    multi_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
