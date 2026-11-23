
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


class FrontierExplorer(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='frontier_explorer')
        self.currentMap = None
        self.attempted = []

        mapQos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(
            OccupancyGrid, '/map', self.mapCallback, mapQos
        )

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

    def mapCallback(self, msg):
        self.currentMap = msg

    #Get the robot's current position
    def robotPosition(self):
        try:
            transform = self.tfBuffer.lookup_transform(
                'map', 'base_link', Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except TransformException:
            return None

    #Convert map cells into real coordinates
    def gridToWorld(self, row, col):
        info = self.currentMap.info
        q = info.origin.orientation

        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        x = (col + 0.5) * info.resolution
        y = (row + 0.5) * info.resolution

        return (
            info.origin.position.x + x * math.cos(yaw) - y * math.sin(yaw),
            info.origin.position.y + x * math.sin(yaw) + y * math.cos(yaw)
        )

    def worldToGrid(self, x, y):
        info = self.currentMap.info
        q = info.origin.orientation

        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        dx = x - info.origin.position.x
        dy = y - info.origin.position.y

        col = math.floor(
            (dx * math.cos(yaw) + dy * math.sin(yaw)) / info.resolution
        )
        row = math.floor(
            (-dx * math.sin(yaw) + dy * math.cos(yaw)) / info.resolution
        )

        return int(row), int(col)

    #Reject goals too close to obstacles or unknown space
    def isClear(self, x, y, radius=0.20):
        msg = self.currentMap
        width, height = msg.info.width, msg.info.height

        row, col = self.worldToGrid(x, y)
        cells = max(1, math.ceil(radius / msg.info.resolution))

        for rr in range(row - cells, row + cells + 1):
            for cc in range(col - cells, col + cells + 1):
                if rr < 0 or rr >= height or cc < 0 or cc >= width:
                    return False

                value = msg.data[rr * width + cc]

                if value < 0 or value > 20:
                    return False

        return True

    #Find known free cells bordering unknown space
    def frontierGroups(self):
        msg = self.currentMap
        width, height = msg.info.width, msg.info.height
        grid = msg.data
        frontier = set()

        for row in range(1, height - 1):
            for col in range(1, width - 1):
                index = row * width + col

                if not 0 <= grid[index] <= 20:
                    continue

                neighbors = (-1, 1, -width, width)

                if any(grid[index + offset] == -1 for offset in neighbors):
                    frontier.add((row, col))

        #Group neighboring frontier cells
        groups = []

        while frontier:
            seed = frontier.pop()
            group = [seed]
            queue = deque([seed])

            while queue:
                row, col = queue.popleft()

                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        neighbor = (row + dr, col + dc)

                        if neighbor in frontier:
                            frontier.remove(neighbor)
                            queue.append(neighbor)
                            group.append(neighbor)

            if len(group) >= 5:
                groups.append(group)

        return groups

    #Select a safe destination near unexplored space
    #Select a destination near unexplored space
    def selectGoal(self, robotX, robotY):
        groups = self.frontierGroups()
        candidates = []

        tooClose = 0
        repeated = 0
        unsafe = 0

        for group in groups:
            step = max(1, len(group) // 30)

            for row, col in group[::step]:
                edgeX, edgeY = self.gridToWorld(row, col)

                dist = math.hypot(
                    edgeX - robotX,
                    edgeY - robotY
                )

                if dist < 0.35:
                    tooClose += 1
                    continue

                #Try several positions behind each frontier
                for retreat in (0.25, 0.40, 0.60, 0.80):
                    if dist <= retreat + 0.10:
                        continue

                    goalX = edgeX + retreat * (robotX - edgeX) / dist
                    goalY = edgeY + retreat * (robotY - edgeY) / dist

                    if any(
                        math.hypot(goalX - x, goalY - y) < 0.35
                        for x, y in self.attempted
                    ):
                        repeated += 1
                        continue

                    #Keep the original obstacle clearance
                    if not self.isClear(goalX, goalY, radius=0.20):
                        unsafe += 1
                        continue

                    yaw = math.atan2(
                        edgeY - goalY,
                        edgeX - goalX
                    )

                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()

                    pose.pose.position.x = goalX
                    pose.pose.position.y = goalY
                    pose.pose.orientation.z = math.sin(yaw / 2)
                    pose.pose.orientation.w = math.cos(yaw / 2)

                    distance = math.hypot(
                        goalX - robotX,
                        goalY - robotY
                    )

                    candidates.append((distance, pose))
                    break

        print(
            f"Frontier analysis: {len(groups)} groups, "
            f"{len(candidates)} candidate goals"
        )
        print(
            f"Rejected: too close={tooClose}, "
            f"previously attempted={repeated}, "
            f"unsafe={unsafe}"
        )

        if not candidates:
            return None, len(groups)

        #Prefer the nearest acceptable destination
        candidates.sort(key=lambda item: item[0])

        return candidates[0][1], len(groups)


def main(args=None):
    rclpy.init(args=args)

    navigator = FrontierExplorer()
    navigating = False

    try:
        print("Waiting for Nav2 in SLAM mode...", flush=True)

        #SLAM replaces AMCL, so skip the AMCL-specific wait
        navigator.waitUntilNav2Active(localizer='robot_localization')

        #Wait for the map and robot position
        deadline = time.monotonic() + 30

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.2)

            if (
                navigator.currentMap is not None
                and navigator.robotPosition() is not None
            ):
                break
        else:
            print("No map or robot transform. Exploration not started.")
            return

        #Limit the first test to six destinations
        maxGoals = 6

        for index in range(maxGoals):
            rclpy.spin_once(navigator, timeout_sec=0.2)

            position = navigator.robotPosition()

            if position is None:
                print("Lost robot position. Stopping.")
                break

            goalPose, groupCount = navigator.selectGoal(*position)

            if goalPose is None:
                print(
                    f"No safe new frontier goals found "
                    f"({groupCount} groups). Stopping."
                )
                break

            x = goalPose.pose.position.x
            y = goalPose.pose.position.y
            navigator.attempted.append((x, y))

            print(
                f"Exploration goal {index + 1}: "
                f"({x:.2f}, {y:.2f}), "
                f"frontier groups: {groupCount}",
                flush=True
            )

            #Send destination to Nav2
            if not navigator.goToPose(goalPose):
                print("Goal rejected. Trying another frontier.")
                continue

            navigating = True
            startTime = time.monotonic()
            timedOut = False

            #Monitor the navigation task
            while not navigator.isTaskComplete():
                if time.monotonic() - startTime > 120:
                    print("Goal timed out. Canceling.")
                    navigator.cancelTask()
                    timedOut = True
                    break

            navigating = False

            if timedOut:
                continue

            result = navigator.getResult()
            print(f"Goal result: {result.name}", flush=True)

            if result == TaskResult.SUCCEEDED:
                rclpy.spin_once(navigator, timeout_sec=1.0)

        print("Exploration session finished.")

    except KeyboardInterrupt:
        print("Exploration stopped by user.")

        if navigating:
            navigator.cancelTask()

    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
