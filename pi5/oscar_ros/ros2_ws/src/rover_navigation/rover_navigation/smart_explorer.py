
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult

#Reuse the already working frontier detection and map geometry
from frontier_explorer import FrontierExplorer


class SmartExplorer(FrontierExplorer):
    def __init__(self):
        super().__init__()
        self.outletGoal = None

        #Future YOLO localization node publishes a map-frame approach pose
        self.create_subscription(
            PoseStamped, '/outlet/approach_pose', self.outletCallback, 10
        )

    def outletCallback(self, msg):
        if msg.header.frame_id != 'map':
            print('Ignoring outlet: approach pose must use map frame.')
            return
        self.outletGoal = msg
        print('Outlet approach target received.', flush=True)

    def knownCells(self):
        return sum(value >= 0 for value in self.currentMap.data)

    def makePose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    #Check actual Nav2 routes before accepting a frontier
    def reachableFrontier(self, robotX, robotY):
        start = self.makePose(robotX, robotY)
        checked = 0

        while checked < 30:
            pose, groupCount = self.selectGoal(robotX, robotY)
            if pose is None:
                break

            x, y = pose.pose.position.x, pose.pose.position.y
            checked += 1

            #Avoid choosing the same candidate again in this session
            self.attempted.append((x, y))

            try:
                path = self.getPath(start, pose, use_start=True)
            except Exception as error:
                print(f'Path check error: {error}', flush=True)
                path = None

            if path is not None and len(path.poses) >= 2:
                print(
                    f'Reachable frontier: {groupCount} groups; '
                    f'{checked} planner checks', flush=True
                )
                return pose

        print(f'No reachable frontier found ({checked} paths checked).')
        return None

    def updateMap(self, seconds=6):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

    def navigate(self, pose, interruptForOutlet=True):
        if not self.goToPose(pose):
            return 'REJECTED'

        deadline = time.monotonic() + 120
        while not self.isTaskComplete():
            if interruptForOutlet and self.outletGoal is not None:
                self.cancelTask()
                while not self.isTaskComplete():
                    pass
                return 'OUTLET_FOUND'

            if time.monotonic() > deadline:
                self.cancelTask()
                while not self.isTaskComplete():
                    pass
                return 'TIMEOUT'

        return self.getResult().name

    #Outlet target must already be localized; YOLO box is not enough
    def visitOutlet(self):
        target = self.outletGoal
        self.outletGoal = None
        if target is None:
            return False

        x, y = target.pose.position.x, target.pose.position.y
        position = self.robotPosition()

        if position is None or not self.isClear(x, y):
            print('Outlet approach rejected: unsafe or unknown map cell.')
            return False

        try:
            path = self.getPath(
                self.makePose(*position), target, use_start=True
            )
        except Exception as error:
            print(f'Outlet path check error: {error}')
            return False

        if path is None or len(path.poses) < 2:
            print('Outlet approach rejected: no valid path.')
            return False

        print(f'Navigating to outlet approach ({x:.2f}, {y:.2f})')
        result = self.navigate(target, interruptForOutlet=False)
        print(f'Outlet approach result: {result}')
        return result == TaskResult.SUCCEEDED.name


def main(args=None):
    rclpy.init(args=args)
    explorer = SmartExplorer()

    try:
        print('Waiting for Nav2 in SLAM mode...', flush=True)
        explorer.waitUntilNav2Active(localizer='robot_localization')

        #Wait for map and map->base_link transform
        deadline = time.monotonic() + 30
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(explorer, timeout_sec=0.2)
            if explorer.currentMap is not None and explorer.robotPosition():
                break
        else:
            print('STOP: Map or robot transform unavailable.')
            return

        startCells = explorer.knownCells()
        noFrontierRounds = 0
        stagnantGoals = 0
        goalCount = 0
        maxGoals = 20

        print(f'Initial known map cells: {startCells}', flush=True)

        while rclpy.ok() and goalCount < maxGoals:
            explorer.updateMap(0.5)

            #Outlet detections take priority over continued exploration
            if explorer.outletGoal is not None:
                if explorer.visitOutlet():
                    print('MISSION: OUTLET_APPROACH_REACHED')
                    return
                continue

            position = explorer.robotPosition()
            if position is None:
                print('STOP: Robot transform lost.')
                break

            goal = explorer.reachableFrontier(*position)
            if explorer.outletGoal is not None:
                continue

            if goal is None:
                noFrontierRounds += 1
                print(f'No reachable frontier: check {noFrontierRounds}/3')
                if noFrontierRounds >= 3:
                    print('STOP: NO_REACHABLE_FRONTIER')
                    print('This does not prove full room coverage.')
                    break
                explorer.updateMap(6.0)
                continue

            noFrontierRounds = 0
            goalCount += 1
            x, y = goal.pose.position.x, goal.pose.position.y
            before = explorer.knownCells()
            print(f'Goal {goalCount}: ({x:.2f}, {y:.2f})', flush=True)

            result = explorer.navigate(goal)
            print(f'Navigation result: {result}', flush=True)

            if result == 'OUTLET_FOUND':
                continue

            explorer.updateMap(6.0)
            after = explorer.knownCells()
            gained = after - before
            print(f'Map growth: {before} -> {after} ({gained:+d} cells)')

            if result == 'SUCCEEDED':
                stagnantGoals = stagnantGoals + 1 if gained < 20 else 0

            if stagnantGoals >= 4:
                print('STOP: MAP_GROWTH_STALLED')
                print('This does not prove full room coverage.')
                break

        if goalCount >= maxGoals:
            print('STOP: TEST_GOAL_LIMIT_REACHED')

        print(
            f'Summary: {goalCount} goals attempted, '
            f'{explorer.knownCells() - startCells:+d} known cells',
            flush=True
        )

    except KeyboardInterrupt:
        print('Mission interrupted.')
        explorer.cancelTask()
    finally:
        explorer.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
