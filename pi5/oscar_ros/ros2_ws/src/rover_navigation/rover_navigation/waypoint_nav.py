
import time
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


#Create a destination for Nav2
def createGoal(navigator, goalX, goalY):
    goalPose = PoseStamped()

    goalPose.header.frame_id = "map"
    goalPose.header.stamp = navigator.get_clock().now().to_msg()

    goalPose.pose.position.x = float(goalX)
    goalPose.pose.position.y = float(goalY)
    goalPose.pose.position.z = 0.0
    goalPose.pose.orientation.w = 1.0

    return goalPose


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    goalRunning = False

    #Set the destinations for the simulation
    waypoints = [
        (-1.0, -0.5),
        (-2.0, -0.5),
        (-1.0, -0.5)
    ]

    try:
        #Wait for Nav2 to become active
        print("Waiting for Nav2...", flush=True)
        navigator.waitUntilNav2Active()

        print(f"Starting navigation with {len(waypoints)} waypoints.")

        #Move through each destination
        for index, (goalX, goalY) in enumerate(waypoints, start=1):

            print(f"\nWaypoint {index}: X={goalX}, Y={goalY}")

            goalPose = createGoal(navigator, goalX, goalY)

            #Send the goal to Nav2
            goalAccepted = navigator.goToPose(goalPose)

            if not goalAccepted:
                print(f"Waypoint {index} was rejected.")
                break

            goalRunning = True
            startTime = time.monotonic()
            lastReport = startTime
            timedOut = False

            #Monitor the robot's progress
            while not navigator.isTaskComplete():

                currentTime = time.monotonic()
                elapsed = currentTime - startTime

                #Cancel if the robot takes too long
                if elapsed >= 180:
                    print("Navigation timed out.")
                    navigator.cancelTask()
                    timedOut = True
                    break

                #Display progress every five seconds
                if currentTime - lastReport >= 5:
                    feedback = navigator.getFeedback()

                    if feedback is not None:
                        print(
                            f"Distance remaining: "
                            f"{feedback.distance_remaining:.2f} meters"
                        )

                    lastReport = currentTime

            if timedOut:
                print("Stopping waypoint sequence.")
                break

            goalRunning = False
            result = navigator.getResult()

            #Determine whether the waypoint was reached
            if result == TaskResult.SUCCEEDED:
                print(f"Waypoint {index} reached successfully.")

            elif result == TaskResult.CANCELED:
                print(f"Waypoint {index} was canceled.")
                break

            else:
                print(f"Waypoint {index} failed.")
                print(f"ROS action status: {navigator.status}")
                break

        else:
            print("\nAll waypoints completed successfully!")

    except KeyboardInterrupt:
        print("\nNavigation interrupted.")

        if goalRunning:
            navigator.cancelTask()

    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
