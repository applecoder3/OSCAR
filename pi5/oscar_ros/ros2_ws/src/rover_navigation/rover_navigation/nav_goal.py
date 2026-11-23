
import time
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    goalAccepted = False

    #Set destination coordinates
    navigator.declare_parameter("goal_x", -1.0)
    navigator.declare_parameter("goal_y", -0.5)

    goalX = float(navigator.get_parameter("goal_x").value)
    goalY = float(navigator.get_parameter("goal_y").value)

    try:
        #Wait for Nav2 to become active
        print("Waiting for Nav2...", flush=True)
        navigator.waitUntilNav2Active()

        #Create the navigation goal
        goalPose = PoseStamped()
        goalPose.header.frame_id = "map"
        goalPose.header.stamp = navigator.get_clock().now().to_msg()

        goalPose.pose.position.x = goalX
        goalPose.pose.position.y = goalY
        goalPose.pose.position.z = 0.0

        #Set the robot's final orientation
        goalPose.pose.orientation.x = 0.0
        goalPose.pose.orientation.y = 0.0
        goalPose.pose.orientation.z = 0.0
        goalPose.pose.orientation.w = 1.0

        #Send the goal to Nav2
        print(f"Navigating to X={goalX}, Y={goalY}", flush=True)

        goalAccepted = navigator.goToPose(goalPose)

        print(f"Goal accepted: {goalAccepted}", flush=True)

        if not goalAccepted:
            print("ERROR: Nav2 rejected the navigation goal.")
            return

        #Monitor navigation progress
        startTime = time.monotonic()
        lastReport = 0.0
        timedOut = False

        while not navigator.isTaskComplete():

            elapsed = time.monotonic() - startTime

            #Cancel if navigation exceeds three minutes
            if elapsed >= 180:
                print("Navigation timeout. Canceling goal.")
                navigator.cancelTask()
                timedOut = True
                break

            #Read navigation feedback
            feedback = navigator.getFeedback()

            if feedback is not None and elapsed - lastReport >= 5:
                distance = feedback.distance_remaining
                recoveries = feedback.number_of_recoveries

                print(f"Distance remaining: {distance:.2f} meters")
                print(f"Recovery attempts: {recoveries}")

                lastReport = elapsed

            time.sleep(0.1)

        #Check the navigation result
        if timedOut:
            print("Navigation stopped because the time limit was reached.")
            return

        result = navigator.getResult()

        print(f"Final result: {result.name}")
        print(f"ROS action status: {navigator.status}")

        if result == TaskResult.SUCCEEDED:
            print("Destination reached successfully.")

        elif result == TaskResult.CANCELED:
            print("Navigation was canceled.")

        elif result == TaskResult.FAILED:
            print("Navigation failed after the goal was accepted.")

        else:
            print("Navigation returned an unknown result.")

        #Display additional error details when available
        if result != TaskResult.SUCCEEDED:
            resultFuture = navigator.result_future

            if resultFuture is not None and resultFuture.done():
                response = resultFuture.result()

                if response is not None:
                    details = response.result

                    errorCode = getattr(details, "error_code", None)
                    errorMessage = getattr(details, "error_msg", "")

                    if errorCode is not None:
                        print(f"Nav2 error code: {errorCode}")

                    if errorMessage:
                        print(f"Nav2 error message: {errorMessage}")

    except KeyboardInterrupt:
        print("Navigation interrupted by user.")

        if goalAccepted:
            navigator.cancelTask()

    except Exception as error:
        print(f"Navigation exception: {error}")

        if goalAccepted:
            navigator.cancelTask()

    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
