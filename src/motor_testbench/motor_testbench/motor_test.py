import rclpy
from rclpy.executors import ExternalShutdownException
import threading
import sys
from motor_testbench.motor_commander import MotorCommander, loopWait

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

def main():
    rclpy.init(args=sys.argv)
    # In rospy callbacks are always called in background threads.
    # Spin the executor in another thread for similar behavior in ROS 2.
    t = threading.Thread(target=spin_in_background)
    t.start()

    mc = MotorCommander()
    mc.get_logger().info("Motor commander started")
    rclpy.get_global_executor().add_node(mc)
    rate = mc.create_rate(100) # 100hz

    # arm the motor
    mc.get_logger().info("arm the motor...")
    mc.armMotor(mc.motorID)
    rate.sleep()
    loopWait(0.5)

    # disarm motor
    mc.get_logger().info("Disarming the motor...")
    mc.disarmMotor(mc.motorID)
    rate.sleep()
    loopWait(0.5)

    # read joint state
    mc.get_logger().info("Current robot joint state post disarm: {}".format(mc.robot_joint_state))
    
    # First loop: try to zero the joint state
    while abs(mc.robot_joint_state[0]) > 0.5:
        mc.get_logger().info("Waiting for the robot joint state to zero...")
        while abs(mc.robot_joint_state[0]) > 0.5:
            mc.setZero(mc.motorID)
            mc.get_logger().info("Current robot joint state post zero: {}".format(mc.robot_joint_state))

    # disarm motor
    mc.get_logger().info("Disarming the motor...")
    mc.disarmMotor(mc.motorID)
    rate.sleep()
    loopWait(0.5)

    mc.get_logger().info("Current robot joint state post disarm: {}".format(mc.robot_joint_state))

    mc.get_logger().info("arm the motor...")
    mc.armMotor(mc.motorID)
    rate.sleep()
    loopWait(2.0)

    mc.get_logger().info("Current robot joint state post arm: {}".format(mc.robot_joint_state))

    ii = 0.0
    while rclpy.ok() and ii < 360.0:
        mc.get_logger().info(f"Current angle: {ii} degrees")
        mc.get_logger().info("set the motor to given degrees...")
        mc.setAngle(mc.motorID, ii, 5.0, 0.0)
        loopWait(2.0)

        mc.get_logger().info("robot joint state post set angle: {}".format(mc.robot_joint_state))
        ii += 5.0

    mc.get_logger().info("disarm the motor...")
    mc.disarmMotor(mc.motorID)
    rate.sleep()
    loopWait(2.0)

    t.join()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt, shutting down motor testbench")
        sys.exit(0)
    except Exception as e:
        print("Exception occurred: ", e)
        sys.exit(1)
