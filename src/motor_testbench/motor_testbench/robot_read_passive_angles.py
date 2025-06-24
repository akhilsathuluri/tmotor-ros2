import rclpy
from rclpy.executors import ExternalShutdownException
import threading
import sys
from motor_testbench.motor_commander import MotorCommander, loopWait
import numpy as np
from std_msgs.msg import Float32MultiArray

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    rclpy.init(args=sys.argv)
    t = threading.Thread(target=spin_in_background)
    t.start()

    motor_ids = [3, 2, 4, 1, 5]
    joint_ids = np.arange(len(motor_ids))

    mc = MotorCommander(motor_ids, joint_ids)
    mc.get_logger().info("Motor commander started")
    rclpy.get_global_executor().add_node(mc)
    rate = mc.create_rate(100) # 100hz

    # arm the motor
    mc.arm_all_joints()
    rate.sleep()
    loopWait(0.5)

    # disarm motor
    mc.disarm_all_joints()
    rate.sleep()
    loopWait(0.5)

    # read joint state
    mc.get_logger().info("Current robot joint state post disarm: {}".format(mc.joint_state))
    
    # zero all motors
    mc.zero_all_joints()
    rate.sleep()
    mc.get_logger().info("Current robot joint state post zero: {}".format(mc.joint_state))

    # disarm motor
    mc.disarm_all_joints()
    rate.sleep()
    loopWait(0.5)

    # arm the motor
    mc.arm_all_joints()
    rate.sleep()
    loopWait(0.5)
    mc.get_logger().info("Robot joint state before action: {}".format(mc.joint_state))

    mc.get_logger().info("Testing Kp: {}".format(mc.kp_val))

    ii = 0.0
    while rclpy.ok() and ii < 35.0:
        mc.disarm_all_joints()
        rate.sleep()
        mc.get_logger().info("Reading passive joint state: {}".format(mc.joint_state))

        # joint_state_msg = Float32MultiArray()
        # joint_state_msg.data = mc.joint_state.tolist()
        # mc.joint_state_publisher.publish(joint_state_msg)
        pass

    mc.get_logger().info("Finished setting angles, disarming motors...")
    mc.disarm_all_joints()
    loopWait(0.5)
    rate.sleep()

    mc.get_logger().info("Final robot joint state: {}".format(mc.joint_state))

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
