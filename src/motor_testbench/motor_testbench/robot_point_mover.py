import rclpy
from rclpy.executors import ExternalShutdownException
import threading
import sys
from motor_testbench.motor_commander import MotorCommander, loopWait, TrajectoryTracker
import numpy as np
import time

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

    # robot joint order motor IDS
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
    mc.get_logger().info("Testing Kd: {}".format(mc.kd_val))

    # initiate the 
    tt = TrajectoryTracker(mc)
    
    # configure trajectory info
    tt.tf = 45.0
    tt.qi = np.array([0.0]*tt.nq)
    # tt.qf = np.array([20.0]*tt.nq)
    tt.vi = np.array([0.0]*tt.nq)
    tt.vf = np.array([0.0]*tt.nq)
    tt.interpolation_method = 'cubic'
    # tt.compute_cc()

    piecewise_trajectory = tt.generate_multi_trajectory()

    tt.t0 = time.time()
    t_current = time.time() - tt.t0
    
    while rclpy.ok() and t_current < tt.tf:
        # q_c, v_c = tt.get_current_reference(tt.t0, time.time())
        q_c, v_c = piecewise_trajectory(t_current)
        mc.get_logger().info("Current reference joint state: {}, {}".format(q_c, v_c))
        mc.set_desired_joint_states(q_c, v_c, mc.kp_val, mc.kd_val)
        t_current = time.time() - tt.t0
        rate.sleep()
        mc.get_logger().info("Reading joint state: {}".format(mc.joint_state))

    time.sleep(2.0)

    mc.get_logger().info("Finished setting angles, disarming motors...")
    # disarm all motors
    mc.disarm_all_joints()
    loopWait(0.5)
    rate.sleep()

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
