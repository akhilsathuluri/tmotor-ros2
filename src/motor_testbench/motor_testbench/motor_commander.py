import time
import numpy as np
from rclpy.node import Node
from functools import partial
from motor_testbench_msgs.msg import TmotorCmd, TmotorData
from std_msgs.msg import Float32MultiArray

def loopWait(dt):
    """
    Function to wait for a given time in seconds.
    """
    t0 = time.time()
    while (time.time() - t0) < dt:
        pass

class MotorCommander(Node):
    def __init__(self, motor_ids, joint_ids):
        super().__init__('motor_commander')
        self.declare_parameter('kp', [0.0])
        self.kp_val = self.get_parameter('kp').get_parameter_value().double_array_value

        self.declare_parameter('kd', [0.0])
        self.kd_val = self.get_parameter('kd').get_parameter_value().double_array_value

        self.declare_parameter('qf', [0.0,0.0,0.0,0.0,0.0])
        self.qf_des = self.get_parameter('qf').get_parameter_value().double_array_value

        self.motorIDs = motor_ids
        self.jointIDs = joint_ids
        self.joint_state = np.zeros(len(motor_ids))
        self.get_logger().info('Motor Commander Node has been started.')
        self.loop_wait_time = 0.0001

        # created in the order of motorIDs == robot joint order
        self.pub_states = []
        self.pub_commands = []
        self.sub_states = []
        for ii in range(len(self.motorIDs)):
            self.pub_states.append(self.create_publisher(TmotorCmd, 'motor_state/motor_'+str(self.motorIDs[ii]), 10))
            self.pub_commands.append(self.create_publisher(TmotorCmd, 'motor_command/motor_'+str(self.motorIDs[ii]), 10))
            self.sub_states.append(self.create_subscription(TmotorData, 'can_response/motor_'+str(self.motorIDs[ii]), partial(self.motor_state_callback, jointID=self.jointIDs[ii]), 10))

        self.joint_state_publisher = self.create_publisher(Float32MultiArray, 'joint_state', 10)

        rate = self.create_rate(50) # 10hz
        self.get_logger().info('publishers initialised')

        # Wait for all publishers to have at least one subscriber
        for ii in range(len(self.motorIDs)):
            while self.pub_states[ii].get_subscription_count() == 0:
                self.get_logger().info(f"Establishing connection with: joint_{self.jointIDs[ii]}, motor_{self.motorIDs[ii]} ... {self.pub_states[ii].get_subscription_count()}")
                loopWait(self.loop_wait_time)
                # NOTE: Do not use rate.sleep() here, it will block the node                
                # rate.sleep()

    def motor_state_callback(self, data, jointID):
        self.joint_state[jointID] = data.position
        # Publish the joint state
        joint_state_msg = Float32MultiArray()
        joint_state_msg.data = self.joint_state.tolist()
        self.joint_state_publisher.publish(joint_state_msg)

    def armMotor(self, jointID):
        cmd = TmotorCmd()
        cmd.status = True
        cmd.setzero = False
        cmd.position = 0.0
        cmd.velocity = 0.0
        cmd.torque = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        self.pub_states[jointID].publish(cmd)
        # self.get_logger().info('motor armed')
        loopWait(self.loop_wait_time)

    def setZero(self, jointID):
        cmd = TmotorCmd()
        cmd.status = True
        cmd.setzero = True
        cmd.position = 0.0
        cmd.velocity = 0.0
        cmd.torque = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        self.pub_states[jointID].publish(cmd)
        # self.get_logger().info('motor set to zero')
        loopWait(0.8)

    def disarmMotor(self, jointID):
        cmd = TmotorCmd()
        cmd.position = 0.0
        cmd.velocity = 0.0
        cmd.torque = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.status = False
        cmd.setzero = False
        self.pub_states[jointID].publish(cmd)
        # self.get_logger().info('motor disarmed')
        loopWait(self.loop_wait_time)

    def setAngle(self, jointID, angle, kp, kd):
        cmd = TmotorCmd()
        cmd.status = True
        cmd.setzero = False
        cmd.position = angle
        cmd.velocity = 0.0
        cmd.torque = 0.0
        cmd.kp = kp # 20
        cmd.kd = kd # 4
        self.pub_commands[jointID].publish(cmd)
        # self.get_logger().info('Angle Set')
        loopWait(0.1)

    def setCommand(self, jointID, angle, velocity, kp, kd):
        cmd = TmotorCmd()
        cmd.status = True
        cmd.setzero = False
        # The input angle is in degrees
        cmd.position = angle
        cmd.velocity = velocity
        cmd.torque = 0.0
        cmd.kp = kp # 20
        cmd.kd = kd # 4
        self.pub_commands[jointID].publish(cmd)

    def zero_all_joints(self):
        for joint_id in self.jointIDs:
            self.get_logger().info(f"Setting joint state to zero for joint_{joint_id}, motor_{self.motorIDs[joint_id]}...")
            while abs(self.joint_state[joint_id]) > 0.5:
                self.setZero(joint_id)
            self.get_logger().info(f"Zeroed joint_{joint_id}, motor_{self.motorIDs[joint_id]}: {self.joint_state[joint_id]}")
        self.get_logger().info("Current robot joint state post zero: {}".format(self.joint_state))


    def arm_all_joints(self):
        for joint_id in self.jointIDs:
            self.get_logger().info(f"Arming joint_{joint_id}, motor_{self.motorIDs[joint_id]}...")
            self.armMotor(joint_id)

    def disarm_all_joints(self):
        for joint_id in self.jointIDs:
            self.get_logger().info(f"Disarming joint_{joint_id}, motor_{self.motorIDs[joint_id]}...")
            self.disarmMotor(joint_id)

    def set_desired_joint_angles(self, desired_angle: list, kp: list, kd: list):
        """
        Set the desired joint state for a specific joint.
        """
        for joint_id in self.jointIDs:
            self.setAngle(joint_id, desired_angle[joint_id], kp[joint_id], kd[joint_id])
        loopWait(0.1)

    def set_desired_joint_states(self, desired_angle: list, desired_velocity: list, kp: list, kd: list):
        """
        Set the desired joint state for a specific joint.
        """
        for joint_id in self.jointIDs:
            self.setCommand(joint_id, desired_angle[joint_id], desired_velocity[joint_id], kp[joint_id], kd[joint_id])
        loopWait(0.1)        

class TrajectoryTracker():
    def __init__(self, mc: MotorCommander):
        self.tf = 0
        self.control_latency = 0.1
        self.nq = len(mc.motorIDs)

        self.qi = np.zeros(self.nq)
        # self.qf = np.zeros(self.nq)
        self.qf = mc.qf_des
        self.vi = np.zeros(self.nq)
        self.vf = np.zeros(self.nq)

        self.interpolation_method = 'cubic'
        self.t0 = 0.0
        # output ref trajectory as a function of time
        self.generated_traj = {}
        self.cc = np.empty((4, self.nq))
        # self.kp = np.ones(self.nq) * 4
        # self.kd = np.ones(self.nq) * 0.1
        self.get_logger = mc.get_logger
        

    def get_current_reference(self, t0, tc):
        t_current = tc - t0
        if self.interpolation_method == 'linear':
            q_c = self.qi*(1-t_current/self.tf) + self.qf*(t_current/self.tf)
            v_c = np.zeros(self.nq)
            return q_c, v_c
        elif self.interpolation_method == 'cubic':
            # if self.cc.size == 0:
            #     raise ValueError("Coefficients 'cc' have not been computed yet.")
            # else:
            q_c = self.cc[0]*t_current**3+self.cc[1]*t_current**2+self.cc[2]*t_current+self.cc[3]
            v_c = 3*self.cc[0]*t_current**2 + 2*self.cc[1]*t_current + self.cc[2]
            q_t_ref = lambda t: self.cc[0]*t**3 + self.cc[1]*t**2 + self.cc[2]*t + self.cc[3]
            v_t_ref = lambda t: 3*self.cc[0]*t**2 + 2*self.cc[1]*t + self.cc[2]
            return q_c, v_c
        elif self.interpolation_method == 'cubic_pos':
            # if self.cc.size == 0:
            #     raise ValueError("Coefficients 'cc' have not been computed yet.")
            # else:
            q_c = self.cc[0]*t_current**3+self.cc[1]*t_current**2+self.cc[2]*t_current+self.cc[3]
            v_c = np.zeros(self.nq)
            return q_c, v_c
        else:
            raise ValueError(f"Interpolation method '{self.interpolation_method}' is not supported.")
        
    # def compute_cc(self):
    #     self.cc[0] = (-2*(self.qf-self.qi)+self.tf*(self.vf+self.vi))/self.tf**3
    #     self.cc[1] = -(-3*(self.qf-self.qi)+self.tf*(self.vf+2*self.vi))/self.tf**2
    #     self.cc[2] = self.vi
    #     self.cc[3] = self.qi

    def compute_cc(self, ti, tf, qi, qf, vi, vf):
        cc = np.empty((4, self.nq))
        cc[0] = (-2*qf + 2*qi + (tf - ti)*(vf + vi)) / (tf - ti)**3
        cc[1] = (3*qf*(tf + ti) - 3*qi*(tf + ti) - (tf - ti)*(tf*vf + 2*ti*vf + 2*tf*vi + ti*vi)) / (tf - ti)**3
        cc[2] = (-6*qf*tf*ti + 6*qi*tf*ti + (tf - ti)*(ti*(2*tf + ti)*vf + tf*(tf + 2*ti)*vi)) / (tf - ti)**3
        cc[3] = (qi*tf**2*(tf - 3*ti) + ti*(qf*(3*tf - ti)*ti - tf*(tf - ti)*(ti*vf + tf*vi))) / (tf - ti)**3
        return cc
    
    # def track_trajectory(self):
    #     self.t0 = time.time()
    #     t_current = time.time() - self.t0
    #     while t_current < self.tf:
    #         q_c, v_c = self.get_current_reference(self.t0, time.time())
    #         self.mc.set_desired_joint_states(q_c, v_c, self.mc.kp_val, self.mc.kd_val)
    #         t_current = time.time() - self.t0

    def generate_multi_trajectory(self):
        total_T = self.tf
        home_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        home_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # intermediate_joint_pos = np.array([
        #     [58.0, 27.0, 33.0, 55.0, -86.0],
        # ])

        intermediate_joint_pos = np.loadtxt('./assets/captured_poses_25-06-23.csv', delimiter=',')
        self.get_logger().info(f"Loaded intermediate joint positions: {intermediate_joint_pos}")

        intermediate_joint_vel = np.zeros_like(intermediate_joint_pos)
        all_pos = np.vstack((home_pos, intermediate_joint_pos, home_pos))
        all_vel = np.vstack((home_vel, intermediate_joint_vel, home_vel))

        time_intervals = np.linspace(0, total_T, len(all_pos))
        qpolys = []
        vpolys = []
        for ii in range(len(time_intervals)-1):
            t0 = time_intervals[ii]
            tf = time_intervals[ii+1]
            
            qi = all_pos[ii]
            qf = all_pos[ii+1]
            vi = all_vel[ii]
            vf = all_vel[ii+1]

            cc = self.compute_cc(t0, tf, qi, qf, vi, vf)
            qpolys.append(lambda t_current, coeffs=cc.copy(): coeffs[0]*t_current**3+coeffs[1]*t_current**2+coeffs[2]*t_current+coeffs[3])
            vpolys.append(lambda t_current, coeffs=cc.copy(): 3*coeffs[0]*t_current**2 + 2*coeffs[1]*t_current + coeffs[2])

        def piecewise_trajectory(t):
            t = max(0, min(t, total_T))
            
            for ii in range(len(time_intervals)-1):
                if time_intervals[ii] <= t < time_intervals[ii+1]:
                    return qpolys[ii](t), vpolys[ii](t)
            
            return all_pos[-1], all_vel[-1]

        return piecewise_trajectory