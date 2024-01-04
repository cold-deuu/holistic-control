import numpy as np
import rospy
from sensor_msgs.msg import JointState
import tf
from tf.transformations import quaternion_matrix
import math
import pinocchio as pin
from std_msgs.msg import Float64
from copy import deepcopy 
import eigenpy


class UR_IK_Control(object):
    def __init__(self):
        folder = '/home/chan/holistic_panda/src/universal_robot/'
        urdfname = 'ur_description/urdf/ur5.urdf'
        self.robot = pin.RobotWrapper.BuildFromURDF(folder+urdfname, folder)
        self.model = self.robot.model
        self.q = self.robot.q0

        rospy.Subscriber('joint_states', JointState, self.JointStateCB)
        self.joint_state = []
        
        self.joint_ctrl_pub = []
        self.joint_ctrl_pub.append(rospy.Publisher('/shoulder_pan_joint_controller/command', Float64, queue_size=10))
        self.joint_ctrl_pub.append(rospy.Publisher('/shoulder_lift_joint_controller/command', Float64, queue_size=10))
        self.joint_ctrl_pub.append(rospy.Publisher('/elbow_joint_controller/command', Float64, queue_size=10))
        self.joint_ctrl_pub.append(rospy.Publisher('/wrist_1_joint_controller/command', Float64, queue_size=10))
        self.joint_ctrl_pub.append(rospy.Publisher('/wrist_2_joint_controller/command', Float64, queue_size=10))
        self.joint_ctrl_pub.append(rospy.Publisher('/wrist3_joint_controller/command', Float64, queue_size=10))
        
        self.joint_ctrl = []
        self.joint_ctrl.append(Float64())
        self.joint_ctrl.append(Float64())
        self.joint_ctrl.append(Float64())
        self.joint_ctrl.append(Float64())
        self.joint_ctrl.append(Float64())
        self.joint_ctrl.append(Float64())
        
        
        self.v = self.robot.v0

        self.rate = rospy.Rate(1000)

    def JointStateCB(self, msg):
        self.joint_state = msg
        
        for i in range(0, len(self.joint_state.name)):
            if self.joint_state.name[i] == "shoulder_pan_joint":
                self.q[0] = self.joint_state.position[2]
                self.v[0] = self.joint_state.velocity[2]
            if self.joint_state.name[i] == "shoulder_lift_joint":
                self.q[1] = self.joint_state.position[1]
                self.v[1] = self.joint_state.velocity[1]
            if self.joint_state.name[i] == "elbow_joint":
                self.q[2] = self.joint_state.position[0]
                self.v[2] = self.joint_state.velocity[0]
            if self.joint_state.name[i] == "wrist_1_joint":
                self.q[3] = self.joint_state.position[3]
                self.v[3] = self.joint_state.velocity[3]
            if self.joint_state.name[i] == "wrist_2_joint":
                self.q[4] = self.joint_state.position[4]
                self.v[4] = self.joint_state.velocity[4]
            if self.joint_state.name[i] == "wrist_3_joint":
                self.q[5] = self.joint_state.position[5]
                self.v[5] = self.joint_state.velocity[5]

        pin.computeAllTerms(self.model, self.robot.data, self.q, self.v)
        
    def q_cubic(self, ros_stime, q_init, q_dot_init, q_goal, duration, ros_time):
        stime = (ros_stime.to_sec() + ros_stime.to_nsec()) * pow(10, -9)
        time = (ros_time.to_sec() + ros_time.to_nsec()) * pow(10, -9)
        q_cubic = deepcopy(q_init)

        if (time < stime):
            return q_init
        elif (time > stime + duration):
            return q_goal
        else:
            for i in range(0, len(q_cubic)):
                a0 = q_init[i]
                a1 = q_dot_init[i]
                a2 = 3.0 / pow(duration, 2) * (q_goal[i] - q_init[i])
                a3 = -1.0 * 2.0 / pow(duration, 3) * (q_goal[i] - q_init[i])
                q_cubic[i] = a0 + a1 * (time - stime) + a2 * pow(time-stime, 2) + a3 * pow(time-stime, 3)
            return q_cubic

    def se3_cubic(self, ros_stime, se3_init, se3_goal, duration, ros_time):
        stime = (ros_stime.to_sec() + ros_stime.to_nsec()) * pow(10, -9)
        time = (ros_time.to_sec() + ros_time.to_nsec()) * pow(10, -9)
        se3_cubic = deepcopy(se3_goal)

        if (time < stime):
            return se3_init
        elif (time > stime + duration):
            return se3_goal
        else:
            rot_diff = pin.log3(np.asmatrix(se3_init.rotation).transpose() * np.asmatrix(se3_goal.rotation))
            tau = self.q_cubic(ros_stime, np.array([0.]), np.array([0.]), np.array([1.]), duration, ros_time)

            for i in range(0, 3):
                a0 = se3_init.translation[i]
                a1 = 0.0
                a2 = 3.0 / pow(duration, 2) * (se3_goal.translation[i] - se3_init.translation[i])
                a3 = -1.0 * 2.0 / pow(duration, 3) * (se3_goal.translation[i] - se3_init.translation[i])
                se3_cubic.translation[i] = a0 + a1 * (time - stime) + a2 * pow(time-stime, 2) + a3 * pow(time-stime, 3)
            
            se3_cubic.rotation = se3_init.rotation * np.matrix(pin.exp(rot_diff * tau))# * pin.exp(rot_diff * tau)
            return se3_cubic

    def quaternion_rotation_matrix(self, Q):
        q0 = Q[3]
        q1 = Q[0]
        q2 = Q[1]
        q3 = Q[2]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix

    def joint_posture_ctrl(self, goal=[], duration=0.0):
        q_goal = goal
        q_init = deepcopy(self.q)
        v_init = deepcopy(self.v)
        stime = deepcopy(rospy.Time.now())

        while rospy.Time.now() <= stime + rospy.Duration(duration + 1.0):
            q_cubic = self.q_cubic(stime, q_init, v_init, q_goal, duration, rospy.Time.now())
            q_d = q_cubic
            for j in range(0, 6):
                self.joint_ctrl[j].data = q_d[j]
                self.joint_ctrl_pub[j].publish(self.joint_ctrl[j])
            
            self.rate.sleep()

    def se3_ctrl(self, goal=[], duration=0.0, relative=True):
        ee_id = self.model.getJointId("wrist_3_joint")
        oMi = self.robot.data.oMi[ee_id]
        
        x_init = deepcopy(oMi)
        EE_goal = pin.SE3()
        EE_goal.translation = goal[:3]
        EE_goal.rotation = self.quaternion_rotation_matrix(goal[3:7])

        if (relative):            
            x_target = deepcopy(oMi) * EE_goal
        else:
            x_target = EE_goal
        
        stime = deepcopy(rospy.Time.now())
        p_gain = 1000.0
        dt = 0.001

        while rospy.Time.now() <= stime + rospy.Duration(duration + 1.0):
            x_cubic = self.se3_cubic(stime, x_init, x_target, duration, rospy.Time.now()) #trajectory
            x_current = self.robot.data.oMi[ee_id]
            dMi = x_cubic.actInv(x_current) # self.target.inverse() * oMi

            err = np.zeros(6)
            err[:3] = dMi.translation # translation error
            err[3:6] = pin.log3(dMi.rotation) # rotation error
            err = np.asmatrix(err)

            J = np.asmatrix(self.robot.getJointJacobian(ee_id, pin.ReferenceFrame.LOCAL ))
            J_inv = J.T * np.linalg.inv(J * J.T + 0.001 * np.matrix(np.eye((6))) )
            q_d_dot = J_inv * (-p_gain * err.T)
            q_d = pin.integrate(self.model, self.q, q_d_dot * dt) # new_q = q + self.v *dt
            for j in range(0, 6):
                self.joint_ctrl[j].data = q_d[j]
                self.joint_ctrl_pub[j].publish(self.joint_ctrl[j])
            
            self.rate.sleep()

        print ("result")
        print (x_target, x_init, x_current)

if __name__ == '__main__':
    try:
        rospy.init_node('inverse_kinematics', anonymous=True)
        target = pin.SE3()

        ik = UR_IK_Control()
        
        import time
        time.sleep(1.0)        

        ik.joint_posture_ctrl(goal = np.array([0, -1.0, 1.0, 0, 0, 0]), duration = 1.0)
        # ik.joint_posture_ctrl(goal = np.array([0, -0.0, 0.0, 0, 0, 0]), duration = 1.0)
        # ik.joint_posture_ctrl(goal = np.array([0, -1.0, 1.0, 0, 0, 0]), duration = 1.0)
        ik.se3_ctrl(goal = np.array([0.2, 0, 0, 0, 0.15, 0, 0.98]), duration=1.0)
        ik.se3_ctrl(goal = np.array([0.6, 0.2, 0.3, 0, 0, 0, 1]), duration=3.0, relative= False)



    except rospy.ROSInterruptException:
        pass
