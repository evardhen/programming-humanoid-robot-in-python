'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def from_trans(self, matrix):
        x, y, z = matrix[3, 0], matrix[3, 1], matrix[3, 2]
        return np.array([x, y, z]) #TODO: angle calculation doesnt make sense

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        '''jointss = {'HeadYaw': 0.0, 'RHipPitch': -0.0, 'RElbowYaw': -0.0, 'RShoulderPitch': 0.0, 'LShoulderPitch': 0.0, 'LKneePitch': -0.0, 'RAnkleRoll': 0.0, 'LShoulderRoll': 0.008726646259971648, 'LHipPitch': -0.0, 'LElbowYaw': 0.0, 'LAnklePitch': -0.0, 'RHipYawPitch': -0.0, 'HeadPitch': 0.0, 'LElbowRoll': -0.0066322511575784525, 'RShoulderRoll': -0.0066322511575784525, 'LAnkleRoll': -0.0, 'LHipYawPitch': -0.0, 'RAnklePitch': -0.0, 'LHipRoll': -0.0, 'RHipRoll': 0.0, 'RElbowRoll': 0.008726646259971648, 'RKneePitch': -0.0}'''
        
        # YOUR CODE HERE
        # jacobian approximation, actual joint range of motion not included --> optimization might give unreasonable angles
        lambda_ = 0.1
        max_step = 0.1
        all_joints = (self.perception.joint).copy()
        target = (self.from_trans(transform)).T

        for i in range(1000):
            self.forward_kinematics(all_joints) # generate homogenous transformation matrices
            T = self.get_list_HTM(effector_name) # get list of homogenous transformation matrices
            Forward_T = np.array([self.from_trans(T[-1])]).T
            error = target - Forward_T
            error[error > max_step] = max_step
            error[error < -max_step] = -max_step
            if(effector_name == 'LArm' or effector_name == 'RArm'): # Wrist doesnt have a motor/angle for which we can optimize
                T = np.array([self.from_trans(i) for i in T[:-1]]).T
            else:
                T = np.array([self.from_trans(i) for i in T[:]]).T
            J = Forward_T - T
            J[-1, :] = 1  
            JJT = np.dot(J, J.T)
            d_theta = lambda_ * np.dot(np.dot(J.T, np.linalg.pinv(JJT)), error)
            for i, name in enumerate(self.chains[effector_name]):
                if name == 'LWristYaw' or name == 'RWristYaw':
                    continue
                all_joints[name] += np.asarray(d_theta.T)[0, i]
            if np.linalg.norm(d_theta) < 1e-4:
                break
        return all_joints

    def get_list_HTM(self, effector_name):
        T = [0] * len(self.chains[effector_name])
        for i, name in enumerate(self.chains[effector_name]):
            T[i] = self.transforms[name] 
        return T

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE        
        angles = self.inverse_kinematics(effector_name,transform)
        names = []
        times = []
        keys = []
        for name in self.chains[effector_name]:
            if name == 'LWristYaw' or name == 'RWristYaw':
                continue
            names.append(name)
            times.append([2, 5])
            keys.append([[self.perception.joint[name], [3, 0, 0], [3, 0, 0]], [angles[name], [3, 0, 0], [3, 0, 0]]])
        self.keyframes = (names, times, keys)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 0] = 1.
    T[-1, 1] = -1.
    T[-1, 2] = 0.
    agent.set_transforms('LArm', T)
    agent.run()