'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
import numpy as np
from os import listdir
import os


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(os.path.dirname(os.path.abspath(__file__)) + '\\' + 'robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        postures = [
            "Back",
            "Belly",
            "Crouch",
            "Frog",
            "HeadBack",
            "Knee",
            "Left",
            "Right",
            "Sit",
            "Stand",
            "StandInit",
        ]

        data = [[perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'], \
                perception.joint['LKneePitch'], perception.joint['RHipYawPitch'], perception.joint['RHipRoll'], \
                perception.joint['RHipPitch'], perception.joint['RKneePitch'], perception.imu[0], perception.imu[1]]]

        posture = self.posture_classifier.predict(data)[0]
        
        return postures[posture]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
