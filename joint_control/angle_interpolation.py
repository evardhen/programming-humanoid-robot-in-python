'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, rightBackToStand, wipe_forehead, leftBellyToStand
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        if(self.start_time == -1):
            self.start_time = perception.time
        current_time = perception.time - self.start_time

        names, times, keys = keyframes
        for name in range(len(names)):
            for time in range(len(times[name])-1):
                if times[name][time] < current_time < times[name][time + 1]:
                    b0 = np.array([times[name][time], keys[name][time][0]])
                    b1 = b0 + np.array([keys[name][time][1][1], keys[name][time][1][2]])
                    b2 = b0 + np.array([keys[name][time][2][1], keys[name][time][2][2]])
                    b3 = np.array([times[name][time + 1], keys[name][time + 1][0]])

                    t = (current_time - times[name][time]) / (times[name][time + 1] \
                        - times[name][time])
                    bezier = (((1 - t) ** 3) * b0) + (3 * ((1 - t) ** 2) * t * b1) \
                                + (3 * (1 - t) * (t ** 2) * b2) + ((t ** 3) * b3)
                    target_joints[names[name]] = bezier[1]
                    if(names[name] == "LHipYawPitch"):
                        target_joints["RHipYawPitch"] = bezier[1]

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
