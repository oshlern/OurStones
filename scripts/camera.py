import numpy as np

class Camera(object):
    def __init__(self):
        print("Initialized Camera")

    def estimate_place_position(self):
        guessed_pose = np.array([0.054, 0.379, 0.294])
        return guessed_pose

    def get_rock_position(self):
        initial_rock_position = np.array([-0.103,0.379,0.296])
        return initial_rock_position
