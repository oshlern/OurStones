import numpy as np

class Camera(Object):
    def __init__(self):
        pass

    def estimate_place_position(self):
        guessed_pose = np.array([1,2,3])
        return guessed_pose

    def get_rock_position(self):
        initial_rock_position = np.array([1,2,3])
        return initial_rock_position
