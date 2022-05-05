import numpy as np

from geometry_msgs.msg import Pose

class Camera(Object):
    def __init__(self):
        pass

    def estimate_place_position(self):
        guessed_pose = np.array([1,2,3])
        return guessed_pose

    def get_rock_position(self):
        initial_rock_position = np.array([1,2,3])
        return initial_rock_position


QUAT_DOWN = [0.727, -0.686, 0.038, 0.002]
DEFAULT_POSITION = [0.752, -0.35, 0.02]

class Arm(Object):
    def __init__(self, planner):
        self.planner = planner
        self.tuck_position = DEFAULT_POSITION
    
    def read_pose(self):
        pose = None # TODO
        position = pose[:3]
        quaternion = pose[3:]
        return position, quaternion

    def move_to(self, position, quaternion=QUAT_DOWN, position_name=""):
        if self.out_of_bounds(position, quaternion):
            raise Exception("Out of bounds with position: {}, quat: {}".format(position, quaternion))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        plan = self.planner.plan_to_pose(pose)
        print("Moving to {} position: {}".format(position_name, pose.position))
        self.planner.execute_plan(plan)

    def tuck(self):
        self.move_to(DEFAULT_POSITION, position_name="tucked")
    
    def move_relative(self, displacement):
        dx, dy, dz = displacement
        current_position, current_quat = self.read_pose()
        target_position = current_position
        target_position[0] += dx
        target_position[1] += dy
        target_position[2] += dz
        self.move_to(target_position, current_quat)
    
    # def set_vel(self, vx, vy, vz): (rotation too)

    def out_of_bounds(self, position, quaternion):
        out = False
        if self.MIN_X < position[0] < self.MAX_X:
            out = True
        if self.MIN_Y < position[1] < self.MAX_Y:
            out = True
        if self.MIN_Z < position[2] < self.MAX_Z:
            out = True
        return out
        # TODO: account for orientation 

class ForceTorqueSensor(Object):
    def __init__(self):
        pass

    def read_force_torque(self): # smoothing?
        FT = None # TODO
        return FT

Z_HAT = np.array([0,0,1])

class Robot(Object):
    def __init__(self, gripper, planner, arm, camera, ft_sensor):
        self.gripper = sawyer_gripper.Gripper("right")
        self.planner = PathPlanner('{}_arm'.format("right"))
        self.arm = Arm(planner)
        self.camera = Camera()
        self.ft_sensor = ForceTorqueSensor()

        self.balance_threshold = 0.1 # min |torque|/|force| to consider balanced
        self.max_adjustment = 0.1
        self.max_iterations = 10
        self.max_force = 10

    def place_attempt(self, overhead_position, orientation):
        self.arm.move_to(overhead_position, orientation)

        # Bring down
        F = np.array([0,0,0])
        while np.linalg.norm(F) < self.max_force:
            self.arm.move_relative(-0.005 * Z_HAT) # go down .5cm. TODO: use vel inputs instead?
            Fx, Fy, Fz, Tx, Ty, Tz = self.ft_sensor.read_force_torque()
            F, T = np.array([Fx, Fy, Fz]), np.array([Tx, Ty, Tz])

        # Sense
        Fx, Fy, Fz, Tx, Ty, Tz = self.ft_sensor.read_force_torque()
        F, T = np.array([Fx, Fy, Fz]), np.array([Tx, Ty, Tz])

        if (np.linalg.norm(T) / np.linalg.norm(F)) < self.balance_threshold:
            print("Object is balanced. Releasing")
            self.gripper.open()
            self.arm.relative_move(0.2 * Z_HAT) # go up 20cm
            return "released"

        r = np.cross(Z_HAT, T)
        F_normal = F
        r_scaled = r / np.linalg.norm(F_normal) # Fz = F_N. Since we calibrated to include gravity.    generally, F_N = F (what about)
        # r_unit =  r/ np.linalg.norm(r)
        # F_N = np.array([Fx, Fy, Fz])
        # normal_vec = F_N / np.linalg.norm(F_N)
        # r_to_flat = - np.cross(normal_vec, np.cross(normal_vec, Z_HAT))
        # r_to_COM = - np.cross(normal_vec, np.cross(normal_vec, r))

        if np.linalg.norm(r_scaled) > self.max_adjustment:
            r_scaled = (r_scaled / np.linalg.norm(r_scaled)) * self.max_adjustment

        self.arm.relative_move(0.1 * Z_HAT) # go up 10cm
        current_position, current_quat = self.arm.read_pose()
        new_overhead_position = current_position + r_scaled # overhead position for next placement
        return new_overhead_position

    def place_loop(self, initial_position, max_iterations=10):
        overhead_position = initial_position + 0.15 * Z_HAT
        for _ in range(max_iterations):
            overhead_pose = self.place_attempt(overhead_position, orientation=QUAT_DOWN) # TODO: unroll?
            if overhead_pose == "Object Released":
                break

    def pick_up_rock(self):
        rock_position = self.camera.get_rock_position()    
        overhead_position = np.array([
            rock_position[0],
            rock_position[1],
            rock_position[2] + 0.2])

        self.arm.tuck()
        self.arm.move_to(overhead_position, position_name="overhead")
        self.arm.move_to(rock_position, position_name="rock")
        print("Grasping rock")
        self.gripper.close()
        self.arm.move_to(overhead_position, position_name="overhead")
        self.arm.tuck()

    def main(self):
        self.gripper.calibrate()
        self.pick_up_rock()
        self.force_torque.calibrate()
        self.arm.tuck()
        initial_position = self.camera.estimate_place_position()
        self.place_loop(initial_position)


# axis of push direction should go through force torque sensor middle