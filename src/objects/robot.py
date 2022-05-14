# TODO: Change into a script?

import numpy as np

Z_HAT = np.array([0,0,1])

class Robot(Object):
    def __init__(self, gripper, arm, camera, ft_sensor):
        self.gripper = gripper
        self.arm = arm
        self.camera = camera
        self.ft_sensor = ft_sensor

        self.balance_threshold = 0.1 # min |torque|/|force| to consider balanced
        self.max_adjustment = 0.1
        self.max_iterations = 10
        self.max_force = 20 # placement force

    # def __init__(self, gripper_args, arm_args, camera_args, ft_sensor_args):
    #     self.gripper = Gripper(*gripper_args)
    #     self.arm = Arm(*arm_args)
    #     self.camera = Camera(*camera_args)
    #     self.ft_sensor = ForceTorqueSensor(*ft_sensor_args)

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
