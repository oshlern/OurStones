# TODO: Change into a script?

import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



"""
Some random visualization toools
"""
def visualize_wrench_3d(wrench):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.axes.set_xlim3d(left=-20, right=20)
    ax.axes.set_ylim3d(bottom=-20, top=20)
    ax.axes.set_zlim3d(bottom=-20, top=20)
    # Coordinate axis
    ax.quiver(*tuple(np.zeros(3)), *tuple([1,0,0]), color='red')
    ax.quiver(*tuple(np.zeros(3)), *tuple([0,1,0]), color='green')
    ax.quiver(*tuple(np.zeros(3)), *tuple([0,0,1]), color='blue')
    # Force
    ax.quiver(*tuple(np.zeros(3)), *tuple(wrench[:3]), color='royalblue')
    # Torque
    ax.quiver(*tuple(np.zeros(3)), *tuple(wrench[3:]), color='forestgreen')
    plt.show()

def visualize_torque(wrench):
    pass

"""
The actually important code
"""

Z_HAT = np.array([0,0,1])

class Robot(object):
    R_base_gripper = np.array([[0, -1, 0],
                               [-1, 0, 0],
                               [0, 0, -1]])
    def __init__(self, gripper, arm, camera, ft_sensor):
        self.gripper = gripper
        self.arm = arm
        self.camera = camera
        self.ft_sensor = ft_sensor

        self.balance_threshold = 0.015 # min |torque|/|force| to consider balanced
        self.max_adjustment = 0.02
        self.max_iterations = 10
        self.max_force = 5.0 # placement force in newtons
        print("Initialized Robot")

    # def __init__(self, gripper_args, arm_args, camera_args, ft_sensor_args):
    #     self.gripper = Gripper(*gripper_args)
    #     self.arm = Arm(*arm_args)
    #     self.camera = Camera(*camera_args)
    #     self.ft_sensor = ForceTorqueSensor(*ft_sensor_args)

    def place_attempt(self, overhead_position, rotation=None):
        if rotation is not None:
            self.arm.move_to(overhead_position, rotation)
        else:
            self.arm.move_to(overhead_position)

        # Bring down
        F = np.array([0,0,0])
        Fz = 0
        while -Fz < self.max_force:
            # print(Fz)
            self.arm.move_relative(-0.002 * Z_HAT) # go down .5cm. TODO: use vel inputs instead?
            time.sleep(1)
            Fx, Fy, Fz, Tx, Ty, Tz = self.ft_sensor.read_force_torque()
            F, T = np.array([Fx, Fy, Fz]), np.array([Tx, Ty, Tz])

        # Sense
        time.sleep(1)
        Fx, Fy, Fz, Tx, Ty, Tz = self.ft_sensor.read_force_torque()
        F, T = np.array([Fx, Fy, Fz]), np.array([Tx, Ty, Tz])

        print("F, T", F, T)
        print("score", np.linalg.norm(T) / np.linalg.norm(F))
        if (np.linalg.norm(T) / np.linalg.norm(F)) < self.balance_threshold:
            print("Object is balanced. Releasing")
            self.gripper.open()
            self.arm.move_relative(0.2 * Z_HAT) # go up 20cm
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

        print("contact torques", Tx, Ty)
        print("update step", r_scaled)
        self.arm.move_relative(0.03 * Z_HAT) # go up 10cm
        current_position, current_rot = self.arm.read_pose()
        new_overhead_position = current_position + self.R_base_gripper@r_scaled # overhead position for next placement
        return new_overhead_position

    def place_loop(self, initial_position, max_iterations=10):
        overhead_position = initial_position + 0.12 * Z_HAT
        for _ in range(max_iterations):
            overhead_position = self.place_attempt(overhead_position) # TODO: unroll?
            if overhead_position == "released":
                break


    def analyze_placement(self, wrench, pose, rot, alpha=0.2, visualize=False):
        F, T = wrench[:3], wrench[3:]
        if visualize:
            visualize_wrench_3d(wrench)

        # Compute step
        r = np.cross(Z_HAT, T)
        F_normal = F[2]
        r_scaled = alpha * r / F_normal
        if np.linalg.norm(r_scaled) > self.max_adjustment:
            print("Using max step size")
            r_scaled = (r_scaled / np.linalg.norm(r_scaled)) * self.max_adjustment
        step = self.R_base_gripper @ r_scaled
        return step

    def place(self, pose):
        # Start at overhead position and work downwards
        self.arm.move_to(pose + 0.02 * Z_HAT)
        self.arm.set_vel([0,0,-0.005,0,0,0])
        # Move down until contact
        while not (-self.ft_sensor.read_force_torque()[2] > self.max_force):
            pass
            # self.arm.set_vel([0,0,-0.01,0,0,0])
        self.arm.stop_speed_control()
        time.sleep(0.1)
        return self.arm.read_pose()

    def place_rock(self, pose_estimate, record=True):
        force_zero = self.zero_point * np.array([1,1,1,0,0,0])
        torque_zero = self.placement_zero * np.array([0,0,0,1,1,1])


        target_pose = pose_estimate
        data = []
        def recorder(wrench):
            data.append((time.time(), wrench))
            return wrench
        get_ft = self.ft_sensor.read_force_torque if not record else lambda: recorder(self.ft_sensor.read_force_torque())

        def place(pose):
            # Start at overhead position and work downwards
            self.arm.move_to(pose + 0.02 * Z_HAT)
            self.arm.set_vel([0,0,-0.005,0,0,0])
            # Move down until contact
            while not (-get_ft()[2] - self.zero_point[2] > self.max_force):
                pass
                # self.arm.set_vel([0,0,-0.01,0,0,0])
            self.arm.stop_speed_control()
            time.sleep(3)
            return self.arm.read_pose()

        def score(ft):
            return np.max(np.abs((ft[3:-1] - torque_zero[3:-1]) / (ft[2] - force_zero[2])))


        # for i in range(30):
        #     get_ft()
        #     time.sleep(0.1)
        # time.sleep(1)
        # plt.plot([t for t, _ in data], [ft[2] for _, ft in data])
        # plt.show()

        # Analyze placement + adjust if needed
        # import pickle
        # f = open("g.pickle", 'wb')
        # pickle.dump(data, f)
        # import pdb; pdb.set_trace()

        N_samples = 1

        current_pose, current_rot = place(target_pose)
        scores = []
        fzs = []

        data = []
        ft = np.zeros(6)
        for i in range(N_samples):
            ft += get_ft()
            time.sleep(0.01)
        ft /= N_samples
        plt.plot([t for t, _ in data], [(w[3] - self.zero_point[3])/np.abs(w[2]) for _, w in data], color="red") # torque in x
        plt.plot([t for t, _ in data], [(w[4] - self.zero_point[4])/np.abs(w[2]) for _, w in data], color='green') # torque in y
        plt.pause(0.1)

        # time.sleep(0.2)
        # ft = get_ft()

        # Ignore the first score since it seems to be inacurate
        scores.append(score(ft))
        fzs.append(ft[2])
        print("Force in z:", fzs[-1])
        print("Placement Score:", scores[-1], '\n')

        while scores[-1] > self.balance_threshold:
            # step = self.analyze_placement(ft  - force_zero - torque_zero, current_pose, current_rot)
            step = self.analyze_placement(ft  - force_zero, current_pose, current_rot)
            print("F/T:", ft)
            print("Step:", step)
            current_pose, current_rot = place(current_pose + step)

            data = []
            ft = np.zeros(6)
            for i in range(N_samples):
                ft += get_ft()
                time.sleep(0.01)
            ft /= N_samples
            plt.cla()
            plt.plot([t for t, _ in data], [(w[3] - self.zero_point[3])/np.abs(w[2]) for _, w in data], color="red") # torque in x
            plt.plot([t for t, _ in data], [(w[4] - self.zero_point[4])/np.abs(w[2]) for _, w in data], color='green') # torque in y
            plt.pause(0.1)


            # ft = get_ft()
            scores.append(score(ft))
            fzs.append(ft[2])

            print("Force in z:", fzs[-1])
            print("Placement Score:", scores[-1], '\n')

        print("\nzero point:", self.zero_point)
        print("scores:", scores)
        print("forces:", fzs, '\n')

        time.sleep(1)
        self.gripper.open()
        time.sleep(1)
        self.arm.move_relative(np.array([0,0,0.03]))
        time.sleep(1)
        self.arm.tuck()

    def pick_up_rock(self):
        rock_position = self.camera.get_rock_position()
        overhead_position = np.array([
            rock_position[0],
            rock_position[1],
            rock_position[2] + 0.2])

        self.arm.tuck()
        self.arm.move_to(overhead_position, position_name="overhead", fast=True)
        # input("Press enter to continue")np.max(np.abs(ft[3:-1] / ft[2]))
        self.arm.move_to(rock_position + Z_HAT * 0.03, position_name="rock_overhead", fast=True)
        self.arm.move_to(rock_position, position_name="rock")
        print("Grasping rock")
        self.ft_sensor.calibrate()
        self.gripper.close()
        # self.ft_sensor.calibrate()
        print('ground FT', self.ft_sensor.read_force_torque())

        # Record placement zero
        N_samples = 50
        self.place(rock_position)
        ft = np.zeros(6)
        for i in range(N_samples):
            ft += self.ft_sensor.read_force_torque()
            time.sleep(0.01)
        ft /= N_samples
        self.placement_zero = ft
        print("Placement zero:", self.placement_zero)

        self.arm.move_to(overhead_position, position_name="overhead", fast=True)

        time.sleep(3)
        z = np.zeros(6)
        for _ in range(100):
            z += self.ft_sensor.read_force_torque()
            time.sleep(0.01)
        z /= 100

        self.zero_point = z
        print('air FT (zero pt)', self.zero_point)
        self.arm.tuck(tcp=True)

    def main(self):
        self.gripper.calibrate()
        # input("Press enter to continue")
        self.pick_up_rock()
        est_pose = self.camera.estimate_place_position()
        est_pose[2] += 0.05
        self.place_rock(est_pose)
        # self.place_loop(initial_position)
