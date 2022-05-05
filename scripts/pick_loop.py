import numpy as np

def out_of_bounds(x,y,z,roll,pitch,yaw):
    if -1 < x < 1:
        return True
    if -1 < y < 1:
        return True
    if -1 < z < 1:
        return True
    return False

MAX_FZ = -10
TORQUE_THRESH = 0.1
MAX_ADJUST = 0.1
ITER_LIMIT = 10
Z_HAT = np.array([0,0,1])

def place_attempt(overhead_pose):
    arm.move_to(overhead_pose)

    # Bring down
    while Fz > MAX_FZ: # smoothing?
        arm.move_relative(0,0,-0.005) # go down .5cm. TODO: use vel inputs instead?
        Fx, Fy, Fz, Tx, Ty, Tz = force_torque.sense()
        x, y, z, roll, pitch, yaw = arm.workspace_pose()
        if out_of_bounds(x,y,z,roll,pitch,yaw):
            raise Exception("Out of bounds with x y z: {} {} {}, rpy: {} {} {}".format(x,y,z,roll,pitch,yaw))
    
    # Sense
    Fx, Fy, Fz, Tx, Ty, Tz = force_torque.sense()
    if np.linalg.norm(np.array([Tx,Ty])) < TORQUE_THRESH:
        gripper.release()
        arm.relative_move(0,0,0.2) # go up 10cm
        return "Object Released"
    else:
        x, y, z, roll, pitch, yaw = arm.workspace_pose()
        r = np.cross(Z_HAT, np.array([Tx,Ty,Tz]))
        r_scaled = r / Fz # Fz = F_N. Since we calibrated to include gravity
        # r_unit =  r/ np.linalg.norm(r)
        F_N = np.array([Fx, Fy, Fz])
        normal_vec = F_N / np.linalg.norm(F_N)
        r_to_flat = - np.cross(normal_vec, np.cross(normal_vec, Z_HAT))
        r_to_COM = - np.cross(normal_vec, np.cross(normal_vec, r))

        if np.linalg.norm(r_scaled) > MAX_ADJUST:
            r_scaled *= MAX_ADJUST / np.linalg.norm(r_scaled)

        
        arm.relative_move(0,0,0.1) # go up 10cm
        x, y, z, roll, pitch, yaw = arm.workspace_pose()
        new_pose = np.array([x,y,z]) + r_scaled # overhead position for next placement
        return new_pose

def place_loop(initial_pose):
    overhead_pose = initial_pose + np.array(0,0,0.2)
    for _ in range(ITER_LIMIT):
        overhead_pose = place_attempt(overhead_pose)
        if overhead_pose == "Object Released":
            break

def main():
    gripper.calibrate()
    pick_up_rock()
    force_torque.calibrate()
    arm.tuck()
    initial_pose = vision.guess_pose()
    place_loop(initial_pose)



# r = Fhat cross (z cross Fhat)



# def execute_grasp(T_world_grasp, planner, gripper):
#     """
#     Perform a pick and place procedure for the object. One strategy (which we have
#     provided some starter code for) is to
#     1. Move the gripper from its starting pose to some distance behind the object
#     2. Move the gripper to the grasping pose
#     3. Close the gripper
#     4. Move up
#     5. Place the object somewhere on the table
#     6. Open the gripper. 

#     As long as your procedure ends up picking up and placing the object somewhere
#     else on the table, we consider this a success!

#     HINT: We don't require anything fancy for path planning, so using the MoveIt
#     API should suffice. Take a look at path_planner.py. The `plan_to_pose` and
#     `execute_plan` functions should be useful. If you would like to be fancy,
#     you can also explore the `compute_cartesian_path` functionality described in
#     http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
    
#     Parameters
#     ----------
#     T_world_grasp : 4x4 :obj:`numpy.ndarray`
#         pose of gripper relative to world frame when grasping object
#     """
#     def close_gripper():
#         """closes the gripper"""
#         # gripper.close(block=True)
#         gripper.close()
#         rospy.sleep(1.0)

#     def open_gripper():
#         """opens the gripper"""
#         # gripper.open(block=True)
#         gripper.open()
#         rospy.sleep(1.0)

#     inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
#     if inp == "exit":
#         return

#     # raise NotImplementedError
#     # Go behind object
#     behind_dist = 0.2
#     pose = Pose()

#     transform = T_world_grasp[:3, 3].ravel()
#     rotation = T_world_grasp[:3, :3]
#     gripper_direction = rotation[:, 2].ravel()

#     contact_position = transform
#     contact_position[2] += 0.017
#     setup_position = contact_position - behind_dist * gripper_direction

#     default_position = [0.752, -0.35, 0.02]
#     default_quaternion = [0.727, -0.686, 0.038, 0.002]

#     pose.position.x = default_position[0]
#     pose.position.y = default_position[1]
#     pose.position.z = default_position[2]
#     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = default_quaternion
#     plan = planner.plan_to_pose(pose)
#     print(default_position)
#     planner.execute_plan(plan) # set speed
    
#     pose.position.x = setup_position[0]
#     pose.position.y = setup_position[1]
#     pose.position.z = setup_position[2]
#     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_matrix(rotation)
#     plan = planner.plan_to_pose(pose)
#     print(setup_position, rotation)
#     planner.execute_plan(plan) # set speed
    
#     # Then swoop in
#     pose.position.x = contact_position[0]
#     pose.position.y = contact_position[1]
#     pose.position.z = contact_position[2]
#     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_matrix(rotation)
#     plan = planner.plan_to_pose(pose)
#     print(contact_position, rotation)
#     planner.execute_plan(plan) 
#     close_gripper()

#     # Bring the object up
#     pose.position.z += 0.2
#     plan = planner.plan_to_pose(pose)
#     planner.execute_plan(plan)

#     # And over
#     pose.position.y -= 0.14
#     plan = planner.plan_to_pose(pose)
#     planner.execute_plan(plan)

#     # And now place it
#     pose.position.z -= 0.2
#     plan = planner.plan_to_pose(pose)
#     planner.execute_plan(plan)
#     open_gripper()

#     # Reset
#     pose.position.z += 0.32
#     plan = planner.plan_to_pose(pose)
#     planner.execute_plan(plan)
#     pose.position.y += 0.1
#     plan = planner.plan_to_pose(pose)
#     planner.execute_plan(plan)

# What if COM not centered in gripper?