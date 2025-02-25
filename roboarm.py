
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load default PyBullet data

# Load environment
plane_id = p.loadURDF("plane.urdf")
table_id = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.63])

# Load Franka Panda Arm with a fixed base
robot_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Load object (use a cube for stability)
object_id = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.05], globalScaling=1.0)

# Apply gravity for stability
p.setGravity(0, 0, -9.8)

# Identify end-effector link by printing joint info
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {joint_info[1].decode('utf-8')}")  # Print joint name

end_effector_link = 11  # This might be 7 or 11, confirm from the output

# Move the arm to an initial safe position
initial_joint_positions = [0, -0.5, 0, -1.5, 0, 1.0, 0.8]
for joint in range(7):
    p.resetJointState(robot_id, joint, initial_joint_positions[joint])

# Function to move arm to target position
def move_arm(target_pos, target_orient, steps=200):
    for _ in range(steps):
        joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link, target_pos, target_orient)
        for joint in range(7):
            p.setJointMotorControl2(robot_id, joint, p.POSITION_CONTROL, joint_angles[joint], force=100)
        p.stepSimulation()
        time.sleep(1 / 240)

# Function to simulate grasping by moving fingers together
def grasp_object():
    # Here you would close the fingers, which is usually done by controlling the motors on the gripper
    print("Grasping the object...")
    # Simulate closing fingers: Update the finger positions or control actuators
    # You can also apply forces here to simulate a pinch (this depends on how you model fingers)
    # No actual code for finger movement here yet, we will just assume "grasping"

# Function to lift the object
def lift_object(lift_height=0.1):
    print("Lifting the object...")
    new_pos = [0.5, 0, lift_height]  # Adjust lift height as needed
    target_orientation = p.getQuaternionFromEuler([0, 3.14, 0])
    move_arm(new_pos, target_orientation, steps=200)

# Function to move the object to a new position
def move_to_new_position(new_position=[0.7, 0, 0.2]):
    print("Moving object to new position...")
    target_orientation = p.getQuaternionFromEuler([0, 3.14, 0])
    move_arm(new_position, target_orientation, steps=200)

# Function to release the object
def release_object():
    print("Releasing the object...")
    # Here you would simulate releasing the object (moving fingers apart)
    # Again, this would depend on how the gripper is modeled
    # Assuming we just stop applying force to the gripper

# Main flow of the automation
def automate_grasping():
    # 1. Move above the object (hover position)
    hover_position = [0.5, 0, 0.3]
    target_orientation = p.getQuaternionFromEuler([0, 3.14, 0])
    move_arm(hover_position, target_orientation, steps=300)

    # 2. Move down to the object to grasp it
    grasp_position = [0.5, 0, 0.05]  # Above the cube
    move_arm(grasp_position, target_orientation, steps=300)

    # 3. Grasp the object
    grasp_object()

    # 4. Lift the object
    lift_object(lift_height=0.2)

    # 5. Move the object to a new position
    move_to_new_position(new_position=[0.7, 0, 0.2])

    # 6. Release the object
    release_object()

# Run the automation
automate_grasping()

# Keep the simulation running
print("Simulation running... Press Ctrl+C to exit.")
while True:
    p.stepSimulation()
    time.sleep(1 / 240)
