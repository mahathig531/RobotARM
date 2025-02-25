import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import spaces
import time

class PandaGraspEnv(gym.Env):
    """Custom RL environment for robotic grasping in PyBullet."""

    def __init__(self, render=False):
        super(PandaGraspEnv, self).__init__()
        self.render_mode = render
        self.physics_client = p.connect(p.GUI)  # ✅ Use GUI mode

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        # Load environment
        self.plane = p.loadURDF("plane.urdf")
        self.table = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.63])
        self.robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
        self.cube = p.loadURDF("cube_small.urdf", basePosition=[0.5, np.random.uniform(-0.2, 0.2), 0.05])

        self.end_effector_link = 11
        self.num_joints = p.getNumJoints(self.robot)

        # Action space: 3D movement + gripper control
        self.action_space = spaces.Box(low=np.array([-0.05, -0.05, -0.05, 0]), 
                                       high=np.array([0.05, 0.05, 0.05, 1]), 
                                       dtype=np.float32)
        
        # Observation space: robot joints + cube position
        self.observation_space = spaces.Box(low=-1, high=1, shape=(10,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        """Reset environment to initial state."""
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)

        # ✅ Reconnect PyBullet if needed
        if not p.isConnected():
            self.physics_client = p.connect(p.GUI)

        # Reload objects
        self.plane = p.loadURDF("plane.urdf")
        self.table = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.63])
        self.robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
        self.cube = p.loadURDF("cube_small.urdf", basePosition=[0.5, np.random.uniform(-0.2, 0.2), 0.05])

        # Reset robot arm position
        initial_joint_positions = [0, -0.5, 0, -1.5, 0, 1.0, 0.8]
        for joint in range(7):
            p.resetJointState(self.robot, joint, initial_joint_positions[joint])

        return self._get_observation(), {}  

    def step(self, action):
        """Apply action and return new state, reward, done, and info."""
        if not p.isConnected():
            raise RuntimeError("🚨 PyBullet is NOT connected! Check if reset() was called.")

        if not isinstance(action, np.ndarray) or len(action) != 4:
            raise ValueError("🚨 Invalid action received! Expected 4 values.")

        # Get current end-effector position
        ee_state = p.getLinkState(self.robot, self.end_effector_link)
        ee_pos = np.array(ee_state[0])

        # ✅ Clip action range to prevent extreme movements
        target_pos = ee_pos + np.clip(action[:3], -0.05, 0.05)

        # ✅ Define workspace limits
        workspace_limits = {
            "x": (0.2, 0.7),  
            "y": (-0.3, 0.3),  
            "z": (0.0, 0.6)  
        }

        # ✅ Ensure target position stays within robot’s reach
        target_pos[0] = np.clip(target_pos[0], workspace_limits["x"][0], workspace_limits["x"][1])
        target_pos[1] = np.clip(target_pos[1], workspace_limits["y"][0], workspace_limits["y"][1])
        target_pos[2] = np.clip(target_pos[2], workspace_limits["z"][0], workspace_limits["z"][1])

        target_orientation = p.getQuaternionFromEuler([0, 3.14, 0])

        # ✅ Compute Inverse Kinematics
        joint_angles = p.calculateInverseKinematics(self.robot, self.end_effector_link, target_pos, target_orientation)

        for joint in range(7):  
            p.setJointMotorControl2(self.robot, joint, p.POSITION_CONTROL, joint_angles[joint], force=100)

        # ✅ CONTROL THE GRIPPER
        gripper_action = action[3]  # Open (1) or Close (0)
        left_finger_joint = 9   
        right_finger_joint = 10  

        if gripper_action > 0.5:  # Open Gripper
            p.setJointMotorControl2(self.robot, left_finger_joint, p.POSITION_CONTROL, targetPosition=0.04, force=10)
            p.setJointMotorControl2(self.robot, right_finger_joint, p.POSITION_CONTROL, targetPosition=0.04, force=10)
        else:  # Close Gripper
            p.setJointMotorControl2(self.robot, left_finger_joint, p.POSITION_CONTROL, targetPosition=0.0, force=100)
            p.setJointMotorControl2(self.robot, right_finger_joint, p.POSITION_CONTROL, targetPosition=0.0, force=100)

        p.stepSimulation()
        time.sleep(1 / 240)  

        # Check if object is grasped
        cube_pos, _ = p.getBasePositionAndOrientation(self.cube)
        grasped = np.linalg.norm(np.array(cube_pos) - np.array(target_pos)) < 0.05  

        # Reward system for grasping & placing
        reward = 10 if grasped else 0
        reward += 20 if cube_pos[2] > 0.1 else 0
        reward += 30 if cube_pos[0] > 0.6 else 0

        done = cube_pos[0] > 0.6 and cube_pos[2] > 0.1

        return self._get_observation(), reward, done, False, {}

    def _get_observation(self):
        """Return robot joint positions + cube position."""
        joint_states = np.array([p.getJointState(self.robot, i)[0] for i in range(7)])
        cube_pos, _ = p.getBasePositionAndOrientation(self.cube)
        return np.concatenate([joint_states, cube_pos])

    def close(self):
        p.disconnect()
