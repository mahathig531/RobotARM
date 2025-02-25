import sys
import os
import time
from stable_baselines3 import PPO
from panda_grasp_env import PandaGraspEnv

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Load trained model
model = PPO.load("panda_grasp_ppo")

# Create environment (with rendering enabled)
env = PandaGraspEnv(render=True)

# Ensure PyBullet is initialized
obs = env.reset()[0]  

done = False
while not done:
    action, _ = model.predict(obs)

    # ✅ Close the gripper before lifting
    action[3] = 0  # Set gripper action to closed (0)
    obs, reward, done, _, _ = env.step(action)

    # ✅ Reduce rendering load (fix for macOS)
    time.sleep(1 / 60)  

print("Task Completed!")
env.close()
