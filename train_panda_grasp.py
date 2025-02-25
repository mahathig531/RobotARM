from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from panda_grasp_env import PandaGraspEnv

# Create and wrap environment
env = DummyVecEnv([lambda: PandaGraspEnv(render=False)])

# Define RL model (PPO)
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
print("Training RL agent...")
model.learn(total_timesteps=500000)  # Adjust timesteps as needed

# Save trained model
model.save("panda_grasp_ppo")
print("Model saved as panda_grasp_ppo.zip")

# Close environment
env.close()
