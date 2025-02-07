import gym
from stable_baselines3 import DQN

import highway_env


if __name__ == '__main__':
    train = True
    # Create the environment
    env = gym.make("highway-fast-v0")
    obs = env.reset()

    # Create the model
    model = DQN('MlpPolicy', env,
                policy_kwargs=dict(net_arch=[256, 256]),
                learning_rate=5e-4,
                buffer_size=15000,
                learning_starts=200,
                batch_size=32,
                gamma=0.8,
                train_freq=1,
                gradient_steps=1,
                target_update_interval=50,
                exploration_fraction=0.7,
                verbose=1,
                tensorboard_log="highway_dqn/")

    # Train the model
    if train:
        model.learn(total_timesteps=int(2e4))
        model.save("highway_dqn/model")

    # Run the algorithm
    model.load("highway_dqn/model")
    while True:
        done = False
        obs = env.reset()
        while not done:
            # Predict
            action, _states = model.predict(obs, deterministic=True)
            # Get reward
            obs, reward, done, info = env.step(action)
            # Render
            env.render()

    env.close()
