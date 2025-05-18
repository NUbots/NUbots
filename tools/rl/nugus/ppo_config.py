from ml_collections import config_dict


def nugus_ppo_config():
    """
    Returns the default PPO configuration for NUgus joystick training.
    """
    return config_dict.create(
        num_timesteps=100_000_000,
        num_evals=1,
        reward_scaling=1.0,
        episode_length=None,  # This will be set from env_cfg
        normalize_observations=True,
        action_repeat=1,
        unroll_length=20,
        num_minibatches=32,
        num_updates_per_batch=4,
        discounting=0.97,
        learning_rate=3e-4,
        entropy_cost=1e-2,
        num_envs=8192,
        batch_size=256,
        max_grad_norm=1.0,
        network_factory=config_dict.create(
            policy_hidden_layer_sizes=(128, 128, 128, 128),
            value_hidden_layer_sizes=(256, 256, 256, 256, 256),
            policy_obs_key="state",
            value_obs_key="state",
        ),
    )
