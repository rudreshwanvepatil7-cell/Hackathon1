from ml_collections import config_dict

import rl.envs as registry


def brax_ppo_config(env_name: str) -> config_dict.ConfigDict:
    """Returns tuned Brax PPO config for the given environment."""
    env_config = registry.get_default_config(env_name)

    return config_dict.create(
        num_timesteps=200_000_000,
        num_evals=10,
        num_resets_per_eval=1,
        reward_scaling=1.0,
        episode_length=env_config.episode_length,
        normalize_observations=True,
        action_repeat=1,
        unroll_length=20,
        num_minibatches=1,
        num_updates_per_batch=4,
        discounting=0.97,
        learning_rate=3e-4,
        entropy_cost=1e-2,
        num_envs=1,
        batch_size=1,
        max_grad_norm=1.0,
        network_factory=config_dict.create(
            policy_hidden_layer_sizes=(128, 128, 128),
            value_hidden_layer_sizes=(128, 128, 128),
            policy_obs_key="state",
            value_obs_key="privileged_state",
        ),
    )
