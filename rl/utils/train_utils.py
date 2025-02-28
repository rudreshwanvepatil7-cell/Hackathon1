import os
import functools
from etils import epath
from tqdm import tqdm

from torch.utils.tensorboard import SummaryWriter
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from mujoco_playground import wrapper

import rl.envs as registry
import rl.config.crab_config as config


class ProgressLogger:
    def __init__(self, log_path, pbar):
        self._writer = SummaryWriter(log_path)
        self._pbar = pbar
        self._cur_steps = 0

    def __call__(self, num_steps, metrics):
        self._pbar.update(num_steps - self._cur_steps)
        self._cur_steps = num_steps
        for k, v in metrics.items():
            # All keys are prefixed with "eval/"
            self._writer.add_scalar(
                k[5:], float(v), num_steps, walltime=metrics["eval/walltime"]
            )


def setup():
    # Configure MuJoCo to use the EGL rendering backend (requires GPU)
    os.environ["MUJOCO_GL"] = "egl"

    # Tell XLA to use Triton GEMM, this improves steps/sec by ~30% on some GPUs
    xla_flags = os.environ.get("XLA_FLAGS", "")
    xla_flags += " --xla_gpu_triton_gemm_any=True"
    os.environ["XLA_FLAGS"] = xla_flags


def train(
    train_name: str,
    env_name: str,
    model_dir: str,
    log_dir: str,
) -> None:
    ckpt_path = epath.Path(model_dir) / train_name
    log_path = epath.Path(log_dir) / train_name
    ckpt_path.mkdir(parents=True, exist_ok=True)

    env = registry.load(env_name)
    env_cfg = registry.get_default_config(env_name)
    train_cfg = config.brax_ppo_config(env_name)
    randomizer = registry.get_domain_randomizer(env_name)

    ppo_training_params = dict(train_cfg)
    network_factory = ppo_networks.make_ppo_networks
    if "network_factory" in train_cfg:
        del ppo_training_params["network_factory"]
        network_factory = functools.partial(
            network_factory, **train_cfg.network_factory
        )

    _train = functools.partial(
        ppo.train,
        **dict(ppo_training_params),
        network_factory=network_factory,
        randomization_fn=randomizer,
        save_checkpoint_path=ckpt_path,
        wrap_env_fn=wrapper.wrap_for_brax_training,
    )

    with tqdm(total=train_cfg.num_timesteps) as pbar:
        make_inference_fn, params, metrics = _train(
            environment=env,
            eval_env=registry.load(env_name, config=env_cfg),
            progress_fn=ProgressLogger(log_path, pbar),
        )
