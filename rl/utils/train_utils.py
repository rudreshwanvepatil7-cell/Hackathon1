import os
import functools
from etils import epath
from tqdm import tqdm
import numpy as np

import jax
from jax import numpy as jp
import mujoco
import mediapy as media
from scipy.spatial.transform import Rotation as R
from torch.utils.tensorboard import SummaryWriter
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from brax.training.agents.ppo import checkpoint
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
            self._writer.add_scalar(
                k, float(v), num_steps, walltime=metrics["eval/walltime"]
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
        log_training_metrics=True,
    )

    with tqdm(total=train_cfg.num_timesteps) as pbar:
        make_inference_fn, params, metrics = _train(
            environment=env,
            eval_env=registry.load(env_name, config=env_cfg),
            progress_fn=ProgressLogger(log_path, pbar),
        )


def _draw_rotate_command(scn, cmd, xyz, rgba=None, radius=0.1, scale=5.0):
    if rgba is None:
        rgba = [0.2, 0.2, 0.6, 1.0]
    scn.ngeom += 1
    scn.geoms[scn.ngeom - 1].category = mujoco.mjtCatBit.mjCAT_DECOR

    rotation_matrix = R.from_euler("xyz", cmd).as_matrix()

    arrow_from = xyz
    to = rotation_matrix @ np.array([1, 0, 0])
    arrow_to = arrow_from + to * scale

    mujoco.mjv_initGeom(
        geom=scn.geoms[scn.ngeom - 1],
        type=mujoco.mjtGeom.mjGEOM_ARROW.value,
        size=np.zeros(3),
        pos=np.zeros(3),
        mat=np.zeros(9),
        rgba=np.asarray(rgba).astype(np.float32),
    )
    mujoco.mjv_connector(
        geom=scn.geoms[scn.ngeom - 1],
        type=mujoco.mjtGeom.mjGEOM_ARROW.value,
        width=radius,
        from_=arrow_from,
        to=arrow_to,
    )


def test(env_name: str, model_path: str, log_dir: str, deterministic: bool = True):
    env_cfg = registry.get_default_config(env_name)
    env = registry.load(env_name, config=env_cfg)

    policy_fn = checkpoint.load_policy(model_path, deterministic=deterministic)

    jit_reset = jax.jit(env.reset)
    jit_step = jax.jit(env.step)
    jit_inference_fn = jax.jit(policy_fn)

    x = 0.0
    y = 0.0
    z = 90.0

    rng = jax.random.PRNGKey(0)
    rollout = []
    modify_scene_fns = []

    rewards = []
    ori = []
    angvel = []
    track = []
    foot_vel = []
    rews = []
    command = jp.radians(jp.array([x, y, z]))

    state = jit_reset(rng)
    state.info["command"] = command
    for i in range(env_cfg.episode_length):
        act_rng, rng = jax.random.split(rng)
        ctrl, _ = jit_inference_fn(state.obs, act_rng)
        state = jit_step(state, ctrl)
        state.info["command"] = command
        rews.append({k: v for k, v in state.metrics.items() if k.startswith("reward/")})
        rollout.append(state)
        rewards.append(
            {k[7:]: v for k, v in state.metrics.items() if k.startswith("reward/")}
        )
        ori.append(env.get_ori(state.data))
        angvel.append(env.get_gyro(state.data))
        track.append(
            env._reward_tracking_angle(state.info["command"], env.get_ori(state.data))
        )

        xyz = np.array(state.data.xpos[env._torso_body_id])
        xyz += np.array([0, 0, 0.2])
        x_axis = state.data.xmat[env._torso_body_id, 0]
        yaw = -np.arctan2(x_axis[1], x_axis[0])
        modify_scene_fns.append(
            functools.partial(
                _draw_rotate_command,
                cmd=state.info["command"],
                xyz=xyz,
            )
        )

    render_every = 2
    fps = 1.0 / env.dt / render_every
    traj = rollout[::render_every]
    mod_fns = modify_scene_fns[::render_every]

    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[2] = True
    scene_option.geomgroup[3] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True

    frames = env.render(
        traj,
        camera="track",
        scene_option=scene_option,
        width=640,
        height=480,
        modify_scene_fns=mod_fns,
    )

    media.write_video(log_dir / "video.mp4", frames, fps=fps)
