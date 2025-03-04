import os
import sys
from typing import Any, Dict, Optional, Union

import jax
import jax.numpy as jp
from jax.scipy.spatial.transform import Rotation
from ml_collections import config_dict
import mujoco
from mujoco import mjx
from mujoco.mjx._src import math

from mujoco_playground._src import mjx_env

cwd = os.getcwd()
sys.path.append(cwd)

from rl.utils import crab_constants as consts  # noqa: E402


def default_config() -> config_dict.ConfigDict:
    return config_dict.create(
        ctrl_dt=0.02,
        sim_dt=0.004,
        episode_length=1000,
        Kp=35.0,
        Kd=0.5,
        action_repeat=1,
        action_scale=1.0,
        history_len=1,
        soft_joint_pos_limit_factor=0.95,
        noise_config=config_dict.create(
            level=1.0,  # Set to 0.0 to disable noise.
            scales=config_dict.create(
                joint_pos=0.03,
                joint_vel=1.5,
                ori=0.1,
                gyro=0.2,
                linvel=0.1,
            ),
        ),
        reward_config=config_dict.create(
            scales=config_dict.create(
                # Tracking
                tracking_angle=1.0,
                # Base reward
                lin_vel=-0.01,
                accel=-0.01,
                # Other
                # stand_still=-1.0,
                termination=-1.0,
                # pose=0.5,
                action_rate=0.0,  # -0.01,
                energy=0.0,  # -0.001,
                dof_pos_limits=-0.33,
                # Regularization.
                torques=0.0,  # -0.00001,
            ),
            tracking_sigma=0.25,
        ),
        termination_config=config_dict.create(max_rps=1.5, max_speed=20),
        pert_config=config_dict.create(
            enable=False,
            velocity_kick=[-0.5, 0.5],
            kick_durations=[0.05, 0.2],
            kick_wait_times=[1.0, 3.0],
        ),
        command_config=config_dict.create(
            # Uniform distribution for command amplitude (rpy)
            a=[3.14, 3.14, 3.14],
            # Probability of not zeroing out new command.
            b=0.8,
        ),
    )


class CrabEnv(mjx_env.MjxEnv):
    def __init__(
        self,
        xml_path=str(consts.XML_PATH),
        config: config_dict.ConfigDict = default_config(),
        config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
    ) -> None:
        super().__init__(config, config_overrides)

        self._mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self._mj_model.opt.timestep = self._config.sim_dt

        # Modify PD gains.
        self._mj_model.dof_damping[6:] = config.Kd
        self._mj_model.actuator_gainprm[:, 0] = config.Kp
        self._mj_model.actuator_biasprm[:, 1] = -config.Kp

        # Increase offscreen framebuffer size to render at higher resolutions.
        self._mj_model.vis.global_.offwidth = 3840
        self._mj_model.vis.global_.offheight = 2160

        self._mjx_model = mjx.put_model(self._mj_model)
        self._xml_path = str(xml_path)
        self._imu_site_id = self._mj_model.site("imu").id

        self._post_init()

    def _post_init(self) -> None:
        self._init_q = jp.array(self._mj_model.keyframe("home").qpos)
        self._default_pose = jp.array(self._mj_model.keyframe("home").qpos[7:])

        # Note: First joint is freejoint.
        self._lowers, self._uppers = self.mj_model.jnt_range[1:].T
        self._soft_lowers = self._lowers * self._config.soft_joint_pos_limit_factor
        self._soft_uppers = self._uppers * self._config.soft_joint_pos_limit_factor

        self._torso_body_id = self._mj_model.body(consts.ROOT_BODY).id
        self._torso_inertia = self._mj_model.body_inertia[self._torso_body_id]

        self._cmd_a = jp.array(self._config.command_config.a)
        self._cmd_b = self._config.command_config.b

    ###################
    # Sensor readings #
    ###################

    def get_global_linvel(self, data: mjx.Data) -> jax.Array:
        return mjx_env.get_sensor_data(self.mj_model, data, consts.GLOBAL_LINVEL_SENSOR)

    def get_global_angvel(self, data: mjx.Data) -> jax.Array:
        return mjx_env.get_sensor_data(self.mj_model, data, consts.GLOBAL_ANGVEL_SENSOR)

    def get_local_linvel(self, data: mjx.Data) -> jax.Array:
        return mjx_env.get_sensor_data(self.mj_model, data, consts.LOCAL_LINVEL_SENSOR)

    def get_accelerometer(self, data: mjx.Data) -> jax.Array:
        return mjx_env.get_sensor_data(self.mj_model, data, consts.ACCELEROMETER_SENSOR)

    def get_gyro(self, data: mjx.Data) -> jax.Array:
        return mjx_env.get_sensor_data(self.mj_model, data, consts.GYRO_SENSOR)

    def get_ori(self, data: mjx.Data) -> jax.Array:
        quat = mjx_env.get_sensor_data(self.mj_model, data, consts.ORIENTATION_SENSOR)
        return Rotation.from_quat(jp.roll(quat, -1, axis=-1)).as_euler("xyz")

    #############
    # Accessors #
    #############

    @property
    def xml_path(self) -> str:
        return self._xml_path

    @property
    def action_size(self) -> int:
        return self._mjx_model.nu

    @property
    def mj_model(self) -> mujoco.MjModel:
        return self._mj_model

    @property
    def mjx_model(self) -> mjx.Model:
        return self._mjx_model

    def reset(self, rng: jax.Array) -> mjx_env.State:
        qpos = self._init_q
        qvel = jp.zeros(self.mjx_model.nv)

        # d(xyzrpy)=U(-0.5, 0.5)
        rng, key = jax.random.split(rng)
        qvel = qvel.at[0:6].set(jax.random.uniform(key, (6,), minval=-0.5, maxval=0.5))

        data = mjx_env.init(self.mjx_model, qpos=qpos, qvel=qvel, ctrl=qpos[7:])

        rng, key1, key2, key3 = jax.random.split(rng, 4)
        time_until_next_pert = jax.random.uniform(
            key1,
            minval=self._config.pert_config.kick_wait_times[0],
            maxval=self._config.pert_config.kick_wait_times[1],
        )
        steps_until_next_pert = jp.round(time_until_next_pert / self.dt).astype(
            jp.int32
        )
        pert_duration_seconds = jax.random.uniform(
            key2,
            minval=self._config.pert_config.kick_durations[0],
            maxval=self._config.pert_config.kick_durations[1],
        )
        pert_duration_steps = jp.round(pert_duration_seconds / self.dt).astype(jp.int32)
        pert_mag = jax.random.uniform(
            key3,
            shape=(3,),
            minval=self._config.pert_config.velocity_kick[0],
            maxval=self._config.pert_config.velocity_kick[1],
        )

        rng, key1, key2 = jax.random.split(rng, 3)
        time_until_next_cmd = jax.random.exponential(key1) * 5.0
        steps_until_next_cmd = jp.round(time_until_next_cmd / self.dt).astype(jp.int32)
        cmd = jax.random.uniform(
            key2, shape=(3,), minval=-self._cmd_a, maxval=self._cmd_a
        )

        info = {
            "rng": rng,
            "command": cmd,
            "steps_until_next_cmd": steps_until_next_cmd,
            "last_act": jp.zeros(self.mjx_model.nu),
            "last_last_act": jp.zeros(self.mjx_model.nu),
            "steps_until_next_pert": steps_until_next_pert,
            "pert_duration_seconds": pert_duration_seconds,
            "pert_duration": pert_duration_steps,
            "steps_since_last_pert": 0,
            "pert_steps": 0,
            "pert_dir": jp.zeros(3),
            "pert_mag": pert_mag,
        }

        metrics = {}
        for k in self._config.reward_config.scales.keys():
            metrics[f"reward/{k}"] = jp.zeros(())
        metrics["swing_peak"] = jp.zeros(())

        obs = self._get_obs(data, info)
        reward, done = jp.zeros(2)
        return mjx_env.State(data, obs, reward, done, metrics, info)

    def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
        if self._config.pert_config.enable:
            state = self._maybe_apply_perturbation(state)

        motor_targets = action * self._config.action_scale
        data = mjx_env.step(self.mjx_model, state.data, motor_targets, self.n_substeps)

        obs = self._get_obs(data, state.info)
        done = self._get_termination(data)

        rewards = self._get_reward(data, action, state.info, state.metrics, done)
        rewards = {
            k: v * self._config.reward_config.scales[k] for k, v in rewards.items()
        }
        reward = jp.clip(sum(rewards.values()) * self.dt, 0.0, 10000.0)

        state.info["last_last_act"] = state.info["last_act"]
        state.info["last_act"] = action
        state.info["steps_until_next_cmd"] -= 1
        state.info["rng"], key1, key2 = jax.random.split(state.info["rng"], 3)
        state.info["command"] = jp.where(
            state.info["steps_until_next_cmd"] <= 0,
            self.sample_command(key1, state.info["command"]),
            state.info["command"],
        )
        state.info["steps_until_next_cmd"] = jp.where(
            done | (state.info["steps_until_next_cmd"] <= 0),
            jp.round(jax.random.exponential(key2) * 5.0 / self.dt).astype(jp.int32),
            state.info["steps_until_next_cmd"],
        )
        for k, v in rewards.items():
            state.metrics[f"reward/{k}"] = v

        done = done.astype(reward.dtype)
        state = state.replace(data=data, obs=obs, reward=reward, done=done)
        return state

    def _get_termination(self, data: mjx.Data) -> jax.Array:
        linvel = self.get_global_linvel(data)
        angvel = self.get_global_angvel(data)
        # terminate if going faster than dictated m/s or rot/s (3pi rad/s)
        return jp.any(
            jp.array(
                [
                    jp.linalg.norm(linvel) > self._config.termination_config.max_speed,
                    jp.linalg.norm(angvel)
                    > self._config.termination_config.max_rps * 2 * jp.pi,
                ]
            ),
            axis=0,
        )

    def _get_obs(self, data: mjx.Data, info: dict[str, Any]) -> Dict[str, jax.Array]:
        ori = self.get_ori(data)
        info["rng"], noise_rng = jax.random.split(info["rng"])
        noisy_ori = (
            ori
            + (2 * jax.random.uniform(noise_rng, shape=ori.shape) - 1)
            * self._config.noise_config.level
            * self._config.noise_config.scales.ori
        )

        gyro = self.get_gyro(data)
        info["rng"], noise_rng = jax.random.split(info["rng"])
        noisy_gyro = (
            gyro
            + (2 * jax.random.uniform(noise_rng, shape=gyro.shape) - 1)
            * self._config.noise_config.level
            * self._config.noise_config.scales.gyro
        )

        joint_angles = data.qpos[7:]
        info["rng"], noise_rng = jax.random.split(info["rng"])
        noisy_joint_angles = (
            joint_angles
            + (2 * jax.random.uniform(noise_rng, shape=joint_angles.shape) - 1)
            * self._config.noise_config.level
            * self._config.noise_config.scales.joint_pos
        )

        joint_vel = data.qvel[6:]
        info["rng"], noise_rng = jax.random.split(info["rng"])
        noisy_joint_vel = (
            joint_vel
            + (2 * jax.random.uniform(noise_rng, shape=joint_vel.shape) - 1)
            * self._config.noise_config.level
            * self._config.noise_config.scales.joint_vel
        )

        linvel = self.get_local_linvel(data)
        info["rng"], noise_rng = jax.random.split(info["rng"])
        noisy_linvel = (
            linvel
            + (2 * jax.random.uniform(noise_rng, shape=linvel.shape) - 1)
            * self._config.noise_config.level
            * self._config.noise_config.scales.linvel
        )

        state = jp.hstack(
            [
                noisy_ori,  # 3
                noisy_linvel,  # 3
                noisy_gyro,  # 3
                noisy_joint_angles,  # 12
                noisy_joint_vel,  # 12
                info["last_act"],  # 12
                info["command"],  # 3
            ]
        )

        accelerometer = self.get_accelerometer(data)
        angvel = self.get_global_angvel(data)

        privileged_state = jp.hstack(
            [
                state,
                ori,  # 3
                gyro,  # 3
                accelerometer,  # 3
                linvel,  # 3
                angvel,  # 3
                joint_angles,  # 12
                joint_vel,  # 12
                data.actuator_force,  # 12
                data.xfrc_applied[self._torso_body_id, 3:],  # 3
                info["steps_since_last_pert"] >= info["steps_until_next_pert"],  # 1
            ]
        )

        return {
            "state": state,
            "privileged_state": privileged_state,
        }

    def _get_reward(
        self,
        data: mjx.Data,
        action: jax.Array,
        info: dict[str, Any],
        metrics: dict[str, Any],
        done: jax.Array,
    ) -> dict[str, jax.Array]:
        del metrics  # Unused.
        return {
            "tracking_angle": self._reward_tracking_angle(
                info["command"], self.get_ori(data)
            ),
            "lin_vel": self._cost_lin_vel(self.get_global_linvel(data)),
            "accel": self._cost_accel(self.get_accelerometer(data)),
            # "stand_still": self._cost_stand_still(info["command"], data.qpos[7:]),
            "termination": self._cost_termination(done),
            # "pose": self._reward_pose(data.qpos[7:]),
            "torques": self._cost_torques(data.actuator_force),
            "action_rate": self._cost_action_rate(
                action, info["last_act"], info["last_last_act"]
            ),
            "energy": self._cost_energy(data.qvel[6:], data.actuator_force),
            "dof_pos_limits": self._cost_joint_pos_limits(data.qpos[7:]),
        }

    ####################
    # Tracking rewards #
    ####################

    def _reward_tracking_angle(
        self,
        commands: jax.Array,
        angle: jax.Array,
    ) -> jax.Array:
        return jp.linalg.norm(commands[:3] - angle)
        # return jp.exp(-angle_error / self._config.reward_config.tracking_sigma)

    ########################
    # Base-related rewards #
    ########################

    def _cost_lin_vel(self, global_linvel) -> jax.Array:
        # Penalize base linear velocity
        return jp.sum(jp.square(global_linvel[:3]))

    def _cost_accel(self, acceleration) -> jax.Array:
        return jp.sum(jp.square(acceleration))

    ##########################
    # Energy-related rewards #
    ##########################

    def _cost_torques(self, torques: jax.Array) -> jax.Array:
        # Penalize torques.
        return jp.sqrt(jp.sum(jp.square(torques))) + jp.sum(jp.abs(torques))

    def _cost_energy(self, qvel: jax.Array, qfrc_actuator: jax.Array) -> jax.Array:
        # Penalize energy consumption.
        return jp.sum(jp.abs(qvel) * jp.abs(qfrc_actuator))

    def _cost_action_rate(
        self, act: jax.Array, last_act: jax.Array, last_last_act: jax.Array
    ) -> jax.Array:
        del last_last_act  # Unused.
        return jp.sum(jp.square(act - last_act))

    #################
    # Other rewards #
    #################

    def _reward_pose(self, qpos: jax.Array) -> jax.Array:
        # Stay close to the default pose.
        weight = jp.array([1.0, 1.0, 0.1] * 4)
        return jp.exp(-jp.sum(jp.square(qpos) * weight))

    def _cost_stand_still(
        self,
        commands: jax.Array,
        qpos: jax.Array,
    ) -> jax.Array:
        cmd_norm = jp.linalg.norm(commands)
        return jp.sum(jp.abs(qpos)) * (cmd_norm < 0.01)

    def _cost_termination(self, done: jax.Array) -> jax.Array:
        # Penalize early termination.
        return done

    def _cost_joint_pos_limits(self, qpos: jax.Array) -> jax.Array:
        # Penalize joints if they cross soft limits.
        out_of_limits = -jp.clip(qpos - self._soft_lowers, None, 0.0)
        out_of_limits += jp.clip(qpos - self._soft_uppers, 0.0, None)
        return jp.sum(out_of_limits)

    ###################################
    # Perturbation & command sampling #
    ###################################

    def _maybe_apply_perturbation(self, state: mjx_env.State) -> mjx_env.State:
        def gen_dir(rng: jax.Array) -> jax.Array:
            angle = jax.random.uniform(rng, minval=0.0, maxval=jp.pi * 2)
            return jp.array([jp.cos(angle), jp.sin(angle), 0.0])

        def apply_pert(state: mjx_env.State) -> mjx_env.State:
            t = state.info["pert_steps"] * self.dt
            u_t = 0.5 * jp.sin(jp.pi * t / state.info["pert_duration_seconds"])
            # kg * m^2 * 1/s * 1/s = kg * m^2/s^2 = Nm
            torque = (  # (3,)
                u_t  # (unitless)
                * self._torso_inertia  # kg * m^2
                * state.info["pert_mag"]  # 1/s
                / state.info["pert_duration_seconds"]  # 1/s
            )
            xfrc_applied = jp.zeros((self.mjx_model.nbody, 6))
            xfrc_applied = xfrc_applied.at[self._torso_body_id, 3:].set(torque)
            data = state.data.replace(xfrc_applied=xfrc_applied)
            state = state.replace(data=data)
            state.info["steps_since_last_pert"] = jp.where(
                state.info["pert_steps"] >= state.info["pert_duration"],
                0,
                state.info["steps_since_last_pert"],
            )
            state.info["pert_steps"] += 1
            return state

        def wait(state: mjx_env.State) -> mjx_env.State:
            state.info["rng"], rng = jax.random.split(state.info["rng"])
            state.info["steps_since_last_pert"] += 1
            xfrc_applied = jp.zeros((self.mjx_model.nbody, 6))
            data = state.data.replace(xfrc_applied=xfrc_applied)
            state.info["pert_steps"] = jp.where(
                state.info["steps_since_last_pert"]
                >= state.info["steps_until_next_pert"],
                0,
                state.info["pert_steps"],
            )
            state.info["pert_dir"] = jp.where(
                state.info["steps_since_last_pert"]
                >= state.info["steps_until_next_pert"],
                gen_dir(rng),
                state.info["pert_dir"],
            )
            return state.replace(data=data)

        return jax.lax.cond(
            state.info["steps_since_last_pert"] >= state.info["steps_until_next_pert"],
            apply_pert,
            wait,
            state,
        )

    def sample_command(self, rng: jax.Array, x_k: jax.Array) -> jax.Array:
        rng, y_rng, w_rng, z_rng = jax.random.split(rng, 4)
        y_k = jax.random.uniform(
            y_rng, shape=(3,), minval=-self._cmd_a, maxval=self._cmd_a
        )
        z_k = jax.random.bernoulli(z_rng, self._cmd_b, shape=(3,))
        w_k = jax.random.bernoulli(w_rng, 0.5, shape=(3,))
        x_kp1 = x_k - w_k * (x_k - y_k * z_k)
        return x_kp1


def domain_randomize(model: mjx.Model, rng: jax.Array):
    @jax.vmap
    def rand_dynamics(rng):
        # Scale static friction: *U(0.9, 1.1).
        # rng, key = jax.random.split(rng)
        # frictionloss = model.dof_frictionloss[6:] * jax.random.uniform(
        #     key, shape=(model.nv - 6,), minval=0.9, maxval=1.1
        # )
        # dof_frictionloss = model.dof_frictionloss.at[6:].set(frictionloss)

        # Scale armature: *U(1.0, 1.05).
        rng, key = jax.random.split(rng)
        armature = model.dof_armature[6:] * jax.random.uniform(
            key, shape=(model.nv - 6,), minval=1.0, maxval=1.05
        )
        dof_armature = model.dof_armature.at[6:].set(armature)

        # Jitter center of mass positiion: +U(-0.05, 0.05).
        # rng, key = jax.random.split(rng)
        # dpos = jax.random.uniform(key, (3,), minval=-0.05, maxval=0.05)
        # body_ipos = model.body_ipos.at[consts.TORSO_BODY_ID].set(
        #     model.body_ipos[consts.TORSO_BODY_ID] + dpos
        # )

        # Scale all link masses: *U(0.9, 1.1).
        rng, key = jax.random.split(rng)
        dmass = jax.random.uniform(key, shape=(model.nbody,), minval=0.9, maxval=1.1)
        body_mass = model.body_mass.at[:].set(model.body_mass * dmass)

        # Add mass to torso: +U(-1.0, 1.0).
        rng, key = jax.random.split(rng)
        dmass = jax.random.uniform(key, minval=-1.0, maxval=1.0)
        body_mass = body_mass.at[consts.TORSO_BODY_ID].set(
            body_mass[consts.TORSO_BODY_ID] + dmass
        )

        # Jitter qpos0: +U(-0.05, 0.05).
        rng, key = jax.random.split(rng)
        qpos0 = model.qpos0
        qpos0 = qpos0.at[7:].set(
            qpos0[7:]
            + jax.random.uniform(key, shape=(model.nq - 7,), minval=-0.05, maxval=0.05)
        )

        return (
            body_mass,
            qpos0,
            dof_armature,
        )

    (
        body_mass,
        qpos0,
        dof_armature,
    ) = rand_dynamics(rng)

    in_axes = jax.tree_util.tree_map(lambda x: None, model)
    in_axes = in_axes.tree_replace(
        {
            "body_mass": 0,
            "qpos0": 0,
            "dof_armature": 0,
        }
    )

    model = model.tree_replace(
        {
            "body_mass": body_mass,
            "qpos0": qpos0,
            "dof_armature": dof_armature,
        }
    )

    return model, in_axes
