# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Joystick for Nugus."""

from typing import Any, Dict, Optional, Union

import jax
import jax.numpy as jp
from ml_collections import config_dict
from mujoco import mjx
import numpy as np

from mujoco_playground._src import collision
from mujoco_playground._src import mjx_env
from mujoco_playground._src.locomotion.nugus import base as nugus_base
from mujoco_playground._src.locomotion.nugus import nugus_constants as consts
from mujoco_playground._src import gait


def default_config() -> config_dict.ConfigDict:
    return config_dict.create(
        ctrl_dt=0.02,
        sim_dt=0.004,
        episode_length=1000,
        Kp=21.1,
        Kd=1.084,
        early_termination=True,
        action_repeat=1,
        action_scale=0.3,
        obs_noise=0.05,
        obs_history_size=3,
        max_foot_height=0.07,
        lin_vel_x=[-0.6, 1.5],
        lin_vel_y=[-0.8, 0.8],
        ang_vel_yaw=[-0.7, 0.7],
        reward_config=config_dict.create(
            scales=config_dict.create(
                tracking_lin_vel=1.5,
                tracking_ang_vel=0.8,
                lin_vel_z=-2.0,
                ang_vel_xy=-0.05,
                orientation=-5.0,
                torques=-0.0002,
                action_rate=-0.01,
                zero_cmd=-0.5,
                termination=-1.0,
                feet_slip=-0.1,
                feet_clearance=0.0,
                energy=-0.0001,
                feet_air_time=2.0,
                feet_height=0.0,
                feet_phase=1.0,
                feet_distance=-1.0,
            ),
            tracking_sigma=0.25,
            max_foot_height=0.07,
        ),
        velocity_kick=[1.0, 5.0],
        kick_durations=[0.05, 0.2],
        kick_wait_times=[1.0, 3.0],
    )


class Joystick(nugus_base.NugusEnv):
    """Track a joystick command."""

    def __init__(
        self,
        config: config_dict.ConfigDict = default_config(),
        config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
    ):
        super().__init__(
            xml_path=str(consts.FEET_ONLY_XML),
            config=config,
            config_overrides=config_overrides,
        )
        self._post_init()

    def _post_init(self) -> None:
        self._init_q = jp.array(self._mj_model.keyframe("stand_bent_knees").qpos)
        self._default_pose = self._mj_model.keyframe("stand_bent_knees").qpos[7:]
        self._lowers = self._mj_model.actuator_ctrlrange[:, 0]
        self._uppers = self._mj_model.actuator_ctrlrange[:, 1]

        self._torso_body_id = self._mj_model.body(consts.ROOT_BODY).id
        self._torso_mass = self._mj_model.body_subtreemass[self._torso_body_id]

        self._feet_site_id = np.array(
            [self._mj_model.site(name).id for name in consts.FEET_SITES]
        )
        self._floor_geom_id = self._mj_model.geom("floor").id

        self._left_feet_geom_id = np.array(
            [self._mj_model.geom(name).id for name in consts.LEFT_FEET_GEOMS]
        )
        self._right_feet_geom_id = np.array(
            [self._mj_model.geom(name).id for name in consts.RIGHT_FEET_GEOMS]
        )

        foot_linvel_sensor_adr = []
        for site in consts.FEET_SITES:
            sensor_id = self._mj_model.sensor(f"{site}_global_linvel").id
            sensor_adr = self._mj_model.sensor_adr[sensor_id]
            sensor_dim = self._mj_model.sensor_dim[sensor_id]
            foot_linvel_sensor_adr.append(
                list(range(sensor_adr, sensor_adr + sensor_dim))
            )
        self._foot_linvel_sensor_adr = jp.array(foot_linvel_sensor_adr)

    def reset(self, rng: jax.Array) -> mjx_env.State:
        rng, cmd_rng, noise_rng, pert1_rng, pert2_rng, pert3_rng, phase_rng = (
            jax.random.split(rng, 7)
        )

        data = mjx_env.init(
            self.mjx_model, qpos=self._init_q, qvel=jp.zeros(self.mjx_model.nv)
        )

        # For feet_air_time and feet_phase rewards
        gait_freq = jax.random.uniform(phase_rng, (1,), minval=1.25, maxval=1.75)
        phase_dt = 2 * jp.pi * self.dt * gait_freq
        phase = jp.array([0, jp.pi])

        time_until_next_pert = jax.random.uniform(
            pert1_rng,
            minval=self._config.kick_wait_times[0],
            maxval=self._config.kick_wait_times[1],
        )
        steps_until_next_pert = jp.round(time_until_next_pert / self.dt).astype(
            jp.int32
        )
        pert_duration_seconds = jax.random.uniform(
            pert2_rng,
            minval=self._config.kick_durations[0],
            maxval=self._config.kick_durations[1],
        )
        pert_duration_steps = jp.round(pert_duration_seconds / self.dt).astype(jp.int32)
        vel_kick = jax.random.uniform(
            pert3_rng,
            minval=self._config.velocity_kick[0],
            maxval=self._config.velocity_kick[1],
        )

        info = {
            "rng": rng,
            "last_act": jp.zeros(self.mjx_model.nu),
            "last_last_act": jp.zeros(self.mjx_model.nu),
            "last_vel": jp.zeros(self.mjx_model.nv - 6),
            "command": self.sample_command(cmd_rng),
            "step": 0,
            "steps_until_next_pert": steps_until_next_pert,
            "pert_duration_seconds": pert_duration_seconds,
            "pert_duration": pert_duration_steps,
            "steps_since_last_pert": 0,
            "pert_steps": 0,
            "direction": jp.zeros(3),
            "vel_kick": vel_kick,
            "motor_targets": jp.zeros(self.mjx_model.nu),
            # Add variables for feet-related rewards
            "feet_air_time": jp.zeros(2),
            "last_contact": jp.zeros(2, dtype=bool),
            "swing_peak": jp.zeros(2),
            # Phase related
            "phase_dt": phase_dt,
            "phase": phase,
        }

        metrics = {}
        for k in self._config.reward_config.scales.keys():
            metrics[f"reward/{k}"] = jp.zeros(())

        obs_history = jp.zeros(self._config.obs_history_size * 49)
        obs = self._get_obs(data, info, obs_history, noise_rng)
        reward, done = jp.zeros(2)
        return mjx_env.State(data, obs, reward, done, metrics, info)

    def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
        rng, cmd_rng, noise_rng = jax.random.split(state.info["rng"], 3)

        # state = self._maybe_apply_perturbation(state, pert_rng)

        motor_targets = self._default_pose + action * self._config.action_scale
        motor_targets = jp.clip(motor_targets, self._lowers, self._uppers)
        data = mjx_env.step(self.mjx_model, state.data, motor_targets, self.n_substeps)

        # Get feet contact information
        left_feet_contact = jp.array(
            [
                collision.geoms_colliding(data, geom_id, self._floor_geom_id)
                for geom_id in self._left_feet_geom_id
            ]
        )
        right_feet_contact = jp.array(
            [
                collision.geoms_colliding(data, geom_id, self._floor_geom_id)
                for geom_id in self._right_feet_geom_id
            ]
        )
        contact = jp.hstack([jp.any(left_feet_contact), jp.any(right_feet_contact)])
        contact_filt = contact | state.info["last_contact"]
        first_contact = (state.info["feet_air_time"] > 0.0) * contact_filt

        # Update feet air time
        state.info["feet_air_time"] += self.dt

        # Track swing peak height
        p_f = data.site_xpos[self._feet_site_id]
        p_fz = p_f[..., -1]
        state.info["swing_peak"] = jp.maximum(state.info["swing_peak"], p_fz)

        obs = self._get_obs(data, state.info, state.obs, noise_rng)
        done = self._get_termination(data)

        rewards = self._get_reward(
            data, action, state.info, state.metrics, done, first_contact, contact
        )
        rewards = {
            k: v * self._config.reward_config.scales[k] for k, v in rewards.items()
        }
        reward = jp.clip(sum(rewards.values()) * self.dt, 0.0, 10000.0)

        # Bookkeeping.
        state.info["motor_targets"] = motor_targets
        state.info["last_last_act"] = state.info["last_act"]
        state.info["last_act"] = action
        state.info["last_vel"] = data.qvel[6:]
        state.info["step"] += 1
        state.info["rng"] = rng
        state.info["command"] = jp.where(
            state.info["step"] > 500,
            self.sample_command(cmd_rng),
            state.info["command"],
        )
        state.info["step"] = jp.where(
            done | (state.info["step"] > 500),
            0,
            state.info["step"],
        )

        # Update phase
        phase_tp1 = state.info["phase"] + state.info["phase_dt"]
        state.info["phase"] = jp.fmod(phase_tp1 + jp.pi, 2 * jp.pi) - jp.pi
        state.info["phase"] = jp.where(
            jp.linalg.norm(state.info["command"]) > 0.01,
            state.info["phase"],
            jp.ones(2) * jp.pi,
        )

        # Update feet tracking variables
        state.info["feet_air_time"] *= ~contact
        state.info["last_contact"] = contact
        state.info["swing_peak"] *= ~contact

        for k, v in rewards.items():
            state.metrics[f"reward/{k}"] = v

        done = jp.float32(done)
        state = state.replace(data=data, obs=obs, reward=reward, done=done)
        return state

    def _get_termination(self, data: mjx.Data) -> jax.Array:
        joint_angles = data.qpos[7:]
        torso_z = data.xpos[self._torso_body_id, -1]

        # Joint limits exceeded.
        joint_limit_exceed = jp.any(joint_angles < self._lowers)
        joint_limit_exceed |= jp.any(joint_angles > self._uppers)

        # Fall detection.
        fall_termination = self.get_gravity(data)[-1] < 0.85
        fall_termination |= torso_z < 0.21

        return jp.where(
            self._config.early_termination,
            joint_limit_exceed | fall_termination,
            joint_limit_exceed,
        )

    def _get_obs(
        self,
        data: mjx.Data,
        info: dict[str, Any],
        obs_history: jax.Array,
        rng: jax.Array,
    ) -> jax.Array:
        obs = jp.concatenate(
            [
                self.get_gyro(data),  # 3
                self.get_gravity(data),  # 3
                info["command"],  # 3
                data.qpos[7:] - self._default_pose,  # 20
                info["last_act"],  # 20
            ]
        )  # total = 49
        if self._config.obs_noise >= 0.0:
            noise = self._config.obs_noise * jax.random.uniform(
                rng, obs.shape, minval=-1.0, maxval=1.0
            )
            obs = jp.clip(obs, -100.0, 100.0) + noise
        obs = jp.roll(obs_history, obs.size).at[: obs.size].set(obs)
        return obs

    def _get_reward(
        self,
        data: mjx.Data,
        action: jax.Array,
        info: dict[str, Any],
        metrics: dict[str, Any],
        done: jax.Array,
        first_contact: jax.Array = None,
        contact: jax.Array = None,
    ) -> dict[str, jax.Array]:
        del metrics  # Unused.
        rewards = {
            # Tracking rewards.
            "tracking_lin_vel": self._reward_tracking_lin_vel(
                info["command"], self.get_local_linvel(data)
            ),
            "tracking_ang_vel": self._reward_tracking_ang_vel(
                info["command"], self.get_gyro(data)
            ),
            # Regularization rewards.
            "lin_vel_z": self._cost_lin_vel_z(self.get_global_linvel(data)),
            "ang_vel_xy": self._cost_ang_vel_xy(self.get_global_angvel(data)),
            "orientation": self._cost_orientation(self.get_gravity(data)),
            "torques": self._cost_torques(data.actuator_force),
            "action_rate": self._cost_action_rate(
                action, info["last_act"], info["last_last_act"]
            ),
            "zero_cmd": self._cost_zero_cmd(
                info["command"],
                self.get_global_linvel(data),
                self.get_global_angvel(data),
                action,
            ),
            "termination": self._cost_termination(done, info["step"]),
            "feet_slip": self._cost_feet_slip(data),
            "feet_clearance": self._cost_feet_clearance(data),
            "energy": self._cost_energy(data.qvel[6:], data.actuator_force),
        }

        # Add T1 feet-related rewards if contact info is provided
        if first_contact is not None and contact is not None:
            rewards.update(
                {
                    "feet_air_time": self._reward_feet_air_time(
                        info["feet_air_time"], first_contact, info["command"]
                    ),
                    "feet_height": self._cost_feet_height(
                        info["swing_peak"], first_contact, info
                    ),
                    "feet_phase": self._reward_feet_phase(
                        data,
                        info["phase"],
                        self._config.reward_config.max_foot_height,
                        info["command"],
                    ),
                    "feet_distance": self._cost_feet_distance(data, info),
                }
            )

        return rewards

    def _reward_tracking_lin_vel(
        self,
        commands: jax.Array,
        local_vel: jax.Array,
    ) -> jax.Array:
        # Tracking of linear velocity commands (xy axes).
        lin_vel_error = jp.sum(jp.square(commands[:2] - local_vel[:2]))
        reward = jp.exp(-lin_vel_error / self._config.reward_config.tracking_sigma)
        return reward

    def _reward_tracking_ang_vel(
        self,
        commands: jax.Array,
        ang_vel: jax.Array,
    ) -> jax.Array:
        # Tracking of angular velocity commands (yaw).
        ang_vel_error = jp.square(commands[2] - ang_vel[2])
        return jp.exp(-ang_vel_error / self._config.reward_config.tracking_sigma)

    def _cost_lin_vel_z(self, global_linvel) -> jax.Array:
        # Penalize z axis base linear velocity.
        return jp.square(global_linvel[2])

    def _cost_ang_vel_xy(self, global_angvel) -> jax.Array:
        # Penalize xy axes base angular velocity.
        return jp.sum(jp.square(global_angvel[:2]))

    def _cost_orientation(self, torso_zaxis: jax.Array) -> jax.Array:
        # Penalize non flat base orientation.
        return jp.sum(jp.square(torso_zaxis[:2]))

    def _cost_torques(self, torques: jax.Array) -> jax.Array:
        # Penalize torques.
        return jp.sqrt(jp.sum(jp.square(torques))) + jp.sum(jp.abs(torques))

    def _cost_energy(self, qvel: jax.Array, qfrc_actuator: jax.Array) -> jax.Array:
        # Penalize energy consumption.
        return jp.sum(jp.abs(qvel) * jp.abs(qfrc_actuator))

    def _cost_action_rate(
        self, act: jax.Array, last_act: jax.Array, last_last_act: jax.Array
    ) -> jax.Array:
        # Penalize first and second derivative of actions.
        c1 = jp.sum(jp.square(act - last_act))
        c2 = jp.sum(jp.square(act - 2 * last_act + last_last_act))
        return c1 + c2

    def _cost_zero_cmd(
        self,
        commands: jax.Array,
        global_linvel: jax.Array,
        ang_vel: jax.Array,
        action: jax.Array,
    ) -> jax.Array:
        del global_linvel, ang_vel  # Unused.
        cmd_norm = jp.linalg.norm(commands)
        penalty = jp.sum(jp.square(action))
        return penalty * (cmd_norm < 0.1)
        # penalty = jp.sum(jp.square(global_linvel[:2]))
        # cmd_norm = jp.linalg.norm(commands[:2])
        # cost_linvel = penalty * (cmd_norm < 0.1)
        # penalty = jp.sum(jp.square(ang_vel[:2]))
        # cmd_norm = jp.linalg.norm(commands[2])
        # cost_angvel = penalty * (cmd_norm < 0.1)
        # return cost_linvel + cost_angvel

    def _cost_termination(self, done: jax.Array, step: jax.Array) -> jax.Array:
        return done & (step < 500)

    def _cost_feet_slip(self, data: mjx.Data) -> jax.Array:
        feet_vel = data.sensordata[self._foot_linvel_sensor_adr]
        vel_xy = feet_vel[..., :2]
        vel_xy_norm_sq = jp.sum(jp.square(vel_xy), axis=-1)

        left_feet_contact = jp.array(
            [
                collision.geoms_colliding(data, geom_id, self._floor_geom_id)
                for geom_id in self._left_feet_geom_id
            ]
        )
        right_feet_contact = jp.array(
            [
                collision.geoms_colliding(data, geom_id, self._floor_geom_id)
                for geom_id in self._right_feet_geom_id
            ]
        )
        feet_contact = jp.hstack(
            [jp.any(left_feet_contact), jp.any(right_feet_contact)]
        )
        return jp.sum(vel_xy_norm_sq * feet_contact)

    def _cost_feet_clearance(self, data: mjx.Data) -> jax.Array:
        feet_vel = data.sensordata[self._foot_linvel_sensor_adr]
        vel_xy = feet_vel[..., :2]
        vel_norm = jp.sqrt(jp.linalg.norm(vel_xy, axis=-1))
        # vel_norm = jp.linalg.norm(vel_xy, axis=-1)
        foot_pos = data.site_xpos[self._feet_site_id]
        foot_z = foot_pos[..., -1]
        delta = (foot_z - self._config.max_foot_height) ** 2
        return jp.sum(delta * vel_norm)

    # TODO(kevin): Rewrite this ugly function.
    def _maybe_apply_perturbation(
        self, state: mjx_env.State, rng: jax.Array
    ) -> mjx_env.State:
        def gen_dir(rng: jax.Array) -> jax.Array:
            angle = jax.random.uniform(rng, minval=0.0, maxval=jp.pi * 2)
            return jp.array([jp.cos(angle), jp.sin(angle), 0.0])

        def apply_pert(state: mjx_env.State) -> mjx_env.State:
            t = state.info["pert_steps"] * self.dt
            u_t = 0.5 * jp.sin(jp.pi * t / state.info["pert_duration_seconds"])
            force = (
                u_t
                * self._torso_mass
                * state.info["vel_kick"]
                / state.info["pert_duration_seconds"]
            )
            xfrc_applied = jp.zeros((self.mjx_model.nbody, 6))
            xfrc_applied = xfrc_applied.at[self._torso_body_id, :3].set(
                force * state.info["direction"]
            )
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
            state.info["steps_since_last_pert"] += 1
            xfrc_applied = jp.zeros((self.mjx_model.nbody, 6))
            data = state.data.replace(xfrc_applied=xfrc_applied)
            state.info["pert_steps"] = jp.where(
                state.info["steps_since_last_pert"]
                >= state.info["steps_until_next_pert"],
                0,
                state.info["pert_steps"],
            )
            state.info["direction"] = jp.where(
                state.info["steps_since_last_pert"]
                >= state.info["steps_until_next_pert"],
                gen_dir(rng),
                state.info["direction"],
            )
            return state.replace(data=data)

        return jax.lax.cond(
            state.info["steps_since_last_pert"] >= state.info["steps_until_next_pert"],
            apply_pert,
            wait,
            state,
        )

    def sample_command(self, rng: jax.Array) -> jax.Array:
        """Samples a random command with a 10% chance of being zero."""
        # _, rng1, rng2, rng3, rng4 = jax.random.split(rng, 5)

        # lin_vel_x = jax.random.uniform(
        #     rng1, minval=self._config.lin_vel_x[0], maxval=self._config.lin_vel_x[1]
        # )
        # lin_vel_y = jax.random.uniform(
        #     rng2, minval=self._config.lin_vel_y[0], maxval=self._config.lin_vel_y[1]
        # )
        # ang_vel_yaw = jax.random.uniform(
        #     rng3,
        #     minval=self._config.ang_vel_yaw[0],
        #     maxval=self._config.ang_vel_yaw[1],
        # )

        # cmd = jp.hstack([lin_vel_x, lin_vel_y, ang_vel_yaw])
        # return jp.where(jax.random.bernoulli(rng4, 0.1), jp.zeros(3), cmd)
        return jp.array([0.5, 0.0, 0.0])

    def _cost_feet_height(
        self,
        swing_peak: jax.Array,
        first_contact: jax.Array,
        info: dict[str, Any],
    ) -> jax.Array:
        """Penalize feet not reaching target height during swing."""
        del info  # Unused.
        error = swing_peak / self._config.reward_config.max_foot_height - 1.0
        return jp.sum(jp.square(error) * first_contact)

    def _reward_feet_air_time(
        self,
        air_time: jax.Array,
        first_contact: jax.Array,
        commands: jax.Array,
        threshold_min: float = 0.2,
        threshold_max: float = 0.5,
    ) -> jax.Array:
        """Reward feet being in the air for an appropriate amount of time."""
        cmd_norm = jp.linalg.norm(commands)
        air_time = (air_time - threshold_min) * first_contact
        air_time = jp.clip(air_time, max=threshold_max - threshold_min)
        reward = jp.sum(air_time)
        reward *= cmd_norm > 0.1  # No reward for zero commands.
        return reward

    def _reward_feet_phase(
        self,
        data: mjx.Data,
        phase: jax.Array,
        foot_height: jax.Array,
        commands: jax.Array,
    ) -> jax.Array:
        """Reward for tracking the desired foot height based on phase."""
        foot_pos = data.site_xpos[self._feet_site_id]
        foot_z = foot_pos[..., -1]
        rz = gait.get_rz(phase, swing_height=foot_height)
        error = jp.sum(jp.square(foot_z - rz))
        reward = jp.exp(-error / 0.01)
        return reward

    def _cost_feet_distance(self, data: mjx.Data, info: dict[str, Any]) -> jax.Array:
        """Penalize feet being too close together."""
        del info  # Unused.
        left_foot_pos = data.site_xpos[self._feet_site_id[0]]
        right_foot_pos = data.site_xpos[self._feet_site_id[1]]

        # Get the base orientation to measure distance in the robot's frame
        base_xmat = data.site_xmat[self._mj_model.site("imu").id]
        base_yaw = jp.arctan2(base_xmat[1, 0], base_xmat[0, 0])

        # Calculate the lateral distance between feet in the robot's frame
        feet_distance = jp.abs(
            jp.cos(base_yaw) * (left_foot_pos[1] - right_foot_pos[1])
            - jp.sin(base_yaw) * (left_foot_pos[0] - right_foot_pos[0])
        )

        # Penalize if feet are too close
        return jp.clip(0.2 - feet_distance, min=0.0, max=0.1)
