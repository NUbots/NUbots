import os
import torch
import warp as wp
import mujoco
import mujoco_warp as mjwarp

HERE = os.path.dirname(os.path.abspath(__file__))

class NugusEnv:
    def __init__(self, nenv: int, device: torch.device):
        self.nenv = nenv
        self.device = device
        
        # Warp initialization
        wp.init()
        self.wp_device = wp.get_device(str(device))
        
        # Load MuJoCo model
        xml_path = os.path.join(HERE, "mujoco", "scene.xml")
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        
        # We need implicitfast for better stability with RL
        self.mj_model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.mj_model.opt.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
        self.mj_model.opt.ls_iterations = 50
        
        # Initialize mujoco_warp
        self.mj_data = mujoco.MjData(self.mj_model)
        with wp.ScopedDevice(self.wp_device):
            self.wp_model = mjwarp.put_model(self.mj_model)
            self.wp_data = mjwarp.put_data(self.mj_model, self.mj_data, nworld=self.nenv)
            self.reset_mask_wp = wp.zeros(self.nenv, dtype=bool)
            
        # Bridge warp arrays to torch tensors
        self.qpos = wp.to_torch(self.wp_data.qpos)
        self.qvel = wp.to_torch(self.wp_data.qvel)
        self.ctrl = wp.to_torch(self.wp_data.ctrl)
        
        # We'll need contact forces, but mjwarp might not expose them simply.
        # For now, let's just stick to qpos/qvel.
        self.NQ = self.mj_model.nq
        self.NV = self.mj_model.nv
        self.NU = self.mj_model.nu
        
        # RL specific states
        self.commands = torch.zeros((self.nenv, 3), dtype=torch.float32, device=self.device)
        self.last_actions = torch.zeros((self.nenv, self.NU), dtype=torch.float32, device=self.device)
        self.phase = torch.zeros(self.nenv, dtype=torch.float32, device=self.device)
        self.phase_indicator = torch.ones(self.nenv, dtype=torch.float32, device=self.device)
        self.step_period = 0.32
        
        # Teacher states
        self.history = torch.zeros((self.nenv, 3, 12), dtype=torch.float32, device=self.device)
        
        # Default standing pose
        key_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_KEY, "stand_bent_knees")
        if key_id != -1:
            self.default_qpos = torch.tensor(self.mj_model.key_qpos[key_id], dtype=torch.float32, device=self.device)
            self.default_ctrl = torch.tensor(self.mj_model.key_ctrl[key_id], dtype=torch.float32, device=self.device)
        else:
            self.default_qpos = torch.zeros(self.NQ, dtype=torch.float32, device=self.device)
            self.default_ctrl = torch.zeros(self.NU, dtype=torch.float32, device=self.device)
            
        # Joint limits
        jnt_ranges = torch.tensor(self.mj_model.jnt_range[1:], dtype=torch.float32, device=self.device)
        self.joint_pos_lower = jnt_ranges[:, 0]
        self.joint_pos_upper = jnt_ranges[:, 1]
        
    def reset_done(self, done: torch.Tensor):
        if not done.any():
            return
            
        reset_mask = done.bool()
        self.reset_mask_wp = wp.from_torch(reset_mask.contiguous())
        
        with wp.ScopedDevice(self.wp_device):
            mjwarp.reset_data(self.wp_model, self.wp_data, reset=self.reset_mask_wp)
            
        # After reset, set the qpos to the default standing pose
        self.qpos[reset_mask] = self.default_qpos
        self.qvel[reset_mask] = 0.0
        self.ctrl[reset_mask] = self.default_ctrl
        self.last_actions[reset_mask] = 0.0
        self.phase[reset_mask] = 0.0
        self.phase_indicator[reset_mask] = 1.0
        
        # Reset history to standing targets
        stand_targets = self.default_ctrl[:12]
        self.history[reset_mask, 0] = stand_targets
        self.history[reset_mask, 1] = stand_targets
        self.history[reset_mask, 2] = stand_targets
        
        # Need to re-forward to update kinematics after manually changing qpos
        with wp.ScopedDevice(self.wp_device):
            mjwarp.forward(self.wp_model, self.wp_data)
            
    def reset_all(self):
        done = torch.ones(self.nenv, dtype=torch.bool, device=self.device)
        self.reset_done(done)
        
    def step(self, teacher_targets: torch.Tensor, student_actions: torch.Tensor, alpha: float = 0.2, nsub: int = 5, dt: float = 0.01):
        # Teacher targets (12) + student residuals (12)
        final_targets = teacher_targets + alpha * torch.tanh(student_actions)
        
        self.ctrl[:, :12] = final_targets
        self.last_actions.copy_(self.ctrl)
        
        # Update history
        self.history[:, 2] = self.history[:, 1].clone()
        self.history[:, 1] = self.history[:, 0].clone()
        self.history[:, 0] = final_targets.clone()
        
        with wp.ScopedDevice(self.wp_device):
            for _ in range(nsub):
                mjwarp.step(self.wp_model, self.wp_data)
                
        # Advance phase
        self.phase += dt
        
        switches = (self.phase >= self.step_period)
        self.phase[switches] -= self.step_period
        self.phase_indicator[switches] *= -1.0

    def get_teacher_obs(self) -> torch.Tensor:
        phase_ratio = self.phase / self.step_period
        phase_sin = torch.sin(2.0 * torch.pi * phase_ratio)
        phase_cos = torch.cos(2.0 * torch.pi * phase_ratio)
        
        # Engine state one-hot: walking is 2
        state_onehot = torch.zeros((self.nenv, 4), dtype=torch.float32, device=self.device)
        state_onehot[:, 2] = 1.0
        
        obs = torch.cat([
            self.commands,
            phase_sin.unsqueeze(1),
            phase_cos.unsqueeze(1),
            self.phase_indicator.unsqueeze(1),
            state_onehot,
            self.history[:, 0],
            self.history[:, 1],
            self.history[:, 2]
        ], dim=1)
        
        return obs

    def get_obs(self) -> torch.Tensor:
        # Base quaternion q = [w, x, y, z]
        q = self.qpos[:, 3:7]
        qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        
        # Inverse vector part
        q_vec_inv = torch.stack([-qx, -qy, -qz], dim=1)
        
        def quat_rotate_inverse(v):
            uv = torch.cross(q_vec_inv, v, dim=1)
            uuv = torch.cross(q_vec_inv, uv, dim=1)
            return v + 2.0 * (qw.unsqueeze(1) * uv + uuv)
            
        global_gravity = torch.zeros((self.nenv, 3), dtype=torch.float32, device=self.device)
        global_gravity[:, 2] = -1.0
        
        proj_gravity = quat_rotate_inverse(global_gravity)
        base_lin_vel = quat_rotate_inverse(self.qvel[:, 0:3])
        base_ang_vel = quat_rotate_inverse(self.qvel[:, 3:6])
        
        # Assemble observation
        obs = torch.cat([
            self.commands,                     # 3
            proj_gravity,                      # 3
            base_lin_vel,                      # 3
            base_ang_vel,                      # 3
            self.qpos[:, 7:] - self.default_qpos[7:],  # 20 (or 40 with backlash)
            self.qvel[:, 6:],                  # 20 (or 40 with backlash)
            self.last_actions                  # 20
        ], dim=1)
        
        return obs
