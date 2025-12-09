"""
Environment used to train a stop-and-go dissipating controller.

This is the environment that was used in:

C. Wu, A. Kreidieh, K. Parvate, E. Vinitsky, A. Bayen, "Flow: Architecture and
Benchmarking for Reinforcement Learning in Traffic Control," CoRR, vol.
abs/1710.05465, 2017. [Online]. Available: https://arxiv.org/abs/1710.05465
"""

from flow.core.params import InitialConfig
from flow.core.params import NetParams
from flow.envs.base import Env

from gym.spaces.box import Box

from copy import deepcopy
import numpy as np
import random
from scipy.optimize import fsolve

ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration of autonomous vehicles
    'max_accel': 1,
    # maximum deceleration of autonomous vehicles
    'max_decel': 1,
    # bounds on the ranges of ring road lengths the autonomous vehicle is
    # trained on
    'ring_length': [220, 270],
}


def v_eq_max_function(v, *args):
    """Return the error between the desired and actual equivalent gap."""
    num_vehicles, length = args

    # maximum gap in the presence of one rl vehicle
    s_eq_max = (length - num_vehicles * 5) / (num_vehicles - 1)

    v0 = 30
    s0 = 2
    tau = 1
    gamma = 4

    error = s_eq_max - (s0 + v * tau) * (1 - (v / v0) ** gamma) ** -0.5

    return error


class WaveAttenuationEnv(Env):
    """Fully observable wave attenuation environment.

    This environment is used to train autonomous vehicles to attenuate the
    formation and propagation of waves in a variable density ring road.

    Required from env_params:

    * max_accel: maximum acceleration of autonomous vehicles
    * max_decel: maximum deceleration of autonomous vehicles
    * ring_length: bounds on the ranges of ring road lengths the autonomous
      vehicle is trained on. If set to None, the environment sticks to the ring
      road specified in the original network definition.

    States
        The state consists of the velocities and absolute position of all
        vehicles in the network. This assumes a constant number of vehicles.

    Actions
        Actions are a list of acceleration for each rl vehicles, bounded by the
        maximum accelerations and decelerations specified in EnvParams.

    Rewards
        The reward function rewards high average speeds from all vehicles in
        the network, and penalizes accelerations by the rl vehicle.

    Termination
        A rollout is terminated if the time horizon is reached or if two
        vehicles collide into one another.
    """

    def __init__(self, env_params, sim_params, network, simulator='traci'):
        for p in ADDITIONAL_ENV_PARAMS.keys():
            if p not in env_params.additional_params:
                raise KeyError(
                    'Environment parameter \'{}\' not supplied'.format(p))

        super().__init__(env_params, sim_params, network, simulator)

    @property
    def action_space(self):
        """See class definition."""
        return Box(
            low=-np.abs(self.env_params.additional_params['max_decel']),
            high=self.env_params.additional_params['max_accel'],
            shape=(self.initial_vehicles.num_rl_vehicles, ),
            dtype=np.float32)

    @property
    def observation_space(self):
        """See class definition."""
        self.obs_var_labels = ["Velocity", "Absolute_pos"]
        return Box(
            low=0,
            high=1,
            shape=(2 * self.initial_vehicles.num_vehicles, ),
            dtype=np.float32)

    def _apply_rl_actions(self, rl_actions):
        """See class definition."""
        self.k.vehicle.apply_acceleration(
            self.k.vehicle.get_rl_ids(), rl_actions)

    def compute_reward(self, rl_actions, **kwargs):
        """See class definition."""
        # in the warmup steps
        if rl_actions is None:
            return 0

        vel = np.array([
            self.k.vehicle.get_speed(veh_id)
            for veh_id in self.k.vehicle.get_ids()
        ])

        if any(vel < -100) or kwargs['fail']:
            return 0.

        # reward average velocity
        eta_2 = 4.
        reward = eta_2 * np.mean(vel) / 20

        # punish accelerations (should lead to reduced stop-and-go waves)
        eta = 4  # 0.25
        mean_actions = np.mean(np.abs(np.array(rl_actions)))
        accel_threshold = 0

        if mean_actions > accel_threshold:
            reward += eta * (accel_threshold - mean_actions)

        return float(reward)

    def get_state(self):
        """See class definition."""
        speed = [self.k.vehicle.get_speed(veh_id) / self.k.network.max_speed()
                 for veh_id in self.k.vehicle.get_ids()]
        pos = [self.k.vehicle.get_x_by_id(veh_id) / self.k.network.length()
               for veh_id in self.k.vehicle.get_ids()]

        return np.array(speed + pos)

    def additional_command(self):
        """Define which vehicles are observed for visualization purposes."""
        # specify observed vehicles
        if self.k.vehicle.num_rl_vehicles > 0:
            for veh_id in self.k.vehicle.get_human_ids():
                self.k.vehicle.set_observed(veh_id)

    def reset(self):
        """See parent class.

        The sumo instance is reset with a new ring length, and a number of
        steps are performed with the rl vehicle acting as a human vehicle.
        """
        # skip if ring length is None
        if self.env_params.additional_params['ring_length'] is None:
            return super().reset()

        # reset the step counter
        self.step_counter = 0
        self.observe = {}
        # update the network
        initial_config = InitialConfig(bunching=50, min_gap=0)
        length = self.env_params.additional_params.get('ring_length', 250)
        # length = random.randint(
        #     self.env_params.additional_params['ring_length'][0],
        #     self.env_params.additional_params['ring_length'][1])
        additional_net_params = {
            'length':
                length,
            'lanes':
                self.net_params.additional_params['lanes'],
            'speed_limit':
                self.net_params.additional_params['speed_limit'],
            'resolution':
                self.net_params.additional_params['resolution']
        }
        net_params = NetParams(additional_params=additional_net_params)

        self.network = self.network.__class__(
            self.network.orig_name, self.network.vehicles,
            net_params, initial_config)
        self.k.vehicle = deepcopy(self.initial_vehicles)
        self.k.vehicle.kernel_api = self.k.kernel_api
        self.k.vehicle.master_kernel = self.k

        # solve for the velocity upper bound of the ring
        v_guess = 10
        v_eq_max = fsolve(v_eq_max_function, np.array(v_guess),
                          args=(len(self.initial_ids)/2, length))[0]

        print('\n-----------------------')
        print('ring length:', net_params.additional_params['length'])
        print('v_max:', v_eq_max)
        print('-----------------------')

        # restart the sumo instance
        self.restart_simulation(
            sim_params=self.sim_params,
            render=self.sim_params.render)

        # perform the generic reset function
        return super().reset()


class WaveAttenuationPOEnv(WaveAttenuationEnv):
    """POMDP version of WaveAttenuationEnv.
        2 vehicles centralized
    Note that this environment only works when there is one autonomous vehicle
    on the network.

    Required from env_params:

    * max_accel: maximum acceleration of autonomous vehicles
    * max_decel: maximum deceleration of autonomous vehicles
    * ring_length: bounds on the ranges of ring road lengths the autonomous
      vehicle is trained on

    States
        The state consists of the speed and headway of the ego vehicle, as well
        as the difference in speed between the ego vehicle and its leader.
        There is no assumption on the number of vehicles in the network.

    Actions
        See parent class

    Rewards
        See parent class

    Termination
        See parent class

    """

    @property
    def observation_space(self):
        """See class definition."""
        return Box(low=-float('inf'), high=float('inf'),
                   shape=(12, ), dtype=np.float32)
  
    @property
    def action_space(self):
        max_dec = abs(self.env_params.additional_params['max_decel'])
        max_acc = self.env_params.additional_params['max_accel']
        low  = np.array([-max_dec, -max_dec], dtype=np.float32)
        high = np.array([ max_acc,  max_acc], dtype=np.float32)
        return Box(low=low, high=high, dtype=np.float32)

    def get_state(self):
        """Centralized observation for two AVs on two lanes (own-lane leader/follower only, with relative speed)."""
        rl_ids = self.k.vehicle.get_rl_ids()
        if len(rl_ids) < 2:
            return np.zeros(12)  # 2×5 + 1(dx) + 1(dv)

        ring_len = self.k.network.length()
        max_speed = 30.0
        max_length = self.env_params.additional_params.get('ring_length', 250)

        obs = []
        self.observe = {}

        speeds = {}
        positions = {}

        # === 遍历每辆 RL 车 ===
        for rid in rl_ids:
            v_rl = self.k.vehicle.get_speed(rid)
            x_rl = self.k.vehicle.get_x_by_id(rid)
            speeds[rid] = v_rl
            positions[rid] = x_rl

            lane_idx = self.k.vehicle.get_lane(rid)
            leaders   = self.k.vehicle.get_lane_leaders(rid)   or []
            followers = self.k.vehicle.get_lane_followers(rid) or []
            headways  = self.k.vehicle.get_lane_headways(rid)  or []
            tailways  = self.k.vehicle.get_lane_tailways(rid)  or []

            lead_id, foll_id = None, None
            d_lead, d_foll = ring_len, ring_len

            if lane_idx < len(leaders):
                lead_id = leaders[lane_idx]
            if lane_idx < len(followers):
                foll_id = followers[lane_idx]
            if lane_idx < len(headways) and headways[lane_idx] is not None:
                d_lead = headways[lane_idx]
            if lane_idx < len(tailways) and tailways[lane_idx] is not None:
                d_foll = tailways[lane_idx]

            v_lead = self.k.vehicle.get_speed(lead_id) if lead_id else v_rl
            v_foll = self.k.vehicle.get_speed(foll_id) if foll_id else v_rl

            # === 拼接该 AV 的观测 ===
            obs.extend([
                v_rl / max_speed,
                v_lead / max_speed,
                v_foll / max_speed,
                d_lead / max_length,
                d_foll / max_length
            ])

            self.observe[rid] = np.array([lead_id, foll_id], dtype=object)

        # === 两辆 AV 的相对距离（有符号） ===
        x0, x1 = positions[rl_ids[0]], positions[rl_ids[1]]
        dx = (x1 - x0) % ring_len
        if dx > ring_len / 2:
            dx -= ring_len
        dx_norm = dx / max_length
        obs.append(dx_norm)

        # === 新增项：速度差 ===
        v0, v1 = speeds[rl_ids[0]], speeds[rl_ids[1]]
        dv = v1 - v0  # 有符号速度差（v2 - v1）
        dv_norm = dv / max_speed
        obs.append(dv_norm)

        return np.array(obs)




    def _apply_rl_actions(self, rl_actions):
        """
        Apply acceleration actions for two RL-controlled vehicles (centralized control).
        rl_actions: [a1, a2] or np.array([a1, a2])
        """
        rl_ids = self.k.vehicle.get_rl_ids()

        # === 安全检查 ===
        if len(rl_ids) == 0:
            return
        elif len(rl_ids) == 1:
            # 若只有一个RL车，则只应用第一个动作
            self.k.vehicle.apply_acceleration(rl_ids, [rl_actions[0]])
            return

        # === 两个RL车独立控制 ===
        rl_id_1, rl_id_2 = rl_ids[:2]
        a1 = float(rl_actions[0])
        a2 = float(rl_actions[1])

        # 可以一次性调用 apply_acceleration 或分别调用
        self.k.vehicle.apply_acceleration([rl_id_1, rl_id_2], [a1, a2])

    def compute_reward(self, rl_actions, **kwargs):
        """
        Improved reward for two cooperative AVs on a ring:
        - Encourage side-by-side (|dx| small)
        - Encourage equal speeds (|v1 - v2| small)
        - Encourage a desired cruising speed (avoid stopping)
        - Encourage smooth actions
        - Penalize very low speeds
        """

        rl_ids = self.k.vehicle.get_rl_ids()
        if len(rl_ids) < 2:
            return 0.0

        if rl_actions is None:
            rl_actions = [0.0, 0.0]

        id1, id2 = rl_ids[:2]
        ring_len = self.k.network.length()
        v_max = 15.0

        # ----------------------------
        # parameters
        # ----------------------------
        desired_speed = 10.0          # 🚀 保持高速运行的目标速度
        w_pos = 2.0                   # 并排行为
        w_sync = 1.5                  # 同速
        w_cruise = 3.0                # 巡航速度奖励（非常强）
        w_smooth = 0.05               # 平滑性（轻）
        w_stop_penalty = 5.0          # 停车惩罚

        # ----------------------------
        # vehicle states
        # ----------------------------
        v1 = self.k.vehicle.get_speed(id1)
        v2 = self.k.vehicle.get_speed(id2)
        x1 = self.k.vehicle.get_x_by_id(id1)
        x2 = self.k.vehicle.get_x_by_id(id2)

        a1 = float(rl_actions[0])
        a2 = float(rl_actions[1])

        # signed relative distance
        dx = (x2 - x1) % ring_len
        if dx > ring_len / 2:
            dx -= ring_len
        dx_norm = abs(dx) / (ring_len / 2)

        # ----------------------------
        # reward components
        # ----------------------------

        # 1) 并排：靠近 → 奖励高
        r_pos = - w_pos * dx_norm

        # 2) 同速：绝对速度差越小越好
        dv = abs(v1 - v2) / v_max
        r_sync = - w_sync * dv

        # 3) 巡航速度：速度越接近 desired_speed 越好
        v_mean = (v1 + v2) / 2
        r_cruise = - w_cruise * abs(v_mean - desired_speed) / desired_speed

        # 4) 平滑动作：少加速
        r_smooth = - w_smooth * (a1 ** 2 + a2 ** 2)

        # 5) 强力“禁停”惩罚：防止刹停 & 低速薅奖励
        if v_mean < 2.0:  # 0~2 m/s 视为低速/停车
            r_stop = -w_stop_penalty * (2.0 - v_mean)
        else:
            r_stop = 0.0

        # ----------------------------
        # final reward
        # ----------------------------
        r_total = (
            r_pos +
            r_sync +
            r_cruise +
            r_smooth +
            r_stop
        )

        return float(r_total)

    # def compute_reward(self, rl_actions, **kwargs):
    #     """
    #     Centralized reward for two AVs:
    #     - Encourage side-by-side (|dx| small)
    #     - Encourage same speed (|v1 - v2| small)
    #     - Encourage higher mean speed (v_mean large)
    #     - Penalize excessive acceleration
    #     """
    #     rl_ids = self.k.vehicle.get_rl_ids()
    #     if len(rl_ids) < 2:
    #         return 0.0

    #     if rl_actions is None:
    #         rl_actions = [0.0, 0.0]

    #     ring_len = self.k.network.length()
    #     v_max = 15.0

    #     # === parameters ===
    #     w_pos = 3.0     # 并排位置权重
    #     w_sync = 1.5    # 同速权重
    #     w_speed = 2  # 平均速度奖励
    #     w_smooth = 0.1  # 平滑惩罚

    #     id1, id2 = rl_ids[:2]
    #     v1 = self.k.vehicle.get_speed(id1)
    #     v2 = self.k.vehicle.get_speed(id2)
    #     x1 = self.k.vehicle.get_x_by_id(id1)
    #     x2 = self.k.vehicle.get_x_by_id(id2)
    #     a1 = float(rl_actions[0])
    #     a2 = float(rl_actions[1])

    #     # === 有符号相对距离 ===
    #     dx = (x2 - x1) % ring_len
    #     if dx > ring_len / 2:
    #         dx -= ring_len
    #     dx_norm = abs(dx) / (ring_len / 2)

    #     # === 各项奖励 ===
    #     # 1️⃣ 并排靠近 (越靠近奖励越高)
    #     r_pos = -w_pos * dx_norm

    #     # 2️⃣ 速度同步 (速度差越小奖励越高)
    #     dv = abs(v1 - v2) / v_max
    #     r_sync = -w_sync * dv

    #     # 3️⃣ 平均速度 (逐步提高)
    #     v_mean = (v1 + v2) / 2
    #     r_speed = w_speed * ((v_mean / v_max) ** 2) 

    #     # 4️⃣ 平滑性 (避免急加速)
    #     r_smooth = -w_smooth * (a1 ** 2 + a2 ** 2)

    #     # === 总奖励 ===
    #     r_total = r_pos + r_sync + r_speed + r_smooth
    #     return float(r_total)




    def additional_command(self):
        """Color or mark observed vehicles in SUMO GUI."""
        if not hasattr(self, 'observe'):
            return

        for rl_id in self.k.vehicle.get_rl_ids():
            if rl_id not in self.observe:
                continue
            for veh_id in self.observe[rl_id]:
                self.k.vehicle.set_observed(veh_id)

