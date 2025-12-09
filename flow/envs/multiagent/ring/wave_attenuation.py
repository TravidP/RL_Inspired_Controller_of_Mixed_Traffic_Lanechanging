"""
Environment used to train a stop-and-go dissipating controller.

This is the environment that was used in:

C. Wu, A. Kreidieh, K. Parvate, E. Vinitsky, A. Bayen, "Flow: Architecture and
Benchmarking for Reinforcement Learning in Traffic Control," CoRR, vol.
abs/1710.05465, 2017. [Online]. Available: https://arxiv.org/abs/1710.05465
"""
import time, random, numpy as np, os
import traci
import numpy as np
from gym.spaces.box import Box
import random
from scipy.optimize import fsolve
from copy import deepcopy
from flow.core.params import VehicleParams, SumoCarFollowingParams, SumoLaneChangeParams
from flow.core.params import InitialConfig
from flow.controllers import PreTrainedRLController, PreTrainedRLControllerLC, IDMController, RLController, ContinuousRouter, SimLaneChangeController
from flow.core.params import NetParams
from flow.envs.multiagent.base import MultiEnv
from flow.envs.ring.wave_attenuation import v_eq_max_function
from flow.networks.ring import RingNetwork

ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration of autonomous vehicles
    'max_accel': 0.5,
    # maximum deceleration of autonomous vehicles
    'max_decel': 0.5,
    # bounds on the ranges of ring road lengths the autonomous vehicle is
    # trained on
    'ring_length': 250,
    'lane_change_duration': 1,
    'Total_Number_Veh': 48,
    'NUM_AUTOMATED':1,
    'DriverModel':10
}


class MultiWaveAttenuationPOEnv(MultiEnv):
    """Multiagent shared model version of WaveAttenuationPOEnv.

    Intended to work with Lord Of The Rings Network.
    Note that this environment current
    only works when there is one autonomous vehicle
    on each ring.

    Required from env_params: See parent class

    States
        See parent class

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
        return Box(low=-5, high=5, shape=(11,), dtype=np.float32)

    @property
    def action_space(self):
        """See class definition."""
        add_params = self.net_params.additional_params
        num_rings = add_params['num_rings']
        return Box(
            low=-np.abs(self.env_params.additional_params['max_decel']),
            high=self.env_params.additional_params['max_accel'],
            shape=(int(self.initial_vehicles.num_rl_vehicles / num_rings), ),
            dtype=np.float32)

    def get_state(self):
        """See class definition."""
        obs = {}
        for rl_id in self.k.vehicle.get_rl_ids():
            lead_id = self.k.vehicle.get_leader(rl_id) or rl_id

            # normalizers
            max_speed = 15.
            max_length = self.env_params.additional_params['ring_length'][1]

            observation = np.array([
                self.k.vehicle.get_speed(rl_id) / max_speed,
                (self.k.vehicle.get_speed(lead_id) -
                 self.k.vehicle.get_speed(rl_id))
                / max_speed,
                self.k.vehicle.get_headway(rl_id) / max_length
            ])
            obs.update({rl_id: observation})

        return obs

    def _apply_rl_actions(self, rl_actions):
        """Split the accelerations by ring."""
        if rl_actions:
            rl_ids = list(rl_actions.keys())
            accel = list(rl_actions.values())
            self.k.vehicle.apply_acceleration(rl_ids, accel)

    def compute_reward(self, rl_actions, **kwargs):
        """See class definition."""
        # in the warmup steps
        if rl_actions is None:
            return {}

        rew = {}
        for rl_id in rl_actions.keys():
            edge_id = rl_id.split('_')[1]
            edges = self.gen_edges(edge_id)
            vehs_on_edge = self.k.vehicle.get_ids_by_edge(edges)
            vel = np.array([
                self.k.vehicle.get_speed(veh_id)
                for veh_id in vehs_on_edge
            ])
            if any(vel < -100) or kwargs['fail']:
                return 0.

            target_vel = self.env_params.additional_params['target_velocity']
            max_cost = np.array([target_vel] * len(vehs_on_edge))
            max_cost = np.linalg.norm(max_cost)

            cost = vel - target_vel
            cost = np.linalg.norm(cost)

            rew[rl_id] = max(max_cost - cost, 0) / max_cost
        return rew

    def additional_command(self):
        """Define which vehicles are observed for visualization purposes."""
        # specify observed vehicles
        for rl_id in self.k.vehicle.get_rl_ids():
            lead_id = self.k.vehicle.get_leader(rl_id) or rl_id
            self.k.vehicle.set_observed(lead_id)

    @staticmethod
    def gen_edges(i):
        """Return the edges corresponding to the rl id."""
        return ['top_{}'.format(i), 'left_{}'.format(i),
                'right_{}'.format(i), 'bottom_{}'.format(i)]


class MultiAgentWaveAttenuationPOEnv(MultiEnv):
    """Multi-agent variant of WaveAttenuationPOEnv.

    Required from env_params:

    * max_accel: maximum acceleration of autonomous vehicles
    * max_decel: maximum deceleration of autonomous vehicles
    * ring_length: bounds on the ranges of ring road lengths the autonomous
      vehicle is trained on. If set to None, the environment sticks to the ring
      road specified in the original network definition.

    States
        The state of each agent (AV) consists of the speed and headway of the
        ego vehicle, as well as the difference in speed between the ego vehicle
        and its leader. There is no assumption on the number of vehicles in the
        network.

    Actions
        Actions are an acceleration for each rl vehicle, bounded by the maximum
        accelerations and decelerations specified in EnvParams.

    Rewards
        The reward function rewards high average speeds from all vehicles in
        the network, and penalizes accelerations by the rl vehicle. This reward
        is shared by all agents.

    Termination
        A rollout is terminated if the time horizon is reached or if two
        vehicles collide into one another.
    """

    def __init__(self, env_params, sim_params, network, simulator='traci'):
        self.last_speed = {}
        for p in ADDITIONAL_ENV_PARAMS.keys():
            if p not in env_params.additional_params:
                raise KeyError(
                    'Environment parameter \'{}\' not supplied'.format(p))
        self.total_step_counter = 0
        super().__init__(env_params, sim_params, network, simulator)
  
    
    # @property


    # def observation_space(self):
    #     """Define 7D observation space for simplified features."""
    #     # 速度和距离都归一化到 [0,1]，队友距离 [-0.5, 0.5]
    #     low  = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0], dtype=np.float32)
    #     high = np.array([1.0, 1.0, 1.0, 1.0, 1.0,  0.5, 1.0], dtype=np.float32)
    #     return Box(low=low, high=high, dtype=np.float32)



    # @property
    # def action_space(self):
    #     max_dec = np.abs(self.env_params.additional_params['max_decel'])
    #     max_acc = self.env_params.additional_params['max_accel']

    #     low = np.array([-max_dec, -1.0], dtype=np.float32)
    #     high = np.array([ max_acc,  1.0], dtype=np.float32)

    #     # 移除 shape 参数，Gym 会根据 low/high 的 shape 自动设定
    #     return Box(low=low, high=high, dtype=np.float32)


    # def action_space(self):
    #     """Only control acceleration; disable lane change."""
    #     max_dec = np.abs(self.env_params.additional_params['max_decel'])
    #     max_acc = self.env_params.additional_params['max_accel']

    #     # 只控制加速度
    #     low = np.array([-max_dec], dtype=np.float32)
    #     high = np.array([max_acc], dtype=np.float32)
    #     return Box(low=low, high=high, dtype=np.float32)
    

    @property
    def action_space(self):
        """See class definition."""
        max_decel = self.env_params.additional_params["max_decel"]
        max_accel = self.env_params.additional_params["max_accel"]

        lb = [-abs(max_decel), -1] 
        ub = [max_accel, 1] 

        return Box(np.array(lb), np.array(ub), dtype=np.float32)
    @property
    def observation_space(self):
        """See class definition."""
        return Box(low=-float('inf'), high=float('inf'),
                   shape=(11, ), dtype=np.float32)

    
    
    # def get_state(self):
    #     """See class definition."""
    #     obs = {}
    #     self.observe = {}

    #     n_lanes   = self.net_params.additional_params['lanes']
    #     max_speed = self.k.network.max_speed()
    #     ring_len  = self.k.network.length()

    #     for rl_id in self.k.vehicle.get_rl_ids():
    #         # 0. 自身速度 & 1. 车道号
    #         v    = self.k.vehicle.get_speed(rl_id)
    #         lane = int(self.k.vehicle.get_lane(rl_id))
    #         leaders    = self.k.vehicle.get_lane_leaders(rl_id)
    #         followers  = self.k.vehicle.get_lane_followers(rl_id)
    #         headways   = self.k.vehicle.get_lane_headways(rl_id)
    #         tailways   = self.k.vehicle.get_lane_tailways(rl_id)
    #         features = [v/max_speed]
    #         for l in range(n_lanes):
    #             is_rl_lane = (l == lane)

    #             if is_rl_lane:
    #                 lead_id = leaders[lane] or rl_id
                    
    #                 dx_f = headways[lane] if lane < len(headways) else 0.0
    #                 dv_f = self.k.vehicle.get_speed(lead_id)
                    
    #                 follow_id = followers[lane] or rl_id
    #                 dx_b = tailways[lane] if lane < len(tailways) else 0.0
    #                 dv_b = self.k.vehicle.get_speed(follow_id)

    #             else:
    #                 other_lane = (lane + 1) % n_lanes

    #                 # 前车
    #                 ahead_id   = leaders[other_lane] or rl_id
    #                 dv_f      = self.k.vehicle.get_speed(ahead_id) 
    #                 dx_f      = headways[other_lane] if other_lane < len(headways) else 0.0

    #                 # 后车
    #                 back_id    = followers[other_lane] or rl_id
    #                 dv_b      = self.k.vehicle.get_speed(back_id)
    #                 dx_b      = tailways[other_lane] if other_lane < len(tailways) else 0.0
    #             features.extend([
    #                 is_rl_lane,
    #                 dv_f / max_speed,       # 2: 1车道前车速度差
    #                 dx_f / ring_len,        # 3: 1车道前车距离
    #                 dv_b / max_speed,       # 4: 1车道后车速度差
    #                 dx_b / ring_len,        # 5: 1车道后车距离
    #                     ])
    #     # 记录可观测车辆 ID
    #         features = np.array(features, dtype=np.float32)
    #         self.observe[rl_id] = np.array([lead_id, follow_id, ahead_id, back_id])
    #         obs[rl_id] = features

    #     return obs

    # def get_state(self):
    #     """Return state observation for each RL vehicle."""
    #     obs = {}
    #     self.observe = {}

    #     n_lanes   = self.net_params.additional_params['lanes']
    #     max_speed = self.k.network.max_speed()
    #     ring_len  = self.k.network.length()

    #     rl_ids = self.k.vehicle.get_rl_ids()
    #     rl_positions = {rid: self.k.vehicle.get_x_by_id(rid) for rid in rl_ids}
    #     rl_lanes = {rid: int(self.k.vehicle.get_lane(rid)) for rid in rl_ids}

    #     for rl_id in rl_ids:
    #         v = self.k.vehicle.get_speed(rl_id)
    #         lane = rl_lanes[rl_id]
    #         leaders    = self.k.vehicle.get_lane_leaders(rl_id)
    #         followers  = self.k.vehicle.get_lane_followers(rl_id)
    #         headways   = self.k.vehicle.get_lane_headways(rl_id)
    #         tailways   = self.k.vehicle.get_lane_tailways(rl_id)

    #         features = [v / max_speed]

    #         # === 最近 RL 车辆 ===
    #         my_pos = rl_positions[rl_id]
    #         min_dist = float('inf')
    #         nearest_rl = None
    #         rel_sign = 0  # + 前方, - 后方

    #         for other_id in rl_ids:
    #             if other_id == rl_id:
    #                 continue
    #             other_pos = rl_positions[other_id]
    #             forward_dist = (other_pos - my_pos) % ring_len
    #             backward_dist = -((my_pos - other_pos) % ring_len)
    #             if forward_dist < min_dist and forward_dist < ring_len / 2:
    #                 min_dist = forward_dist
    #                 nearest_rl = other_id
    #                 rel_sign = +1
    #             elif abs(backward_dist) < min_dist:
    #                 min_dist = abs(backward_dist)
    #                 nearest_rl = other_id
    #                 rel_sign = -1

    #         if nearest_rl is not None:
    #             nearest_dist_norm = rel_sign * (min_dist / ring_len)
    #             same_lane = 1.0 if rl_lanes[nearest_rl] == lane else 0.0
    #         else:
    #             nearest_dist_norm = 0.0
    #             same_lane = 0.0

    #         # === 两个车道（当前+隔壁）的前后车辆特征 ===
    #         all_refs = []  # 用于记录ID
    #         for l in range(n_lanes):
    #             is_rl_lane = (l == lane)
    #             if is_rl_lane:
    #                 lead_id = leaders[lane] or rl_id
    #                 follow_id = followers[lane] or rl_id
    #                 dx_f = headways[lane] if lane < len(headways) else 0.0
    #                 dv_f = self.k.vehicle.get_speed(lead_id)
    #                 dx_b = tailways[lane] if lane < len(tailways) else 0.0
    #                 dv_b = self.k.vehicle.get_speed(follow_id)
    #                 all_refs.extend([lead_id, follow_id])
    #             else:
    #                 other_lane = (lane + 1) % n_lanes
    #                 ahead_id = leaders[other_lane] or rl_id
    #                 back_id  = followers[other_lane] or rl_id
    #                 dx_f = headways[other_lane] if other_lane < len(headways) else 0.0
    #                 dv_f = self.k.vehicle.get_speed(ahead_id)
    #                 dx_b = tailways[other_lane] if other_lane < len(tailways) else 0.0
    #                 dv_b = self.k.vehicle.get_speed(back_id)
    #                 all_refs.extend([ahead_id, back_id])

    #             features.extend([
    #                 is_rl_lane,
    #                 dv_f / max_speed,
    #                 dx_f / ring_len,
    #                 dv_b / max_speed,
    #                 dx_b / ring_len,
    #             ])

    #         # 加上最近 RL 的观测
    #         features.extend([nearest_dist_norm, same_lane])

    #         # 保存所有观测到的相关车辆ID
    #         self.observe[rl_id] = np.array(all_refs + [nearest_rl if nearest_rl else 'None'])
    #         obs[rl_id] = np.array(features, dtype=np.float32)
    #     # print(obs)
    #     return obs

    # def get_state(self):
    #     """Return enhanced state observation for each RL vehicle."""
    #     obs = {}
    #     self.observe = {}

    #     n_lanes   = self.net_params.additional_params['lanes']
    #     max_speed = self.k.network.max_speed()
    #     ring_len  = self.k.network.length()

    #     rl_ids = self.k.vehicle.get_rl_ids()
    #     rl_positions = {rid: self.k.vehicle.get_x_by_id(rid) for rid in rl_ids}
    #     rl_speeds    = {rid: self.k.vehicle.get_speed(rid) for rid in rl_ids}
    #     rl_lanes     = {rid: int(self.k.vehicle.get_lane(rid)) for rid in rl_ids}

    #     for rl_id in rl_ids:
    #         v = rl_speeds[rl_id]
    #         lane = rl_lanes[rl_id]

    #         # === 当前车道 one-hot ===
    #         lane_onehot = [0.0] * n_lanes
    #         lane_onehot[lane] = 1.0

    #         leaders    = self.k.vehicle.get_lane_leaders(rl_id)
    #         followers  = self.k.vehicle.get_lane_followers(rl_id)
    #         headways   = self.k.vehicle.get_lane_headways(rl_id)
    #         tailways   = self.k.vehicle.get_lane_tailways(rl_id)

    #         # === 基础特征 ===
    #         features = [v / max_speed] + lane_onehot

    #         # === 最近 RL 车辆 ===
    #         my_pos = rl_positions[rl_id]
    #         min_dist = float('inf')
    #         nearest_rl = None

    #         for other_id in rl_ids:
    #             if other_id == rl_id:
    #                 continue
    #             other_pos = rl_positions[other_id]
    #             # 环上有符号距离：前方为正，后方为负
    #             dx_signed = (other_pos - my_pos) % ring_len
    #             if dx_signed > ring_len / 2:
    #                 dx_signed -= ring_len  # 负方向
    #             if abs(dx_signed) < abs(min_dist):
    #                 min_dist = dx_signed
    #                 nearest_rl = other_id

    #         # === 队友关系特征 ===
    #         if nearest_rl is not None:
    #             nearest_dist_norm = min_dist / ring_len  # [-0.5, 0.5]
    #             same_lane = 1.0 if rl_lanes[nearest_rl] == lane else 0.0
    #             dv_teammate = (rl_speeds[nearest_rl] - v) / max_speed
    #         else:
    #             nearest_dist_norm = 0.0
    #             same_lane = 0.0
    #             dv_teammate = 0.0

    #         # === 两个车道（当前+隔壁）的前后车辆特征 ===
    #         all_refs = []
    #         for l in range(n_lanes):
    #             is_cur_lane = 1.0 if l == lane else 0.0
    #             lead_id = leaders[l] if l < len(leaders) and leaders[l] else rl_id
    #             follow_id = followers[l] if l < len(followers) and followers[l] else rl_id

    #             dx_f = headways[l] if l < len(headways) else 0.0
    #             dx_b = tailways[l] if l < len(tailways) else 0.0
    #             dv_f = self.k.vehicle.get_speed(lead_id)
    #             dv_b = self.k.vehicle.get_speed(follow_id)

    #             features.extend([
    #                 is_cur_lane,
    #                 dv_f / max_speed,
    #                 dx_f / ring_len,
    #                 dv_b / max_speed,
    #                 dx_b / ring_len,
    #             ])
    #             all_refs.extend([lead_id, follow_id])

    #         # === 加入队友特征 ===
    #         features.extend([
    #             nearest_dist_norm,  # 环形相对距离（正负）
    #             same_lane,          # 是否同车道
    #             dv_teammate,        # 队友速度差
    #         ])

    #         # === 记录 ===
    #         self.observe[rl_id] = np.array(all_refs + [nearest_rl if nearest_rl else 'None'])
    #         obs[rl_id] = np.array(features, dtype=np.float32)

    #     return obs

    def get_state(self):
        """
        Multi-agent observation.
        
        For each RL vehicle vid:
            obs = [
                rl_speed,
                for each lane:
                    is_rl_lane,
                    dist_to_front,
                    front_speed,
                    dist_to_back,
                    back_speed
            ]
        """

        rl_ids = sorted(self.k.vehicle.get_rl_ids())
        # print("RL IDS:", self.k.vehicle.get_rl_ids())

        assert len(rl_ids) >= 1, "Environment must contain at least one AV."

        # ----- constants -----
        n_lanes   = self.net_params.additional_params['lanes']
        max_speed = 15.0
        ring_len  = self.k.network.length()

        obs_dict = {}
        self.observe = {}
        for rl_id in rl_ids:

            # ======================
            # 1) 自身速度与车道
            # ======================
            obs = []

            v_rl = self.k.vehicle.get_speed(rl_id)
            lane_rl = int(self.k.vehicle.get_lane(rl_id))
            obs.append(v_rl / max_speed)

            # ======================
            # 2) 该 AV 的 lane info
            # ======================
            leaders   = self.k.vehicle.get_lane_leaders(rl_id)
            followers = self.k.vehicle.get_lane_followers(rl_id)
            headways  = self.k.vehicle.get_lane_headways(rl_id)
            tailways  = self.k.vehicle.get_lane_tailways(rl_id)

            observe_ids = []

            for lane in range(n_lanes):

                # 是否是 AV 所在的车道
                is_rl_lane = (lane == lane_rl)
                obs.append(1.0 if is_rl_lane else 0.0)

                # ---- front ----
                lead_id = leaders[lane] or rl_id
                v_f  = self.k.vehicle.get_speed(lead_id)
                dx_f = headways[lane] if lane < len(headways) else 0.0

                # ---- back ----
                back_id = followers[lane] or rl_id
                v_b  = self.k.vehicle.get_speed(back_id)
                dx_b = tailways[lane] if lane < len(tailways) else 0.0

                obs.extend([
                    dx_f / ring_len,
                    v_f / max_speed,
                    dx_b / ring_len,
                    v_b / max_speed,
                ])

                observe_ids.extend([lead_id, back_id])

            # 保存该 AV 观测到的原始 ID（用于 debug）
            self.observe[rl_id] = np.array(observe_ids, dtype=object)

            # 加入 multi-agent 结果
            obs_dict[rl_id] = np.array(obs, dtype=np.float32)
        # print(self.observe)
        # print(obs_dict)
        return obs_dict

    # def get_state(self):
    #     """Return 7D observation per RL vehicle:
    #     [self_v, lead_v, headway, back_v, back_gap, other_rl_signed_dist, other_rl_v]"""
    #     obs = {}
    #     self.observe = {}

    #     max_speed = self.k.network.max_speed()
    #     ring_len  = self.k.network.length()
    #     rl_ids = self.k.vehicle.get_rl_ids()
    #     if len(rl_ids) == 0:
    #         return obs

    #     # 缓存RL位置信息与速度
    #     rl_pos = {rid: self.k.vehicle.get_x_by_id(rid) for rid in rl_ids}
    #     rl_spd = {rid: self.k.vehicle.get_speed(rid) for rid in rl_ids}

    #     for rid in rl_ids:
    #         v_self = rl_spd[rid]
    #         lane   = int(self.k.vehicle.get_lane(rid))

    #         # ===== 当前车道前车 =====
    #         leaders   = self.k.vehicle.get_lane_leaders(rid)
    #         followers = self.k.vehicle.get_lane_followers(rid)
    #         headways  = self.k.vehicle.get_lane_headways(rid)
    #         tailways  = self.k.vehicle.get_lane_tailways(rid)

    #         lead_id = leaders[lane] if (leaders and lane < len(leaders) and leaders[lane]) else None
    #         if lead_id is None:
    #             lead_v = v_self
    #             headway = 1.0
    #         else:
    #             lead_v = self.k.vehicle.get_speed(lead_id)
    #             headway = (headways[lane] / ring_len) if (headways and lane < len(headways)) else 0.0

    #         # ===== 当前车道后车 =====
    #         back_id = followers[lane] if (followers and lane < len(followers) and followers[lane]) else None
    #         if back_id is None:
    #             back_v = v_self
    #             back_gap = 1.0
    #         else:
    #             back_v = self.k.vehicle.get_speed(back_id)
    #             back_gap = (tailways[lane] / ring_len) if (tailways and lane < len(tailways)) else 0.0

    #         # ===== 另一辆 RL 的有符号距离 =====
    #         other_rl_id = None
    #         signed_dist_norm = 0.0
    #         other_rl_v = 0.0
    #         if len(rl_ids) >= 2:
    #             my_pos = rl_pos[rid]
    #             other_candidates = [x for x in rl_ids if x != rid]
    #             if other_candidates:
    #                 dmin = float("inf")
    #                 chosen = None
    #                 dx_signed_best = 0.0
    #                 for oid in other_candidates:
    #                     raw = (rl_pos[oid] - my_pos) % ring_len
    #                     # 有符号化：前方为正，后方为负
    #                     dx_signed = raw if raw <= ring_len / 2 else raw - ring_len
    #                     if abs(dx_signed) < abs(dmin):
    #                         dmin = abs(dx_signed)
    #                         dx_signed_best = dx_signed
    #                         chosen = oid
    #                 other_rl_id = chosen
    #                 signed_dist_norm = dx_signed_best / ring_len  # 归一化到 [-0.5, 0.5]
    #                 other_rl_v = rl_spd[other_rl_id]

    #         # ===== 构造观测 =====
    #         features = [
    #             v_self / max_speed,         # 自身速度 (0~1)
    #             lead_v / max_speed,         # 前车速度 (0~1)
    #             headway,                    # 前车距 / ring_len
    #             back_v / max_speed,         # 后车速度 (0~1)
    #             back_gap,                   # 后车距 / ring_len
    #             signed_dist_norm,           # 队友相对距离 (-0.5~0.5)
    #             other_rl_v / max_speed,     # 队友速度 (0~1)
    #         ]

    #         self.observe[rid] = np.array([lead_id, back_id, other_rl_id], dtype=object)
    #         obs[rid] = np.array(features, dtype=np.float32)
    #         # print(obs)
    #     return obs



    # def _apply_rl_actions(self, rl_actions):
    #     """Split the accelerations by ring."""
    #     if rl_actions:
    #         rl_ids = list(rl_actions.keys())
    #         accel = list(rl_actions.values())
    #         self.k.vehicle.apply_acceleration(rl_ids, accel)


    # def _apply_rl_actions(self, rl_actions):
    #     """Split the actions into acceleration and lane-change and apply both."""
    #     if not rl_actions:
    #         return

    #     rl_ids = list(rl_actions.keys())
    #     accels = []
    #     lane_changes = []

    #     for act in rl_actions.values():
    #         # act is a length-2 array: [acceleration, continuous_lane_change]
    #         accel, lc_cont = act

    #         # 1) 加速度
    #         accel = (accel * 2 - 1) * (self.env_params.additional_params["max_accel"] if accel > 0.5 else self.env_params.additional_params["max_decel"])
    #         accels.append(accel)

    #         # 2) 连续变道信号 → 三档整数 -1, 0, +1
    #         lc_cmd = int(np.round(lc_cont))
    #         lane_changes.append(lc_cmd)
    #     cooldown = self.env_params.additional_params['lane_change_duration']
    #     current_time = self.k.kernel_api.simulation.getTime()
    #     old_lanes = {vid: self.k.vehicle.get_lane(vid) for vid in rl_ids}
    #     final_lcs = []
    #     for vid, cmd in zip(rl_ids, lane_changes):
    #         last_time = self._last_req_time.get(vid, -1e9)
    #         blocked   = (current_time <= last_time + cooldown)
    #         # 如果在冷却期或者根本没要变道，就置零
    #         if blocked or cmd == 0:
    #             final_lcs.append(0)
    #         else:
    #             final_lcs.append(cmd)


    #         #print(f"[DEBUG] vid={vid}, now={current_time:.1f}s, "
    #          #     f"last_req={last_time:.1f}s, blocked={blocked}")
    #             # 再屏蔽冷却期中的指令
                      
        
    #     self.k.vehicle.apply_acceleration(rl_ids, accels)
                
    #     # 再执行变道指令（-1 左变道，0 不变道，+1 右变道）
    #     #print(final_lcs)
    #     self.k.vehicle.apply_lane_change(rl_ids, final_lcs)
    #     for vid, cmd in zip(rl_ids, final_lcs):
    #         if cmd != 0:
    #             new_lane = self.k.vehicle.get_lane(vid)
    #             old_lane = old_lanes[vid]
    #             if new_lane != old_lane:
    #                 self._last_req_time[vid] = current_time
    #     # for vid in self.k.vehicle.get_ids():
    #     #     mode = self.k.kernel_api.vehicle.getLaneChangeMode(vid)
    #     #     print(f"[CHECK] veh={vid} lane_change_mode={mode}")

    # def _apply_rl_actions(self, actions):
    #     # 安全地获取所有 rl 车辆，按名称排序
    #     sorted_rl_ids = sorted(self.k.vehicle.get_rl_ids())

    #     acceleration = []
    #     raw_dir = []

    #     for vid in sorted_rl_ids:
    #         if vid not in actions:
    #             print(f"[WARNING] No action for {vid}, skipping")
    #             continue
    #         a, d = actions[vid]
    #         acceleration.append(a)
    #         raw_dir.append(d)

    #     acceleration = np.array(acceleration)
    #     raw_dir = np.array(raw_dir)

    #     # 离散化 lane change 动作
    #     dir_discrete = np.zeros_like(raw_dir, dtype=int)
    #     dir_discrete[raw_dir > 1/3] = 1
    #     dir_discrete[raw_dir < -1/3] = -1

    #     # 变道冷却机制
    #     non_lane_changing = [
    #         self.time_counter <=
    #         self.env_params.additional_params["lane_change_duration"]
    #         + self.k.vehicle.get_last_lc(vid)
    #         for vid in sorted_rl_ids
    #     ]
    #     dir_discrete = np.where(non_lane_changing, 0, dir_discrete)

    #     # 应用动作
    #     self.k.vehicle.apply_acceleration(sorted_rl_ids, acc=acceleration)
    #     self.k.vehicle.apply_lane_change(sorted_rl_ids, direction=dir_discrete)

    # def _apply_rl_actions(self, actions):
    #     """Apply RL actions (only acceleration, no lane change)."""
    #     sorted_rl_ids = sorted(self.k.vehicle.get_rl_ids())

    #     accelerations = []
    #     for vid in sorted_rl_ids:
    #         if vid not in actions:
    #             print(f"[WARNING] No action for {vid}, skipping")
    #             continue
    #         # 动作是单一标量
    #         a = actions[vid]
    #         # 有的算法会输出 [a] 或 ndarray，需要安全取值
    #         if isinstance(a, (list, np.ndarray)):
    #             a = float(a[0])
    #         accelerations.append(a)

    #     accelerations = np.array(accelerations, dtype=np.float32)

    #     # 仅执行加速度控制
    #     self.k.vehicle.apply_acceleration(sorted_rl_ids, acc=accelerations)

    #     # 禁止变道
    #     self.k.vehicle.apply_lane_change(sorted_rl_ids, direction=np.zeros_like(accelerations, dtype=int))
    def _apply_rl_actions(self, actions):
        """
        Multi-AV version.
        actions: dict {vid: [acceleration, raw_direction]}
        """

        sorted_rl_ids = sorted(self.k.vehicle.get_rl_ids())

        for vid in sorted_rl_ids:
            if vid not in actions:
                continue

            acc, raw_dir = actions[vid]
            acc = float(acc)
            raw_dir = float(raw_dir)

            # ---- 1. raw_dir → discrete ----
            if raw_dir > 1/3:
                dir_discrete = 1
            elif raw_dir < -1/3:
                dir_discrete = -1
            else:
                dir_discrete = 0

            # ---- 2. non-lane-change time window ----
            last_lc = self.k.vehicle.get_last_lc(vid)
            lc_duration = self.env_params.additional_params["lane_change_duration"]

            if self.time_counter <= last_lc + lc_duration:
                dir_discrete = 0

            # ---- 3. Apply control ----
            self.k.vehicle.apply_acceleration(vid, acc)
            self.k.vehicle.apply_lane_change(vid, direction=dir_discrete)


    # def compute_reward(self, rl_actions, **kwargs):
    #     """Compute lane-wise normalized reward based on mean and std of platoon speed."""
    #     if rl_actions is None:
    #         return {key: 0. for key in self.k.vehicle.get_rl_ids()}

    #     veh_ids = self.k.vehicle.get_ids()
    #     rl_ids = self.k.vehicle.get_rl_ids()

    #     vel = np.array([self.k.vehicle.get_speed(veh_id) for veh_id in veh_ids])
    #     if any(vel < -100) or kwargs.get('fail', False):
    #         return {key: 0. for key in rl_ids}

    #     # 最大速度，用于归一化（可从参数中取）
    #     v_max = 10

    #     # 每辆车所在车道
    #     lanes = [self.k.vehicle.get_lane(veh_id) for veh_id in veh_ids]
    #     lane_to_speeds = {}
    #     for veh_id, lane, v in zip(veh_ids, lanes, vel):
    #         lane_to_speeds.setdefault(lane, []).append(v)

    #     # 计算每条车道的平均速度和标准差
    #     lane_stats = {
    #         lane: {
    #             "mean": np.mean(speeds),
    #             "std": np.std(speeds)
    #         } for lane, speeds in lane_to_speeds.items()
    #     }

    #     # 权重参数
    #     w_mean = 0.3
    #     w_std = 0.7

    #     rewards = {}
    #     for rl_id in rl_ids:
    #         lane = self.k.vehicle.get_lane(rl_id)
    #         mean_v = lane_stats[lane]["mean"]
    #         std_v = lane_stats[lane]["std"]

    #         # 归一化
    #         mean_norm = mean_v / v_max
    #         std_norm = std_v / mean_v

    #         # 奖励函数：高平均速度、低波动
    #         reward = w_mean * mean_norm - w_std * std_norm
    #         rewards[rl_id] = reward

    #     return rewards
            
    # def compute_reward(self, rl_actions, **kwargs):
    #     """
    #     Dual-RL platoon stabilizing reward:
    #     - Encourage side-by-side formation (small distance)
    #     - Encourage synchronized and moderate speed
    #     - Suppress traffic waves (small speed variance among human vehicles)
    #     - Penalize large acceleration (smooth control)
    #     """
    #     rl_ids = sorted(self.k.vehicle.get_rl_ids())
    #     if len(rl_ids) < 2:
    #         return {rid: 0.0 for rid in rl_ids}

    #     ring_len = self.k.network.length()
    #     v_ref = float(self.env_params.additional_params.get("speed_ref", 6.0))
    #     v_max = self.k.network.max_speed()

    #     # === 获取 RL 车辆状态 ===
    #     x = np.array([self.k.vehicle.get_x_by_id(i) for i in rl_ids])
    #     v = np.array([self.k.vehicle.get_speed(i) for i in rl_ids])

    #     # === 基于速度差分计算加速度 ===
    #     sim_step = getattr(self, "sim_step", 0.1)  # 默认时间步 0.1s，可根据环境修改
    #     a = []
    #     for i, rid in enumerate(rl_ids):
    #         v_now = v[i]
    #         v_prev = self.last_speed.get(rid, v_now) if hasattr(self, "last_speed") else v_now
    #         a0 = (v_now - v_prev) / max(sim_step, 1e-3)
    #         a.append(a0)
    #         # 更新缓存
    #         if not hasattr(self, "last_speed"):
    #             self.last_speed = {}
    #         self.last_speed[rid] = v_now
    #     a = np.array(a)

    #     # === 环形有符号距离 ===
    #     raw = (x[1] - x[0]) % ring_len
    #     dx_signed = raw if raw <= ring_len / 2 else raw - ring_len
    #     abs_dx = abs(dx_signed)

    #     # === 车流速度方差 ===
    #     human_ids = self.k.vehicle.get_human_ids()
    #     if len(human_ids) > 0:
    #         human_speeds = [self.k.vehicle.get_speed(h) for h in human_ids]
    #         v_var = np.var(human_speeds) / (v_max ** 2)
    #     else:
    #         v_var = 0.0

    #     # === 奖励项 ===
    #     w1 = 4.0   # 并排
    #     w2 = 3.0   # 同步惩罚
    #     w3 = 1.0   # 提速
    #     w4 = 2.0   # 抑制波动
    #     w5 = 0.5   # 平滑

    #     beta = 20.0
    #     eps = 1e-3

    #     # (1) 并排靠近：对数函数
    #     R_pair = w1 * np.log(1.0 + beta / (abs_dx + eps))

    #     # (2) 速度同步 + 提速
    #     dv = abs(v[0] - v[1])
    #     mean_v = np.mean(v)

    #     R_sync = -w2 * (dv / v_ref)
    #     R_speed = w3 * (mean_v / v_ref)

    #     # (3) 抑制波动：人车速度方差越小越好
    #     R_wave = -w4 * v_var

    #     # (4) 平滑控制：加速度越小越好（基于差分计算）
    #     R_accel = -w5 * np.sum(a ** 2)

    #     # === 总奖励 ===
    #     R_total = R_pair + R_sync + R_speed + R_wave + R_accel
    #     R_total = float(R_total)

    #     return {rid: R_total for rid in rl_ids}

    def compute_reward(self, rl_actions, **kwargs):
        """
        Multi-agent global reward.
        Every RL agent gets the same reward = average velocity.
        """

        if rl_actions is None:
            return {vid: 0.0 for vid in self.k.vehicle.get_rl_ids()}

        # ---- gather all speeds ----
        vel = np.array([
            self.k.vehicle.get_speed(veh_id)
            for veh_id in self.k.vehicle.get_ids()
        ])

        # ---- fail conditions ----
        if any(vel < -100) or kwargs.get("fail", False):
            return {vid: 0.0 for vid in self.k.vehicle.get_rl_ids()}

        # ---- compute global reward ----
        eta_2 = 4.
        reward = eta_2 * np.mean(vel) / 20.0

        # ---- assign to all RL agents ----
        rl_ids = self.k.vehicle.get_rl_ids()
        reward_dict = {vid: reward for vid in rl_ids}

        return reward_dict




    # def compute_reward(self, rl_actions, **kwargs):
    #     """
    #     Asymmetric linear reward tailored to:
    #     - Front car: brake to close the gap to the rear car.
    #     - Rear car: wait, then when close, synchronize and go fast together.

    #     Notation:
    #     dx_signed > 0  => agent1 is in front of agent0
    #     dx_signed < 0  => agent0 is in front of agent1
    #     """
    #     rl_ids = sorted(self.k.vehicle.get_rl_ids())
    #     if len(rl_ids) < 2:
    #         return {rid: 0.0 for rid in rl_ids}

    #     ring_len = self.k.network.length()
    #     v_ref = float(self.env_params.additional_params.get("speed_ref", 6.0))  # 可达速度基准

    #     # 位置信息与速度
    #     x = np.array([self.k.vehicle.get_x_by_id(i) for i in rl_ids])
    #     v = np.array([self.k.vehicle.get_speed(i)  for i in rl_ids])

    #     # 有符号环形距离（以 rl0 视角：rl1在前为正）
    #     raw = (x[1] - x[0]) % ring_len
    #     dx_signed = raw if raw <= ring_len / 2 else raw - ring_len
    #     abs_dx = abs(dx_signed)

    #     # 判定谁在前、谁在后
    #     if dx_signed > 0:
    #         front_idx, rear_idx = 1, 0
    #     else:
    #         front_idx, rear_idx = 0, 1

    #     # --- 线性/分段因子 ---
    #     d_th   = float(self.env_params.additional_params.get("pair_dist_th", 2))    # 并排阈值（米）
    #     d_cap  = float(self.env_params.additional_params.get("pair_dist_cap", 50.0))  # 距离压缩上限（米）
    #     closeness = 1.0 - min(abs_dx, d_cap) / d_cap          # ∈[0,1]，越近越大
    #     near_gate = 1.0 if abs_dx <= d_th else 0.0            # 并排门控（简洁线性版）

    #     # --- 速度项与同步项（线性） ---
    #     mean_v = float(np.mean(v))
    #     dv     = abs(v[0] - v[1])

    #     # --- 权重（按你的目标已非对称化） ---
    #     # 前车：强距离、近了再给速度，加一个“远了跑太快扣分”的项来诱导刹车
    #     # 后车：距离权重≈0，只看同步与“近了后自身速度”
    #     w_close_front = float(self.env_params.additional_params.get("w_close_front", 11.0))
    #     w_speed_near  = float(self.env_params.additional_params.get("w_speed_near", 0.5))
    #     w_speed_far_punish_front = float(self.env_params.additional_params.get("w_speed_far_punish_front", 1))
    #     w_sync        = float(self.env_params.additional_params.get("w_sync", 2))
    #     w_4 = 0.6
    #     # 可选整体缩放，增强学习信号
    #     scale = float(self.env_params.additional_params.get("reward_scale", 10.0))

    #     rewards = {}

    #     # --- 前车奖励 ---
    #     i = front_idx
    #     # 1) 距离（只给前车）：越近越好
    #     term_close = w_close_front * closeness
    #     # 2) 并排后速度奖励：near_gate 生效，鼓励并排后一起更快
    #     term_speed_near = w_speed_near * near_gate * (v[i] / max(v_ref, 1e-6))
    #     # 3) 远时速度惩罚：不近时跑太快扣分 => 逼迫前车减速等待
    #     term_speed_far_pen = - w_speed_far_punish_front * (1 - near_gate) * (v[i] / max(v_ref, 1e-6))
    #     # 4) 同步项：速度差越小越好（防抖）
    #     term_sync = - w_sync* near_gate * (dv / max(v_ref, 1e-6))
    
    #     v_now = v[i]
    #     v_prev = self.last_speed.get(rl_ids[i], v_now)
    #     a0 = (v_now - v_prev) / max(self.sim_step, 1e-3)
    #     self.last_speed[rl_ids[i]] = v_now

    #     # print(f"[DEBUG] {rl_ids[i]}: v_now={v_now:.3f}, v_prev={v_prev:.3f}, a={a0:.3f}")

    #     term_accel = - w_4  * (a0 ** 2)


    #     R_front = scale * (term_close + term_speed_near + term_speed_far_pen + term_sync + term_accel)
    #     rewards[rl_ids[i]] = float(R_front)

    #     # --- 后车奖励 ---
    #     i = rear_idx
    #     # 后车几乎不看距离：只在并排后给速度奖励 + 同步
        
    #     term_speed_near = w_speed_near * (v[i] / max(v_ref, 1e-6))
    #     term_sync = - w_sync * near_gate * (dv / max(v_ref, 1e-6))
    #     v_now = v[i]
    #     v_prev = self.last_speed.get(rl_ids[i], v_now)
    #     a0 = (v_now - v_prev) / max(self.sim_step, 1e-3)
    #     self.last_speed[rl_ids[i]] = v_now

    #     # print(f"[DEBUG] {rl_ids[i]}: v_now={v_now:.3f}, v_prev={v_prev:.3f}, a={a0:.3f}")

    #     term_accel = - w_4 * (a0 ** 2)
    #     R_rear = scale * (term_speed_near + term_sync+term_accel)
    #     rewards[rl_ids[i]] = float(R_rear)

    #     return rewards








    # def compute_reward(self, rl_actions, **kwargs):
    #     """See class definition."""
    #     # in the warmup steps
    #     if rl_actions is None:
    #         return 0

    #     vel = np.array([
    #         self.k.vehicle.get_speed(veh_id)
    #         for veh_id in self.k.vehicle.get_ids()
    #     ])

    #     if any(vel < -100) or kwargs['fail']:
    #         return 0.

    #     # reward average velocity
        
    #     reward = np.mean(vel)
   

    #     # punish accelerations (should lead to reduced stop-and-go waves)
        
    #     # accels = [abs(action[0]) for action in rl_actions.values()]
    #     # mean_actions= np.mean(accels)
    #     # # mean_actions = np.mean(np.abs(list(rl_actions.values())))
    #     # accel_threshold = 0

    #     # std_vel = np.std(vel)
        
    #     # eta_2 = 2.0     # 平均速度奖励
    #     # eta = 1.0       # 加速度惩罚
    #     # eta_std = 0.5   # 速度一致性惩罚
    #     # # 更新后的 reward 结构
    #     # reward = eta_2 * np.mean(vel)           # vel ≈ 4 → reward ≈ 8.0
    #     # if mean_actions > accel_threshold:
    #     #     reward += eta * (accel_threshold - mean_actions)            # mean_actions ≈ 2.5 → -2.5
    #     # reward -= eta_std * std_vel             # std_vel ≈ 1.2 → -0.6


    #     return {key: reward for key in self.k.vehicle.get_rl_ids()}
        
    # def compute_reward(self, rl_actions, **kwargs):
    #     """
    #     Asymmetric smooth reward with 'paired mode':
    #     - Phase 1: approach (front car brakes to close the gap)
    #     - Phase 2: paired mode (once distance stays small for a while)
    #     → both maintain similar speed and stable distance
    #     """

    #     rl_ids = sorted(self.k.vehicle.get_rl_ids())
    #     if len(rl_ids) < 2:
    #         return {rid: 0.0 for rid in rl_ids}

    #     # === 基础信息 ===
    #     ring_len = self.k.network.length()
    #     v_ref = float(self.env_params.additional_params.get("speed_ref", 6.0))
       

    #     x = np.array([self.k.vehicle.get_x_by_id(i) for i in rl_ids])
    #     v = np.array([self.k.vehicle.get_speed(i)  for i in rl_ids])

    #     # === 有符号距离 ===
    #     raw = (x[1] - x[0]) % ring_len
    #     dx_signed = raw if raw <= ring_len / 2 else raw - ring_len
    #     abs_dx = abs(dx_signed)

    #     # 判定前后角色
    #     if dx_signed > 0:
    #         front_idx, rear_idx = 1, 0
    #     else:
    #         front_idx, rear_idx = 0, 1

    #     # === 参数 ===
    #     d_target = float(self.env_params.additional_params.get("target_dist", 0.0))  # 理想距离
    #     d_th   = float(self.env_params.additional_params.get("pair_dist_th", 3))   # 认为并排的距离阈值
    #     T_hold = int(self.env_params.additional_params.get("pair_hold_steps", 80))   # 连续多少步算 paired
    #     d_cap  = float(self.env_params.additional_params.get("pair_dist_cap", 50.0))

    #     # 权重
    #     w_close_front = float(self.env_params.additional_params.get("w_close_front", 3.0))
    #     w_speed_near  = float(self.env_params.additional_params.get("w_speed_near", 0.6))
    #     w_speed_far_punish_front = float(self.env_params.additional_params.get("w_speed_far_punish_front", 0.5))
    #     w_sync = float(self.env_params.additional_params.get("w_sync", 0.5))
    #     w_accel = float(self.env_params.additional_params.get("w_jerk", 0.4))
    #     scale = float(self.env_params.additional_params.get("reward_scale", 10.0))
    #     # === 加速度惩罚项 ===
    #     a0 = float(rl_actions.get(rl_ids[0], np.array([0.0]))[0]) if rl_ids[0] in rl_actions else 0.0
    #     a1 = float(rl_actions.get(rl_ids[1], np.array([0.0]))[0]) if rl_ids[1] in rl_actions else 0.0

    #     # 惩罚加速度绝对值平方，抑制持续大加减速（不再考虑上一步）
    #     accel_penalty = - w_accel * ((a0 ** 2) + (a1 ** 2))

    #     # === 初始化 paired 状态记录 ===
    #     if not hasattr(self, "pair_timer"):
    #         self.pair_timer = 0
    #     if not hasattr(self, "paired"):
    #         self.paired = False

    #     # 更新 pair_timer 状态
    #     if abs_dx <= d_th:
    #         self.pair_timer += 1
    #     else:
    #         self.pair_timer = 0

    #     if self.pair_timer >= T_hold:
    #         self.paired = True
    #     elif abs_dx > d_th * 2.0:  # 偏离太远时取消 paired
    #         self.paired = False

    #     dv = abs(v[0] - v[1])
    #     mean_v = np.mean(v)
    #     closeness = 1.0 - min(abs_dx, d_cap) / d_cap

    #     rewards = {}

    #     # === 如果已经进入 paired 模式 ===
    #     if self.paired:
    #         # 强化维持稳定距离和速度（平方误差）
    #         Kd = 2.0
    #         Kv = 1.0
    #         Kspeed = 0.4

    #         dist_err = (abs_dx - d_target)
    #         R_dist_sq = -Kd * (dist_err ** 2) / (ring_len ** 2)
    #         R_vel_sq  = -Kv * (dv ** 2) / (v_ref ** 2)
    #         R_speed   = Kspeed * (mean_v / v_ref)

    #         R_total = scale * (R_dist_sq + R_vel_sq + R_speed + accel_penalty)
    #         rewards = {rid: float(R_total) for rid in rl_ids}

    #     # === 否则仍在靠拢阶段 ===
    #     else:
    #         near_gate = 1.0 if abs_dx <= d_th else 0.0
    #         # 前车靠近
    #         i = front_idx
    #         dist_err = (abs_dx - d_target)
    #         term_close = - w_close_front * (dist_err ** 2) / (ring_len ** 2)
    #         term_speed_near = w_speed_near * near_gate * (v[i] / v_ref)
    #         term_speed_far_pen = - w_speed_far_punish_front * (1 - near_gate) * (v[i] / v_ref)
    #         term_sync = - w_sync * (dv / v_ref)
    #         R_front = scale * (term_close + term_speed_near + term_speed_far_pen + term_sync + accel_penalty)
    #         rewards[rl_ids[i]] = float(R_front)

    #         # 后车等待 + 同步
    #         i = rear_idx
    #         term_speed_near = w_speed_near * near_gate * (v[i] / v_ref)
    #         term_sync = - w_sync * (dv / v_ref)
    #         R_rear = scale * (term_speed_near + term_sync + accel_penalty)
    #         rewards[rl_ids[i]] = float(R_rear)

    #     return rewards


    def additional_command(self):
        """Define which vehicles are colored/observed for visualization purposes."""
        # 取一下网络参数        
        
        for rl_id in self.k.vehicle.get_rl_ids():
            if rl_id not in self.observe:
                continue 
            for veh_id in self.observe[rl_id]:
                 
                self.k.vehicle.set_observed(veh_id)
           

    def reset(self, new_inflow_rate=None):
        """See parent class.

        The sumo instance is reset with a new ring length, and a number of
        steps are performed with the rl vehicle acting as a human vehicle.
        """

        # skip if ring length is None
       
        self.observe = {}
        self._last_req_time = {}
        if self.env_params.additional_params['ring_length'] is None:
            return super().reset()

        # reset the step counter
        self.step_counter = 0
        self.total_step_counter =0 
        # update the network
        initial=InitialConfig(
            spacing="uniform",
            perturbation=0,
            shuffle=True,
            )
        # override_length = self.env_params.additional_params.get("worker_length_override", None)
        # if override_length is not None:
        #     self.env_params.additional_params["ring_length"] = override_length
        #     self.net_params.additional_params["length"] = override_length
        #     self.k.network.length = override_length
        # length = self.net_params.additional_params.get('length', 250)
        length = self.env_params.additional_params.get('ring_length', 250)
        # if isinstance(ring_length_param, (list, tuple)):
        #     length = random.randint(ring_length_param[0], ring_length_param[1])
        # else:
        #     length = ring_length_param
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
            net_params, initial)
        self.k.vehicle = deepcopy(self.initial_vehicles)
        self.k.vehicle.kernel_api = self.k.kernel_api
        self.k.vehicle.master_kernel = self.k

        # solve for the velocity upper bound of the ring
        # v_guess = 4
        # v_eq_max = fsolve(v_eq_max_function, np.array(v_guess),
        #                   args=(len(self.initial_ids)/self.net_params.additional_params['lanes'], length))[0]

        # print('\n-----------------------')
        # print('ring length:', net_params.additional_params['length'])
        # print('v_max:', v_eq_max)
        # print('-----------------------')

        # restart the sumo instance
        self.restart_simulation(
            sim_params=self.sim_params,
            render=self.sim_params.render)

      
        # perform the generic reset function
        obs = super().reset()


        return obs
  
    # def reset(self, new_inflow_rate=None):
    #     from flow.controllers import RLController, IDMController, ContinuousRouter
    #     from flow.core.params import VehicleParams, InitialConfig, NetParams, SumoCarFollowingParams
    #     from copy import deepcopy
    #     import random
    #     self.observe = {}
    #     self._last_req_time = {}
    #     # 1️⃣ 生成新车辆配置
    #     Total_Number_Veh = self.env_params.additional_params.get('Total_Number_Veh', 44)
    #     NUM_AUTOMATED = self.env_params.additional_params.get('NUM_AUTOMATED', 2)
    #     num_human = Total_Number_Veh - NUM_AUTOMATED
    #     human1 = random.randrange(0, 20, 2)
    #     humans_remaining = num_human - human1

    #     vehicles = VehicleParams()
    #     vehicles.add("rl0", (RLController, {}), routing_controller=(ContinuousRouter, {}), num_vehicles=1)
    #     vehicles.add("human0", (IDMController, {"noise": 0.2}),
    #                 car_following_params=SumoCarFollowingParams(min_gap=2),
    #                 routing_controller=(ContinuousRouter, {}), num_vehicles=human1)
    #     vehicles.add("rl1", (RLController, {}), routing_controller=(ContinuousRouter, {}), num_vehicles=NUM_AUTOMATED - 1)
    #     vehicles.add("human1", (IDMController, {"noise": 0.2}),
    #                 car_following_params=SumoCarFollowingParams(min_gap=2),
    #                 routing_controller=(ContinuousRouter, {}), num_vehicles=humans_remaining)

    #     # 2️⃣ 替换掉父类的 initial_vehicles
    #     self.initial_vehicles = deepcopy(vehicles)

    #     # 3️⃣ 更新网络参数（可选）
    #     length = self.env_params.additional_params.get('ring_length', 250)
    #     self.net_params.additional_params['length'] = length
   

        

    #     # 4️⃣ 调用父类 reset，继续后续渲染/更新逻辑
    #     state =super().reset()
   
    #     return state


    # def reset(self, **kwargs):
    #     """Reset with random human vehicle distribution around multiple RL agents."""
    #     import os, time, random, numpy as np
    #     unique_seed = (os.getpid() + int(time.time() * 1e6)) % 2**32
    #     random.seed(unique_seed)
    #     np.random.seed(unique_seed)

    #     self.observe = {}
    #     self._last_req_time = {}

    #     # ===== 参数提取 =====
    #     Total_Number_Veh = self.env_params.additional_params['Total_Number_Veh']
    #     NUM_AUTOMATED = self.env_params.additional_params['NUM_AUTOMATED']
    #     num_human = Total_Number_Veh - NUM_AUTOMATED
    #     human1 = random.randrange(0, 20, 2)
    #     print(f"[Reset] human1={human1}, seed state={random.getstate()[1][0]}")
    #     humans_remaining = num_human - human1

        # ===== 定义车辆 =====
        # vehicles = VehicleParams()
        # vehicles.add(
        #     "rl0",
        #     acceleration_controller=(RLController, {}),
        #     routing_controller=(ContinuousRouter, {}),
        #     num_vehicles=1,
        # )
        # vehicles.add(
        #     "human0",
        #     acceleration_controller=(IDMController, {"noise": 0.2}),
        #     car_following_params=SumoCarFollowingParams(min_gap=2),
        #     routing_controller=(ContinuousRouter, {}),
        #     num_vehicles=human1,
        # )
        # vehicles.add(
        #     "rl1",
        #     acceleration_controller=(RLController, {}),
        #     routing_controller=(ContinuousRouter, {}),
        #     num_vehicles=NUM_AUTOMATED - 1,
        # )
        # vehicles.add(
        #     "human1",
        #     acceleration_controller=(IDMController, {"noise": 0.2}),
        #     car_following_params=SumoCarFollowingParams(min_gap=2),
        #     routing_controller=(ContinuousRouter, {}),
        #     num_vehicles=humans_remaining,
        # )

    #     # ===== 禁止 Flow 的 shuffle =====
    #     self.initial_config = InitialConfig(spacing="uniform", shuffle=False)

    #     # ✅ 核心修改点：不再手动构建 network，不再调用 setup_initial_state
    #     # Flow 会在 super().reset() 里自动生成 edge / route / starting positions
    #     # 我们只更新车辆定义
    #     self.veh = vehicles
    #     self.k.vehicle.vehicle_params = vehicles

    #     # ===== 调用父类 reset（让 Flow 完全接管初始化逻辑） =====
    #     obs = super().reset(**kwargs)

    #     print(f"[Reset done] {len(self.k.vehicle.get_ids())} vehicles initialized, "
    #         f"{len(self.k.vehicle.get_rl_ids())} RL vehicles, seed={unique_seed}")

    #     return obs


class RandomDriverWaveEnv(MultiAgentWaveAttenuationPOEnv):

   
    def __init__(self, env_params=None, sim_params=None, network=None, simulator='traci', **kwargs):
        flow_params = dict(
            env=env_params,
            sim=sim_params,
            net=network,
            initial=None,
            veh=None,
            simulator=simulator
        )
        self.flow_params = flow_params  # 存一份
        super().__init__(env_params, sim_params, network, simulator)


    def add_random_vehicles_with_total(
        self,
        vehicles: VehicleParams,
        prefix: str,
        indices: range,
        total_vehicles: int
    ):
        """
        Distribute `total_vehicles` randomly across types prefix_i,
        with each type getting randomized car-following and lane-change params.
        """
        k = len(indices)
        # Optionally guard:
        # assert total_vehicles >= 0, "total_vehicles must be non‐negative"
        # Use multinomial to allow zeros:
        #counts = np.random.default_rng(SEED).multinomial(total_vehicles, [1/k]*k)
        counts = np.random.multinomial(total_vehicles, [1/k]*k)
        counts = [int(c) for c in counts]

        for i, num_veh in zip(indices, counts):
            # sample random parameters
            noise   = random.uniform(0.0, 1.0)
            min_gap = random.uniform(0.2, 2.5)
            accel   = random.uniform(1.0, 3.0)
            decel   = random.uniform(2.0, 5.0)
            lc_assertive   = random.uniform(0.1, 2.5)
            lc_cooperative = random.uniform(0.1, 2.5)
            lc_speed_gain  = random.uniform(0.1, 2.0)
            lc_keep_right  = random.choice([0.0, 1.0])
            min_gap_lat    = random.uniform(0.1, 0.5)
            max_speed   = random.choice([2.0, 3.0, 5.0, 11.0])

            print(f"Type {prefix}_{i}: count={num_veh}, "
                f"noise={noise:.3f}, min_gap={min_gap:.2f}, "
                f"accel={accel:.2f}, decel={decel:.2f}, "
                f"lc_assertive={lc_assertive:.2f}, "
                f"lc_cooperative={lc_cooperative:.2f}, "
                f"lc_speed_gain={lc_speed_gain:.2f}, "
                f"lc_keep_right={lc_keep_right:.1f}, "
                f"min_gap_lat={min_gap_lat:.2f}"
                f"max_speed={max_speed:.2f}"
            )

            vehicles.add(
                veh_id=f"{prefix}_{i}",
                acceleration_controller=(IDMController, {
                    "noise": noise
                }),
                car_following_params=SumoCarFollowingParams(
                    min_gap=min_gap,
                    accel=accel,
                    decel=decel,
                    sigma=noise,
                    max_speed=max_speed
                ),
                routing_controller=(ContinuousRouter, {}),
                lane_change_controller=(SimLaneChangeController, {}),
                lane_change_params=SumoLaneChangeParams(
                    model="LC2013",
                    lane_change_mode=1621,
                    lc_assertive=lc_assertive,
                    lc_cooperative=lc_cooperative,
                    lc_speed_gain=lc_speed_gain,
                    lc_keep_right=lc_keep_right,
                    min_gap_lat=min_gap_lat
                ),
                num_vehicles=num_veh
            )

    
    
    
    def reset(self, **kwargs):
        # 1) build brand‐new VehicleParams
        vehicles = VehicleParams()
        Total_Number_Veh = self.env_params.additional_params['Total_Number_Veh']
        NUM_AUTOMATED = self.env_params.additional_params['NUM_AUTOMATED']
        k = self.env_params.additional_params['DriverModel']  # same number of human types
        num_human = Total_Number_Veh - NUM_AUTOMATED
        humans_remaining = num_human
        for i in range(NUM_AUTOMATED):
            vehicles.add(
                veh_id="rl_{}".format(i),
                acceleration_controller=(RLController, {}),
                # lane_change_controller=None,
                lane_change_controller=(SimLaneChangeController, {}),
                lane_change_params=SumoLaneChangeParams(
                    model="LC2013",
                    lane_change_mode=512,   # 完全 RL 控制（如果你实现了 RL lateral）
                    min_gap_lat=0.1,
                    lc_keep_right=0.0
                    ),
                routing_controller=(ContinuousRouter, {}),
                num_vehicles=1)
            vehicles_to_add = round(humans_remaining / (NUM_AUTOMATED - i))
            humans_remaining -= vehicles_to_add
            self.add_random_vehicles_with_total(
                    vehicles,
                    prefix="human",
                    indices=range(k),
                    total_vehicles=vehicles_to_add
                )
    

        # 2) replace the env’s VehicleParams
        self.veh = vehicles
        self.flow_params["veh"] = vehicles
        self.setup_initial_state()
        # 3) call the parent reset
        return super().reset(**kwargs)


