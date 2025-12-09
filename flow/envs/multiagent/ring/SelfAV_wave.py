"""
Environment used to train a stop-and-go dissipating controller.

This is the environment that was used in:

C. Wu, A. Kreidieh, K. Parvate, E. Vinitsky, A. Bayen, "Flow: Architecture and
Benchmarking for Reinforcement Learning in Traffic Control," CoRR, vol.
abs/1710.05465, 2017. [Online]. Available: https://arxiv.org/abs/1710.05465
"""
import traci
import numpy as np
from gym.spaces.box import Box
import random
from scipy.optimize import fsolve
from copy import deepcopy
from flow.core.params import VehicleParams, SumoCarFollowingParams, SumoLaneChangeParams
from flow.core.params import InitialConfig
from flow.controllers import IDMController, RLController, ContinuousRouter, SimLaneChangeController
from flow.core.params import NetParams
from flow.envs.multiagent.base import MultiEnv
from flow.envs.ring.wave_attenuation import v_eq_max_function


ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration of autonomous vehicles
    'max_accel': 1,
    # maximum deceleration of autonomous vehicles
    'max_decel': 1,
    # bounds on the ranges of ring road lengths the autonomous vehicle is
    # trained on
    'ring_length': [220, 270],
    'lane_change_duration': 5,
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
        return Box(low=-1, high=1, shape=(3,), dtype=np.float32)

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
        for p in ADDITIONAL_ENV_PARAMS.keys():
            if p not in env_params.additional_params:
                raise KeyError(
                    'Environment parameter \'{}\' not supplied'.format(p))

        super().__init__(env_params, sim_params, network, simulator)

    @property
    def observation_space(self):
        """See class definition."""
        return Box(low=-5, high=5, shape=(10,), dtype=np.float32)

    @property
    def action_space(self):
        max_dec = np.abs(self.env_params.additional_params['max_decel'])
        max_acc = self.env_params.additional_params['max_accel']

        low = np.array([-max_dec, -1.0], dtype=np.float32)
        high = np.array([ max_acc,  1.0], dtype=np.float32)

        # 移除 shape 参数，Gym 会根据 low/high 的 shape 自动设定
        return Box(low=low, high=high, dtype=np.float32)

    
    
    def get_state(self):
        """See class definition."""
        obs = {}
        self.observe = {}

        n_lanes   = self.net_params.additional_params['lanes']
        max_speed = self.k.network.max_speed()
        ring_len  = self.k.network.length()

        for rl_id in self.k.vehicle.get_rl_ids():
            # 0. 自身速度 & 1. 车道号
            v    = self.k.vehicle.get_speed(rl_id)
            lane = int(self.k.vehicle.get_lane(rl_id))
            leaders    = self.k.vehicle.get_lane_leaders(rl_id)
            followers  = self.k.vehicle.get_lane_followers(rl_id)
            headways   = self.k.vehicle.get_lane_headways(rl_id)
            tailways   = self.k.vehicle.get_lane_tailways(rl_id)
            features = v
            for l in range(n_lanes):
                is_rl_lane = (l == lane)

                if is_rl_lane:
                    lead_id = leaders[lane] or rl_id
                    
                    dx_f = headways[lane] if lane < len(headways) else 0.0
                    dv_f = self.k.vehicle.get_speed(lead_id)
                    
                    follow_id = followers[lane] or rl_id
                    dx_b = tailways[lane] if lane < len(tailways) else 0.0
                    dv_b = self.k.vehicle.get_speed(follow_id)

                else:
                    other_lane = (lane + 1) % n_lanes

                    # 前车
                    ahead_id   = leaders[other_lane] or rl_id
                    dv_f      = self.k.vehicle.get_speed(ahead_id) 
                    dx_f      = headways[other_lane] if other_lane < len(headways) else 0.0

                    # 后车
                    back_id    = followers[other_lane] or rl_id
                    dv_b      = v - self.k.vehicle.get_speed(back_id)
                    dx_b      = tailways[other_lane] if other_lane < len(tailways) else 0.0
                features.extend = np.array([
                    is_rl_lane,
                    dv_f / max_speed,       # 2: 1车道前车速度差
                    dx_f / ring_len,        # 3: 1车道前车距离
                    dv_b / max_speed,       # 4: 1车道后车速度差
                    dx_b / ring_len,        # 5: 1车道后车距离
                          ])

            # 记录可观测车辆 ID
            self.observe[rl_id] = np.array([lead_id, follow_id, ahead_id, back_id])
            obs[rl_id] = features

        return obs


  

    # def _apply_rl_actions(self, rl_actions):
    #     """Split the accelerations by ring."""
    #     if rl_actions:
    #         rl_ids = list(rl_actions.keys())
    #         accel = list(rl_actions.values())
    #         self.k.vehicle.apply_acceleration(rl_ids, accel)


    def _apply_rl_actions(self, rl_actions):
        """Split the actions into acceleration and lane-change and apply both."""
        if not rl_actions:
            return

        rl_ids = list(rl_actions.keys())
        accels = []
        lane_changes = []

        for act in rl_actions.values():
            # act is a length-2 array: [acceleration, continuous_lane_change]
            accel, lc_cont = act

            # 1) 加速度
            accels.append(accel)

            # 2) 连续变道信号 → 三档整数 -1, 0, +1
            lc_cmd = int(np.round(lc_cont))
            lane_changes.append(lc_cmd)
        cooldown = self.env_params.additional_params['lane_change_duration']
        current_time = self.k.kernel_api.simulation.getTime()
        old_lanes = {vid: self.k.vehicle.get_lane(vid) for vid in rl_ids}
        final_lcs = []
        for vid, cmd in zip(rl_ids, lane_changes):
            last_time = self._last_req_time.get(vid, -1e9)
            blocked   = (current_time <= last_time + cooldown)
            # 如果在冷却期或者根本没要变道，就置零
            if blocked or cmd == 0:
                final_lcs.append(0)
            else:
                final_lcs.append(cmd)


            #print(f"[DEBUG] vid={vid}, now={current_time:.1f}s, "
             #     f"last_req={last_time:.1f}s, blocked={blocked}")
                # 再屏蔽冷却期中的指令
                      
        
        self.k.vehicle.apply_acceleration(rl_ids, accels)
                
        # 再执行变道指令（-1 左变道，0 不变道，+1 右变道）
        #print(final_lcs)
        self.k.vehicle.apply_lane_change(rl_ids, final_lcs)
        for vid, cmd in zip(rl_ids, final_lcs):
            if cmd != 0:
                new_lane = self.k.vehicle.get_lane(vid)
                old_lane = old_lanes[vid]
                if new_lane != old_lane:
                    self._last_req_time[vid] = current_time
        # for vid in self.k.vehicle.get_ids():
        #     mode = self.k.kernel_api.vehicle.getLaneChangeMode(vid)
        #     print(f"[CHECK] veh={vid} lane_change_mode={mode}")


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
        
        reward = np.mean(vel)

        # punish accelerations (should lead to reduced stop-and-go waves)
        
        # accels = [abs(action[0]) for action in rl_actions.values()]
        # mean_actions= np.mean(accels)
        # # mean_actions = np.mean(np.abs(list(rl_actions.values())))
        # accel_threshold = 0

        # std_vel = np.std(vel)
        
        # eta_2 = 2.0     # 平均速度奖励
        # eta = 1.0       # 加速度惩罚
        # eta_std = 0.5   # 速度一致性惩罚
        # # 更新后的 reward 结构
        # reward = eta_2 * np.mean(vel)           # vel ≈ 4 → reward ≈ 8.0
        # if mean_actions > accel_threshold:
        #     reward += eta * (accel_threshold - mean_actions)            # mean_actions ≈ 2.5 → -2.5
        # reward -= eta_std * std_vel             # std_vel ≈ 1.2 → -0.6


        return {key: reward for key in self.k.vehicle.get_rl_ids()}

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

        # update the network
        initial_config = InitialConfig(bunching=50, min_gap=0)
        override_length = self.env_params.additional_params.get("worker_length_override", None)
        if override_length is not None:
            self.env_params.additional_params["ring_length"] = override_length
            self.net_params.additional_params["length"] = override_length
            self.k.network.length = override_length
        length = self.env_params.additional_params.get('ring_length', 250)
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
        v_guess = 4
        v_eq_max = fsolve(v_eq_max_function, np.array(v_guess),
                          args=(len(self.initial_ids)/self.net_params.additional_params['lanes'], length))[0]

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
