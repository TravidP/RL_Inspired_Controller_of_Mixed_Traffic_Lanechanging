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
    'ring_length': 250,
    'lane_change_duration': 5,
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

        # # punish accelerations (should lead to reduced stop-and-go waves)
        # eta = 4  # 0.25
        # mean_actions = np.mean(np.abs(np.array(rl_actions)))
        # accel_threshold = 0

        # if mean_actions > accel_threshold:
        #     reward += eta * (accel_threshold - mean_actions)

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
        self.observe = {}
        # skip if ring length is None
        if self.env_params.additional_params['ring_length'] is None:
            return super().reset()

        # reset the step counter
        self.step_counter = 0

        # update the network
        initial_config = InitialConfig(bunching=50, min_gap=0)
        length = self.env_params.additional_params['ring_length']
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
    def action_space(self):
        """See class definition."""
        max_decel = self.env_params.additional_params["max_decel"]
        max_accel = self.env_params.additional_params["max_accel"]

        lb = [-abs(max_decel), -1] * self.initial_vehicles.num_rl_vehicles
        ub = [max_accel, 1] * self.initial_vehicles.num_rl_vehicles

        return Box(np.array(lb), np.array(ub), dtype=np.float32)
    @property
    def observation_space(self):
        """See class definition."""
        return Box(low=-float('inf'), high=float('inf'),
                   shape=(11, ), dtype=np.float32)

    # def get_state(self):
    #     """See class definition."""
    #     rl_id = self.k.vehicle.get_rl_ids()[0]
    #     lead_id = self.k.vehicle.get_leader(rl_id) or rl_id

    #     # normalizers
    #     max_speed = 15.
    #     if self.env_params.additional_params['ring_length'] is not None:
    #         max_length = self.env_params.additional_params['ring_length'][1]
    #     else:
    #         max_length = self.k.network.length()

    #     observation = np.array([
    #         self.k.vehicle.get_speed(rl_id) / max_speed,
    #         (self.k.vehicle.get_speed(lead_id) -
    #          self.k.vehicle.get_speed(rl_id)) / max_speed,
    #         (self.k.vehicle.get_x_by_id(lead_id) -
    #          self.k.vehicle.get_x_by_id(rl_id)) % self.k.network.length()
    #         / max_length
    #     ])

    #     return observation
        
    def get_state(self):
        """
        New single-AV observation structure:

        obs = [
            rl_speed,
            for each lane:
                is_rl_lane,
                dist_to_front,
                front_speed,
                dist_to_back,
                back_speed
        ]

        dist, fdist are normalized by ring length
        speed is normalized by max_speed
        """

        rl_ids = self.k.vehicle.get_rl_ids()
        assert len(rl_ids) >= 1, "Environment must contain at least one AV."

        rl_id = rl_ids[0]
        other_rl_id = rl_ids[1] if len(rl_ids) > 1 else None

        # ----- constants -----
        n_lanes   = self.net_params.additional_params['lanes']
        max_speed = 15
        ring_len  = self.k.network.length()

        # ----- build obs -----
        obs = []

        # =============== 1) 自身速度 ===============
        v_rl = self.k.vehicle.get_speed(rl_id)
        lane_rl = int(self.k.vehicle.get_lane(rl_id))
        obs.append(v_rl / max_speed)

        # =============== SUMO raw info ===============
        leaders   = self.k.vehicle.get_lane_leaders(rl_id)
        followers = self.k.vehicle.get_lane_followers(rl_id)
        headways  = self.k.vehicle.get_lane_headways(rl_id)
        tailways  = self.k.vehicle.get_lane_tailways(rl_id)

        observe_ids = []

        # =============== 2) 每一条 lane 的信息 ===============
        for lane in range(n_lanes):

            # 是否是 AV 所在车道
            is_rl_lane = (lane == lane_rl)
            obs.append(1.0 if is_rl_lane else 0.0)

            # ---- front vehicle ----
            lead_id = leaders[lane] or rl_id
            v_f  = self.k.vehicle.get_speed(lead_id)
            dx_f = headways[lane] if lane < len(headways) else 0.0

            # ---- back vehicle ----
            back_id = followers[lane] or rl_id
            v_b  = self.k.vehicle.get_speed(back_id)
            dx_b = tailways[lane] if lane < len(tailways) else 0.0

            # ---- obs 添加 ----
            obs.extend([
                dx_f / ring_len,
                v_f / max_speed,
                dx_b / ring_len,
                v_b / max_speed,
            ])

            # ---- 记录原始 ID ----
            observe_ids.extend([lead_id, back_id])

        # ---- include second AV if exists ----
        observe_ids.append(other_rl_id)

        # 原始 ID 记录
        self.observe = np.array(observe_ids, dtype=object)

        return np.array(obs, dtype=np.float32)


    def _apply_rl_actions(self, actions):
        """
        Single-AV version.
        actions = [acceleration, raw_direction]
        - acceleration: float
        - raw_direction: continuous in [-1, 1]
        """

        rl_ids = self.k.vehicle.get_rl_ids()
        if len(rl_ids) != 1:
            raise ValueError("Single-AV environment: must have exactly 1 RL vehicle.")

        vid = rl_ids[0]

        # ---------------------
        # 1. 拆分动作
        # ---------------------
        acc = float(actions[0])
        raw_dir = float(actions[1])  # continuous in [-1, 1]

        # ---------------------
        # 2. 离散化换道动作
        # ---------------------
        if raw_dir > 1/3:
            dir_discrete = 1
        elif raw_dir < -1/3:
            dir_discrete = -1
        else:
            dir_discrete = 0

        # ---------------------
        # 3. 非换道周期内禁止换道
        # ---------------------
        last_lc = self.k.vehicle.get_last_lc(vid)
        lc_duration = self.env_params.additional_params["lane_change_duration"]

        # 仍处于不允许换道的时间窗口
        if self.time_counter <= last_lc + lc_duration:
            dir_discrete = 0

        # ---------------------
        # 4. 应用加速度 & 换道指令
        # ---------------------
        self.k.vehicle.apply_acceleration(vid, acc)
        self.k.vehicle.apply_lane_change(vid, direction=dir_discrete)


    def additional_command(self):
        """Define which vehicles are observed for visualization purposes."""
        # specify observed vehicles
        # rl_id = self.k.vehicle.get_rl_ids()[0]
        # lead_id = self.k.vehicle.get_leader(rl_id) or rl_id
        # self.k.vehicle.set_observed(lead_id)
        for veh_id in self.observe:     
            self.k.vehicle.set_observed(veh_id)