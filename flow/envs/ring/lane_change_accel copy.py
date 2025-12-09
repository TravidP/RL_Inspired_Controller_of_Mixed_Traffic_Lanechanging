"""Environments that can train both lane change and acceleration behaviors."""

from flow.envs.ring.accel import AccelEnv
from flow.core import rewards

from gym.spaces.box import Box
import numpy as np

ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration for autonomous vehicles, in m/s^2
    "max_accel": 3,
    # maximum deceleration for autonomous vehicles, in m/s^2
    "max_decel": 3,
    # lane change duration for autonomous vehicles, in s. Autonomous vehicles
    # reject new lane changing commands for this duration after successfully
    # changing lanes.
    "lane_change_duration": 5,
    # desired velocity for all vehicles in the network, in m/s
    "target_velocity": 10,
    # specifies whether vehicles are to be sorted by position during a
    # simulation step. If set to True, the environment parameter
    # self.sorted_ids will return a list of all vehicles sorted in accordance
    # with the environment
    'sort_vehicles': False
}


class LaneChangeAccelEnv(AccelEnv):
    """Fully observable lane change and acceleration environment.

    This environment is used to train autonomous vehicles to improve traffic
    flows when lane-change and acceleration actions are permitted by the rl
    agent.

    Required from env_params:

    * max_accel: maximum acceleration for autonomous vehicles, in m/s^2
    * max_decel: maximum deceleration for autonomous vehicles, in m/s^2
    * lane_change_duration: lane change duration for autonomous vehicles, in s
    * target_velocity: desired velocity for all vehicles in the network, in m/s
    * sort_vehicles: specifies whether vehicles are to be sorted by position
      during a simulation step. If set to True, the environment parameter
      self.sorted_ids will return a list of all vehicles sorted in accordance
      with the environment

    States
        The state consists of the velocities, absolute position, and lane index
        of all vehicles in the network. This assumes a constant number of
        vehicles.

    Actions
        Actions consist of:

        * a (continuous) acceleration from -abs(max_decel) to max_accel,
          specified in env_params
        * a (continuous) lane-change action from -1 to 1, used to determine the
          lateral direction the vehicle will take.

        Lane change actions are performed only if the vehicle has not changed
        lanes for the lane change duration specified in env_params.

    Rewards
        The reward function is the two-norm of the distance of the speed of the
        vehicles in the network from a desired speed, combined with a penalty
        to discourage excess lane changes by the rl vehicle.

    Termination
        A rollout is terminated if the time horizon is reached or if two
        vehicles collide into one another.
    """

    def __init__(self, env_params, sim_params, network, simulator='traci'):
        for p in ADDITIONAL_ENV_PARAMS.keys():
            if p not in env_params.additional_params:
                raise KeyError(
                    'Environment parameter "{}" not supplied'.format(p))

        super().__init__(env_params, sim_params, network, simulator)

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
        return Box(
            low=0,
            high=1,
            shape=(3 * self.initial_vehicles.num_vehicles, ),
            dtype=np.float32)

    def compute_reward(self, rl_actions, **kwargs):
        """See class definition."""
        # compute the system-level performance of vehicles from a velocity
        # perspective
        reward = rewards.average_velocity(self, fail=kwargs["fail"])

        # punish excessive lane changes by reducing the reward by a set value
        # every time an rl car changes lanes (10% of max reward)
        # for veh_id in self.k.vehicle.get_rl_ids():
        #     if self.k.vehicle.get_last_lc(veh_id) == self.time_counter:
        #         reward -= 0.1

        return reward

    def get_state(self):
        """See class definition."""
        # normalizers
        max_speed = self.k.network.max_speed()
        length = self.k.network.length()
        max_lanes = max(
            self.k.network.num_lanes(edge)
            for edge in self.k.network.get_edge_list())

        speed = [self.k.vehicle.get_speed(veh_id) / max_speed
                 for veh_id in self.sorted_ids]
        pos = [self.k.vehicle.get_x_by_id(veh_id) / length
               for veh_id in self.sorted_ids]
        lane = [self.k.vehicle.get_lane(veh_id) / max_lanes
                for veh_id in self.sorted_ids]

        return np.array(speed + pos + lane)

    def _apply_rl_actions(self, actions):
        acceleration = actions[::2]
        raw_dir    = actions[1::2]   # 这是一个连续 [-1,1] 向量

        # 离散化：任何正值视为 +1，负值视为 -1，接近 0 的（绝对值<阈值）可视为 0
        # 这里阈值可以设得很小，比如 0.05
        dir_discrete = np.zeros_like(raw_dir, dtype=int)
        dir_discrete[ raw_dir >  0.05 ] =  1
        dir_discrete[ raw_dir < -0.05 ] = -1

        # 其它逻辑（排序、非换道期间置 0）保持不变
        sorted_rl_ids = [vid for vid in self.sorted_ids
                        if vid in self.k.vehicle.get_rl_ids()]

        non_lane_changing = [
            self.time_counter <=
            self.env_params.additional_params["lane_change_duration"]
            + self.k.vehicle.get_last_lc(vid)
            for vid in sorted_rl_ids
        ]
        # 在“非换道期”把意图置 0
        dir_discrete[non_lane_changing] = 0

        # 最后把离散结果传给 TraCI
        self.k.vehicle.apply_acceleration(sorted_rl_ids, acc=acceleration)
        self.k.vehicle.apply_lane_change(sorted_rl_ids, direction=dir_discrete)

    def additional_command(self):
        """Define which vehicles are observed for visualization purposes."""
        # specify observed vehicles
        if self.k.vehicle.num_rl_vehicles > 0:
            for veh_id in self.k.vehicle.get_human_ids():
                self.k.vehicle.set_observed(veh_id)


class LaneChangeAccelPOEnv(LaneChangeAccelEnv):
    """POMDP version of LaneChangeAccelEnv.

    Required from env_params:

    * max_accel: maximum acceleration for autonomous vehicles, in m/s^2
    * max_decel: maximum deceleration for autonomous vehicles, in m/s^2
    * lane_change_duration: lane change duration for autonomous vehicles, in s
    * target_velocity: desired velocity for all vehicles in the network, in m/s

    States
        States are a list of rl vehicles speeds, as well as the speeds and
        bumper-to-bumper headways between the rl vehicles and their
        leaders/followers in all lanes. There is no assumption on the number of
        vehicles in the network, so long as the number of rl vehicles is
        static.

    Actions
        See parent class.

    Rewards
        See parent class.

    Termination
        See parent class.

    Attributes
    ----------
    num_lanes : int
        maximum number of lanes on any edge in the network
    visible : list of str
        lists of visible vehicles, used for visualization purposes
    """

    def __init__(self, env_params, sim_params, network, simulator='traci'):
        super().__init__(env_params, sim_params, network, simulator)

        self.num_lanes = max(self.k.network.num_lanes(edge)
                             for edge in self.k.network.get_edge_list())
        self.visible = []

    @property
    def observation_space(self):
        """See class definition."""
        # return Box(
        #     low=0,
        #     high=1,
        #     shape=(4 * self.initial_vehicles.num_rl_vehicles *
        #            self.num_lanes + self.initial_vehicles.num_rl_vehicles, ),
        #     dtype=np.float32)
        num_rl = self.initial_vehicles.num_rl_vehicles
        obs_dim = 4 * num_rl * self.num_lanes + num_rl
        return Box(low=0, high=1, shape=(obs_dim,), dtype=np.float32)

    def get_state(self):
        """See class definition."""
        num_rl = self.k.vehicle.num_rl_vehicles
        obs = [0.0] * (4 * num_rl * self.num_lanes + num_rl)
        self.visible = []

        max_length = self.k.network.length()
        max_speed = self.k.network.max_speed()

        for i, rl_id in enumerate(self.k.vehicle.get_rl_ids()):
            # 初始化局部列表
            headway = [1.0] * self.num_lanes
            tailway = [1.0] * self.num_lanes
            vel_in_front = [0.0] * self.num_lanes
            vel_behind = [0.0] * self.num_lanes

            # 获取邻近车辆信息
            lane_headways = self.k.vehicle.get_lane_headways(rl_id)
            lane_tailways = self.k.vehicle.get_lane_tailways(rl_id)
            lane_leaders = self.k.vehicle.get_lane_leaders(rl_id)
            lane_followers = self.k.vehicle.get_lane_followers(rl_id)

            # 归一化距离
            for j in range(len(lane_headways)):
                headway[j] = lane_headways[j] / max_length
            for j in range(len(lane_tailways)):
                tailway[j] = lane_tailways[j] / max_length

            # 归一化速度
            for j, leader in enumerate(lane_leaders):
                if leader:
                    vel_in_front[j] = self.k.vehicle.get_speed(leader) / max_speed
                    self.visible.append(leader)
            for j, follower in enumerate(lane_followers):
                if follower:
                    vel_behind[j] = self.k.vehicle.get_speed(follower) / max_speed
                    self.visible.append(follower)

            # 写入向量片段
            start = 4 * self.num_lanes * i
            end   = 4 * self.num_lanes * (i + 1)
            obs[start:end] = headway + tailway + vel_in_front + vel_behind

            # 写入自身速度
            ego_idx = 4 * self.num_lanes * num_rl + i
            obs[ego_idx] = self.k.vehicle.get_speed(rl_id) / max_speed
    # 之前的 obs 构造逻辑 ...

        obs = np.array(obs, dtype=np.float32)
        # 关键：裁剪到 [0,1]
        obs = np.clip(obs, 0.0, 1.0)
        return obs

    def additional_command(self):
        """Define which vehicles are observed for visualization purposes."""
        # specify observed vehicles
        for veh_id in self.visible:
            self.k.vehicle.set_observed(veh_id)
