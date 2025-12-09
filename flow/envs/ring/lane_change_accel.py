"""Environments that can train both lane change and acceleration behaviors."""

from flow.envs.ring.accel import AccelEnv
from flow.core import rewards
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams
from copy import deepcopy

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
    "lane_change_duration": 1,
    # desired velocity for all vehicles in the network, in m/s
    "target_velocity": 30,
    # specifies whether vehicles are to be sorted by position during a
    # simulation step. If set to True, the environment parameter
    # self.sorted_ids will return a list of all vehicles sorted in accordance
    # with the environment
    'sort_vehicles': False,
    "ring_length": 250,
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

    # def compute_reward(self, rl_actions, **kwargs):
    #     """See class definition."""
    #     rl_ids = self.k.vehicle.get_rl_ids()
    #     # if kwargs.get("fail", False) or len(rl_ids) == 0:
    #     #     return -100.0

    #     speeds = self.k.vehicle.get_speed(rl_ids)
        
    #     # print(f"[REWARD DEBUG] rl_ids = {rl_ids}")
    #     # print(f"[REWARD DEBUG] speeds = {speeds}")
    #     # print(f"[REWARD DEBUG] reward = {speeds}")
    #     # rewards = {rl_id: float(s) for rl_id, s in zip(rl_ids, speeds)}
    #     return float(np.mean(speeds))/30
    
    def compute_reward(self, rl_actions, **kwargs):
        # """Compute reward as the average speed of all vehicles."""
        # all_ids = self.k.vehicle.get_ids()
        # speeds = self.k.vehicle.get_speed(all_ids)
        # # print (speeds)
        # if len(speeds) == 0:
        #     return 0.0  # 或者 return -100.0，防止除以 0
        # return float(np.mean(speeds)) / 30 
    #   --------------------------------------------
        # rl_ids = self.k.vehicle.get_rl_ids()          # 获取所有 RL 车辆 ID
        # speeds = self.k.vehicle.get_speed(rl_ids)     # 获取 RL 车辆速度列表

        # # 防止空列表（比如 reset 初始阶段）
        # if len(speeds) == 0:
        #     return 0.0

        # # 计算 RL 平均速度并做归一化
        # return float(np.mean(speeds)) / 30.0
    
    #---------------------------------------------Platoon reward
    # """Compute reward as the average lane speed of each RL vehicle's lane."""
        # rl_ids = self.k.vehicle.get_rl_ids()
        # if len(rl_ids) == 0:
        #     return 0.0

        # lane_speeds = []
        # all_ids = self.k.vehicle.get_ids()

        # for rl_id in rl_ids:
        #     # 获取该RL车辆所在车道
        #     lane_id = self.k.vehicle.get_lane(rl_id)

        #     # 获取同车道车辆
        #     lane_vehicles = [
        #         vid for vid in all_ids
        #         if self.k.vehicle.get_lane(vid) == lane_id
        #     ]

        #     if len(lane_vehicles) == 0:
        #         continue

        #     # 计算该车道的平均速度
        #     lane_speed = np.mean(self.k.vehicle.get_speed(lane_vehicles))
        #     lane_speeds.append(lane_speed)

        # if len(lane_speeds) == 0:
        #     return 0.0

        # # 返回RL所在车道的平均速度，并归一化
        # return float(np.mean(lane_speeds)) / 30.0
    
    # def compute_reward(self, rl_actions, **kwargs):
        # """Reward = 0.5 * avg speed of all vehicles + 0.5 * avg speed of RL vehicles"""
        # all_ids = self.k.vehicle.get_ids()
        # rl_ids = self.k.vehicle.get_rl_ids()

        # # 获取速度
        # all_speeds = self.k.vehicle.get_speed(all_ids)
        # rl_speeds = self.k.vehicle.get_speed(rl_ids)

        # if len(all_speeds) == 0 or len(rl_speeds) == 0:
        #     return 0.0

        # # 归一化：假设最大速度为 30 m/s
        # all_avg_speed = np.mean(all_speeds) / 30
        # rl_avg_speed = np.mean(rl_speeds) / 30

        # # 加权求和
        # reward = 0.5 * all_avg_speed + 0.5 * rl_avg_speed
        # return float(reward)


        # # return float(np.mean(speeds)) / 30 - float(np.std(speeds)) / 30  # 假设 30 是最大速度进行归一化
        # """Compute reward as lane platoon average speed minus lane speed std deviation."""
        rl_ids = self.k.vehicle.get_rl_ids()
        if len(rl_ids) == 0:
            return 0.0

        lane_rewards = []
        all_ids = self.k.vehicle.get_ids()
        V_MAX = 30.0      # 最大速度 (m/s)
        STD_WEIGHT = 0.5  # 标准差惩罚权重，可根据实验调节

        for rl_id in rl_ids:
            # 获取该 RL 车辆所在车道
            lane_id = self.k.vehicle.get_lane(rl_id)

            # 获取同车道所有车辆
            lane_vehicles = [
                vid for vid in all_ids
                if self.k.vehicle.get_lane(vid) == lane_id
            ]

            if len(lane_vehicles) == 0:
                continue

            # 计算该车道速度列表
            speeds = np.array(self.k.vehicle.get_speed(lane_vehicles))

            # 计算平均速度与标准差
            lane_mean = np.mean(speeds)
            lane_std = np.std(speeds)

            # 奖励：平均速度高 + 速度分布稳定
            reward_lane = (lane_mean / V_MAX) - STD_WEIGHT * (lane_std / V_MAX)
            lane_rewards.append(reward_lane)

        if len(lane_rewards) == 0:
            return 0.0

        # 所有RL车所在车道的平均奖励
        return float(np.mean(lane_rewards))
    
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
        dir_discrete[raw_dir > 1/3] = 1
        dir_discrete[raw_dir < -1/3] = -1

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

        # update the network
        initial_config = InitialConfig(bunching=50, min_gap=0)
        length =self.env_params.additional_params['ring_length']
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
       
        # restart the sumo instance
        self.restart_simulation(
            sim_params=self.sim_params,
            render=self.sim_params.render)

        # perform the generic reset function
        return super().reset()


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
        return Box(low=-5, high=5, shape=(11,), dtype=np.float32)

    def get_state(self):
        """See class definition."""
        obs = []
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
            features = [v/max_speed]
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
                    dv_b      = self.k.vehicle.get_speed(back_id)
                    dx_b      = tailways[other_lane] if other_lane < len(tailways) else 0.0
                features.extend([
                    is_rl_lane,
                    dv_f / max_speed,       # 2: 1车道前车速度差
                    dx_f / ring_len,        # 3: 1车道前车距离
                    dv_b / max_speed,       # 4: 1车道后车速度差
                    dx_b / ring_len,        # 5: 1车道后车距离
                        ])
            # 记录可观测车辆 ID
            self.observe[rl_id] = np.array([lead_id, follow_id, ahead_id, back_id])
            obs = features

        return obs


    def additional_command(self):
        """Extra operations executed every simulation step: 
        1) mark visible vehicles
        2) log all vehicle states into CSV (once per step)
        """
        import csv, os
        kv = self.k.vehicle

        # ============================================================
        # 1) Flow 原来的可视化 observe 逻辑
        # ============================================================
        for rl_id in self.k.vehicle.get_rl_ids():
            if rl_id not in self.observe:
                continue
            for veh_id in self.observe[rl_id]:
                self.k.vehicle.set_observed(veh_id)

        # ============================================================
        # 2) 每步自动记录 CSV 日志
        # ============================================================

        # 创建目录
        save_dir = "/home/spei/flow_logs"
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "trajectory_log_allhuman44.csv")

        # 当前 step
        step = getattr(self, "step_counter", 0)

        # 避免同一步多次写入
        flag_attr = "_csv_logged_step"
        last_logged = getattr(self, flag_attr, -1)
        if last_logged == step:
            return
        setattr(self, flag_attr, step)

        # ==== 定义绝对坐标计算 ====
        def get_global_position(pid):
            lane_pos = kv.get_position(pid)
            if lane_pos is None:
                return 0.0

            edge = kv.get_edge(pid)
            ring_length = 250
            junc = 0.1

            if edge == "bottom":
                offset = 0.0
            elif edge == "right":
                offset = 0.25 * ring_length + junc
            elif edge == "top":
                offset = 0.5 * ring_length + 2 * junc
            elif edge == "left":
                offset = 0.75 * ring_length + 3 * junc
            else:
                return lane_pos

            return offset + lane_pos

        # === 写入 CSV ===
        first_write = not os.path.exists(save_path)
        ids = kv.get_ids()

        with open(save_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "id", "type", "step", "lane_id", "lane_position_global", "speed"
            ])
            if first_write:
                writer.writeheader()

            for pid in ids:
                vtype = "rl" if "rl" in pid else "human"
                lane_id = kv.get_lane(pid) or 0
                speed = kv.get_speed(pid) or 0.0
                abs_pos = get_global_position(pid)

                writer.writerow({
                    "id": pid,
                    "type": vtype,
                    "step": step,
                    "lane_id": lane_id,
                    "lane_position_global": abs_pos,
                    "speed": speed,
                })

