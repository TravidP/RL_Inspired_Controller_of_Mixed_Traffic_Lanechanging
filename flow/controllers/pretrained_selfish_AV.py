import torch
import numpy as np
from types import SimpleNamespace
from flow.controllers.base_controller import BaseController
from flow.controllers.base_lane_changing_controller import BaseLaneChangeController
from flow.controllers.SimpleFFN import SimpleFFN
from flow.controllers.u import *  # 你已有
from flow.controllers.ut import *  # 假设你把 FFN 单独写了个 py 文件
#from flow.controllers.utils import setdefaults


# Define NamedArrays and distribution classes


class PreTrainedRLController(BaseController):
    def __init__(self, veh_id, car_following_params,  **kwargs):
        action_space = dict(
        accel=Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32),
        lc=Discrete(2)
        )
        dist_class = build_dist(action_space)
        config = Namespace(
            observation_space=Box(low=-1, high=1, shape=(11,), dtype=np.float32),
            model_output_size=dist_class.model_output_size,
            dist_class=dist_class,
            use_critic=False,
            layers=[64, 'tanh', 64, 'tanh'],
            weight_init='orthogonal',
            weight_scale='default'
        )
        self.min_gap = kwargs.pop("min_gap", 0.0)
        model_path = kwargs.pop("model_path", None)
        assert model_path is not None, "You must provide model_path in controller config"
        # Create configuration object (mimicking training config)
        self.model = FFN(config)
        ckpt = torch.load(model_path, map_location="cpu")
        self.model.load_state_dict(ckpt['net'])  # 正确加载权重self.model.eval()
        self.max_speed=10
        self.ring_length=250
        self.max_accel=0.5
        self.max_decel=0.5

        # self.max_speed = kwargs.get("max_speed", 10.0)
        # self.ring_length = kwargs.get("ring_length", 250)
        
        
        super().__init__(veh_id, car_following_params, **kwargs)

    
    def get_accel(self, env):
        obs = self.get_observation(env)
        with torch.no_grad():
            x = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
            out = self.model(x, policy=True, argmax=False)
            accel = out.action.accel.item()
            accel = (accel - (-1)) / (1 - (-1))
            accel = (accel * 2 - 1) * (self.max_accel if accel > 0.5 else self.max_decel)
            print (accel)
            return float(accel)

    def get_action(self,env):
        accel=self.get_accel(env)
        return accel

    def get_observation(self, env):
        v = env.k.vehicle.get_speed(self.veh_id)
        lane = int(env.k.vehicle.get_lane(self.veh_id))
        # try:
        print(self.veh_id)
        leaders = env.k.vehicle.get_lane_leaders(self.veh_id)
        # print(env.k.vehicle.__vehicles[self.veh_id])
        followers = env.k.vehicle.get_lane_followers(self.veh_id)
        headways = env.k.vehicle.get_lane_headways(self.veh_id)
        tailways = env.k.vehicle.get_lane_tailways(self.veh_id)
        # except KeyError:
        #     # 返回一个默认状态，全 0 向量（或上一次状态）
        #     return np.zeros(11, dtype=np.float32)
        obs=[v/self.max_speed]
        # n_lanes = env.k.network.num_lanes(self.veh_id)
        n_lanes=2
        for l in range(n_lanes):
            is_rl_lane = (l == lane)

            if is_rl_lane:
                lead_id = leaders[lane] or self.veh_id
                
                dx_f = headways[lane] if lane < len(headways) else 0.0
                dv_f = env.k.vehicle.get_speed(lead_id)
                
                follow_id = followers[lane] or self.veh_id
                dx_b = tailways[lane] if lane < len(tailways) else 0.0
                dv_b = env.k.vehicle.get_speed(follow_id)

            else:
                other_lane = (lane + 1) % n_lanes

                # 前车
                ahead_id   = leaders[other_lane] or self.veh_id
                dv_f      = env.k.vehicle.get_speed(ahead_id) 
                dx_f      = headways[other_lane] if other_lane < len(headways) else 0.0

                # 后车
                back_id    = followers[other_lane] or self.veh_id
                dv_b      = env.k.vehicle.get_speed(back_id)
                dx_b      = tailways[other_lane] if other_lane < len(tailways) else 0.0
            obs.extend([
                is_rl_lane,
                dv_f / self.max_speed,
                dx_f / self.ring_length,
                dv_b / self.max_speed,
                dx_b / self.ring_length,
            ])
        obs = np.array(obs)
        obs = np.clip(obs, 0, 1) * (1 - (-1)) -1
        print(obs)
        return obs.astype(np.float32)



class PreTrainedRLControllerLC(BaseLaneChangeController):
    def __init__(self, veh_id, lane_change_params,  **kwargs):
        action_space = dict(
        accel=Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32),
        lc=Discrete(2)
        )
        dist_class = build_dist(action_space)
        config = Namespace(
            observation_space=Box(low=-1, high=1, shape=(11,), dtype=np.float32),
            model_output_size=dist_class.model_output_size,
            dist_class=dist_class,
            use_critic=False,
            layers=[64, 'tanh', 64, 'tanh'],
            weight_init='orthogonal',
            weight_scale='default'
        )
        model_path = kwargs.pop("model_path", None)
        assert model_path is not None, "You must provide model_path in controller config"
        # Create configuration object (mimicking training config)
        self.model = FFN(config)
        ckpt = torch.load(model_path, map_location="cpu")
        self.model.load_state_dict(ckpt['net'])  # 正确加载权重
        self.model.eval()
        self.max_speed=10
        self.ring_length=250
        self.max_accel=0.5
        self.max_decel=0.5
        super().__init__(veh_id, lane_change_params, **kwargs)
 

    def get_lane_change_action(self, env):
        obs = self.get_observation(env)
        with torch.no_grad():
            x = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
            out = self.model(x, policy=True, argmax=True)
            lc = out.action.lc.item()
            lc = bool(np.round((lc - (-1)) / (1 - (-1))))
          
            # return lc
            current_lane = env.k.vehicle.get_lane(self.veh_id)
            print(current_lane)
            if lc != current_lane:
                if current_lane :
                    direction=-1
                else:
                    direction=1
            else: 
                direction=0
            print(direction)
            return direction

    def get_observation(self, env):
        v = env.k.vehicle.get_speed(self.veh_id)
        lane = int(env.k.vehicle.get_lane(self.veh_id))
        # try:
        leaders = env.k.vehicle.get_lane_leaders(self.veh_id)
        followers = env.k.vehicle.get_lane_followers(self.veh_id)
        headways = env.k.vehicle.get_lane_headways(self.veh_id)
        tailways = env.k.vehicle.get_lane_tailways(self.veh_id)
        # except KeyError:
        #     # 返回一个默认状态，全 0 向量（或上一次状态）
        #     return np.zeros(11, dtype=np.float32)
        obs=[v/self.max_speed]
        # n_lanes = env.k.network.num_lanes(self.veh_id)
        n_lanes=2
        for l in range(n_lanes):
            is_rl_lane = (l == lane)

            if is_rl_lane:
                lead_id = leaders[lane] or self.veh_id
                
                dx_f = headways[lane] if lane < len(headways) else 0.0
                dv_f = env.k.vehicle.get_speed(lead_id)
                
                follow_id = followers[lane] or self.veh_id
                dx_b = tailways[lane] if lane < len(tailways) else 0.0
                dv_b = env.k.vehicle.get_speed(follow_id)

            else:
                other_lane = (lane + 1) % n_lanes

                # 前车
                ahead_id   = leaders[other_lane] or self.veh_id
                dv_f      = env.k.vehicle.get_speed(ahead_id) 
                dx_f      = headways[other_lane] if other_lane < len(headways) else 0.0

                # 后车
                back_id    = followers[other_lane] or self.veh_id
                dv_b      = env.k.vehicle.get_speed(back_id)
                dx_b      = tailways[other_lane] if other_lane < len(tailways) else 0.0
            obs.extend([
                is_rl_lane,
                dv_f / self.max_speed,
                dx_f / self.ring_length,
                dv_b / self.max_speed,
                dx_b / self.ring_length,
            ])
        obs  = np.array(obs)
        obs = np.clip(obs, 0, 1) * (1 - (-1)) -1
        return obs.astype(np.float32)
