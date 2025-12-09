from flow.controllers import IDMController, ContinuousRouter, SimLaneChangeController
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams
from flow.core.params import VehicleParams, SumoCarFollowingParams, SumoLaneChangeParams
from flow.envs import LaneChangeAccelPOEnv
from flow.networks import RingNetwork
from flow.core.experiment import Experiment

# 仿真参数
NUM_TOTAL_VEHICLES = 44
HORIZON = 10000  # 可缩短测试速度
RENDER = True

# 车辆定义
vehicles = VehicleParams()
vehicles.add(
    veh_id="idm",
    acceleration_controller=(IDMController, {"noise": 0.2}),
    car_following_params=SumoCarFollowingParams(min_gap=0),
    routing_controller=(ContinuousRouter, {}),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(model="LC2013", lane_change_mode=1621),
    num_vehicles=NUM_TOTAL_VEHICLES
)

# flow_params 定义
flow_params = dict(
    exp_tag="run_all_human_idm_ring",
    env_name=LaneChangeAccelPOEnv,
    network=RingNetwork,
    simulator='traci',
    sim=SumoParams(sim_step=0.1, render=RENDER),
    env=EnvParams(
        horizon=HORIZON,
        warmup_steps=0,
        clip_actions=False,
        additional_params={
            "max_accel": 3.5,
            "max_decel": -3.5,
            "lane_change_duration": 1,
            "target_velocity": 30,
            "sort_vehicles": False,
            "ring_length": 250,
        },
    ),
    net=NetParams(
        additional_params={
            "length": 250,
            "lanes": 2,
            "speed_limit": 30,
            "resolution": 40,
        },
    ),
    veh=vehicles,
    initial=InitialConfig(),
)

# 运行仿真
if __name__ == "__main__":
    exp = Experiment(flow_params)
    exp.run(1, convert_to_csv=False)
