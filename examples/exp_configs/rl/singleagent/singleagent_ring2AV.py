"""Ring road example.

Trains a single autonomous vehicle to stabilize the flow of 21 human-driven
vehicles in a variable length ring road.
"""
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams
from flow.core.params import VehicleParams, SumoCarFollowingParams, SumoLaneChangeParams
from flow.controllers import SimCarFollowingController, PairAlignRuleController, RLController,SimLaneChangeController, IDMController, ContinuousRouter
from flow.envs import WaveAttenuationPOEnvCentral
from flow.networks import RingNetwork
import random


# time horizon of a single rollout
HORIZON = 5000
# number of rollouts per training iteration
N_ROLLOUTS = 20
# number of parallel workers
N_CPUS = 5
NUM_AUTOMATED = 2
exp_name ="Centralized2AV"
# We place one autonomous vehicle and 22 human-driven vehicles in the network
vehicles = VehicleParams()
Total_Number_Veh = 44


# We evenly distribute the automated vehicles in the network.
num_human = Total_Number_Veh - NUM_AUTOMATED
human1=random.randrange(0, 20, 2)
humans_remaining = num_human-human1

vehicles = VehicleParams()
vehicles.add(
    veh_id="rl0",
    acceleration_controller=(RLController, {}),
    # acceleration_controller=(PairAlignRuleController, {
    #     "pair_id": "rl1",          # 伙伴是 rl1
    #     "ring_length": 250,
    #     "k_sync": 0.8,
    #     "k_pair": 0.08,
    #     "k_front": 0.3,
    #     "k_back": 0.15,
    #     "k_v": 0.0,
    #     "v_star": 12.0,
    #     "safe_gap": 7.0,
    #     "hard_brake": -2.5,
    #     "max_accel": 1.0,
    #     "max_decel": 1.0,
    # }),
    routing_controller=(ContinuousRouter, {}),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(
        model="LC2013",
        lane_change_mode=512,   # 完全 RL 控制（如果你实现了 RL lateral）
        ),
    num_vehicles=1)
vehicles.add(
    veh_id="human0",
    acceleration_controller=(SimCarFollowingController, {}),
    car_following_params=SumoCarFollowingParams(
        min_gap=2,
        sigma=0.2,
        accel=1.0,          # 对齐：accel = 1
        decel=1.5,          # 对齐：decel = 1.5
    ),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(
        model="LC2013",
        lane_change_mode=1621
    ),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=human1)
vehicles.add(
    veh_id="rl1",
    acceleration_controller=(RLController, {}),
    # acceleration_controller=(PairAlignRuleController, {
    #     "pair_id": "rl0",          # 伙伴是 rl0
    #     "ring_length": 250,
    #     "k_sync": 0.8,
    #     "k_pair": 0.08,
    #     "k_front": 0.3,
    #     "k_back": 0.15,
    #     "k_v": 0.0,
    #     "v_star": 12.0,
    #     "safe_gap": 7.0,
    #     "hard_brake": -2.5,
    #     "max_accel": 1.0,
    #     "max_decel": 1.0,
    # }),
    routing_controller=(ContinuousRouter, {}),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(
        model="LC2013",
        lane_change_mode=512,   # 完全 RL 控制（如果你实现了 RL lateral）
        ),
    num_vehicles=NUM_AUTOMATED-1)
vehicles.add(
    veh_id="human1",
    acceleration_controller=(SimCarFollowingController, {}),
    car_following_params=SumoCarFollowingParams(
        min_gap=2,
        sigma=0.2,
        accel=1.0,          # 对齐：accel = 1
        decel=1.5,          # 对齐：decel = 1.5
    ),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(
        model="LC2013",
        lane_change_mode=1621
    ),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=humans_remaining)

flow_params = dict(
    # name of the experiment
    exp_tag=exp_name,

    # name of the flow environment the experiment is running on
    env_name=WaveAttenuationPOEnvCentral,

    # name of the network class the experiment is running on
    network=RingNetwork,

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        sim_step=0.1,
        render=False,
        restart_instance=False
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        horizon=HORIZON,
        warmup_steps=2000,
        clip_actions=False,
        additional_params={
            "max_accel": 0.5,
            "max_decel": 0.5,
            "ring_length": 250,
        },
    ),

    # network-related parameters (see flow.core.params.NetParams and the
    # network's documentation or ADDITIONAL_NET_PARAMS component)
    net=NetParams(
        additional_params={
            "length": 250,
            "lanes": 2,
            "speed_limit": 30,
            "resolution": 40,
        }, ),

    # vehicles to be placed in the network at the start of a rollout (see
    # flow.core.params.VehicleParams)
    veh=vehicles,

    # parameters specifying the positioning of vehicles upon initialization/
    # reset (see flow.core.params.InitialConfig)
    initial=InitialConfig(
        spacing="uniform",
        perturbation=0,
        shuffle=True,
        )
)
