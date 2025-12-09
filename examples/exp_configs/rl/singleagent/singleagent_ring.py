"""Ring road example.

Trains a single autonomous vehicle to stabilize the flow of 21 human-driven
vehicles in a variable length ring road.
"""
from flow.core.params import SumoLaneChangeParams, SumoParams, EnvParams, InitialConfig, NetParams
from flow.core.params import VehicleParams, SumoCarFollowingParams
from flow.controllers import SimLaneChangeController,RLController, IDMController, ContinuousRouter,SimCarFollowingController
from flow.envs import WaveAttenuationPOEnv
from flow.networks import RingNetwork

# time horizon of a single rollout
HORIZON = 5000
# number of rollouts per training iteration
N_ROLLOUTS = 20
# number of parallel workers
N_CPUS = 5

# We place one autonomous vehicle and 22 human-driven vehicles in the network
vehicles = VehicleParams()
vehicles.add(
    veh_id="human",
    # acceleration_controller=(IDMController, {"noise": 0}),
    acceleration_controller=(SimCarFollowingController,{}),
    car_following_params=SumoCarFollowingParams(
        min_gap=2,
        sigma=0.2,
        accel=1.0,          # 对齐：accel = 1
        decel=1.5,          # 对齐：decel = 1.5
    ),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(
        model="LC2013",
        lcStrategic=1.0,
        lcCooperative=1.0,
        lcSpeedGain=1.0,
        lcKeepRight=1.0,
        lane_change_mode=1621
        # lane_change_mode=512
    ),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=43)
vehicles.add(
    veh_id="rl",
    acceleration_controller=(RLController, {}),
    car_following_params=SumoCarFollowingParams(
        accel=1.0,
        decel=1.5,
        tau=1.0,
        min_gap=0,
        max_speed=10.0,
        speed_factor=1.0,
        speed_dev=0.1,
        sigma=0.0,          # 对齐：sigma = 0
    ),
    lane_change_controller=(SimLaneChangeController, {}),
    lane_change_params=SumoLaneChangeParams(
        model="LC2013",
        lane_change_mode=512
    ),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=1)

flow_params = dict(
    # name of the experiment
    exp_tag="stabilizing_the_ring",

    # name of the flow environment the experiment is running on
    env_name=WaveAttenuationPOEnv,

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
            "lane_change_duration": 0,
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
    initial=InitialConfig(),
)
