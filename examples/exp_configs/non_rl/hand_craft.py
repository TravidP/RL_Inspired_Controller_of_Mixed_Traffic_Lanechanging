"""
Example: Two AVs on a two-lane ring performing synchronized motion.

- Automatically computes the steady-state equilibrium velocity v_eq_max
  based on ring length and vehicle density.
- Updates both the environment's target_velocity and each AV controller's v_star.
"""

from flow.controllers import SimCarFollowingController,HandcraftSignController,IDMController, ContinuousRouter, SimLaneChangeController
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams, VehicleParams, SumoCarFollowingParams,SumoLaneChangeParams
from flow.networks.ring import RingNetwork
from flow.envs import WaveAttenuationPOEnvCentral
from scipy.optimize import fsolve
import numpy as np
import random
import os
SEED = 22
os.environ["PYTHONHASHSEED"] = str(SEED)
np.random.seed(SEED)
random.seed(SEED)
# ================================================================
# ⚙️ Basic experiment setup
# ================================================================
TOTAL_VEH = 44
NUM_AUTOMATED = 2
NUM_HUMAN = TOTAL_VEH - NUM_AUTOMATED
RING_LENGTH = 250
LANES = 2
HORIZON = 5000
DriverModel=10

# ================================================================
# 🚗 Vehicle configuration
# ================================================================
def add_random_vehicles_with_total(
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
        noise   = random.uniform(0.2, 1.0)
        min_gap = random.uniform(0, 2)
        accel   = random.uniform(1.0, 3.0)
        decel   = random.uniform(2.0, 5.0)
        lc_assertive   = random.uniform(0.1, 2.5)
        lc_cooperative = random.uniform(0.1, 2.5)
        lc_speed_gain  = random.uniform(0.1, 2.0)
        lc_keep_right  = random.choice([0.0, 1.0])
        min_gap_lat    = random.uniform(0.1, 0.5)
        max_speed   = random.choice([30.0])

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
            acceleration_controller=(SimCarFollowingController, {
                # "noise": noise
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


vehicles = VehicleParams()

vehicles.add(
    veh_id="rl0",
    acceleration_controller=(HandcraftSignController, {
            "handcraft": 4.36,      # 目标速度
            "max_accel": 0.5,
            "max_decel": 0.5,
        }),
    routing_controller=(ContinuousRouter, {}),
    lane_change_controller=(SimLaneChangeController, {}),
    num_vehicles=1,
    color="255, 0, 0",
)
# vehicles.add(
#     veh_id="human0",
#     acceleration_controller=(SimCarFollowingController, {}),
#     car_following_params=SumoCarFollowingParams(
#         min_gap=2,
#         sigma=0.2,
#         accel=1.0,          # 对齐：accel = 1
#         decel=1.5,          # 对齐：decel = 1.5
#     ),
#     lane_change_controller=(SimLaneChangeController, {}),
#     lane_change_params=SumoLaneChangeParams(
#         model="LC2013",
#         lane_change_mode=1621
#     ),
#     routing_controller=(ContinuousRouter, {}),
#     num_vehicles=NUM_HUMAN)

add_random_vehicles_with_total(
            vehicles,
            prefix="human",
            indices=range(10),
            total_vehicles=NUM_HUMAN//2
        )

vehicles.add(
    veh_id="rl1",
    acceleration_controller=(HandcraftSignController, {
            "handcraft": 4.36,      # 目标速度
            "max_accel": 0.5,
            "max_decel": 0.5,
        }),
    routing_controller=(ContinuousRouter, {}),
    lane_change_controller=(SimLaneChangeController, {}),
    num_vehicles=1,
    color="255, 0, 0",)

add_random_vehicles_with_total(
            vehicles,
            prefix="human1",
            indices=range(10),
            total_vehicles=NUM_HUMAN-NUM_HUMAN // 2
        )

# ================================================================
# 🧩 Flow experiment parameters
# ================================================================
flow_params = dict(
    exp_tag='ring_handcraft_diff',
    env_name=WaveAttenuationPOEnvCentral,
    network=RingNetwork,
    simulator='traci',

    # SUMO parameters
    sim=SumoParams(
        render=True,
        sim_step=0.1,
        restart_instance=False,
    ),

    # Environment parameters
    env=EnvParams(
        horizon=HORIZON,
        warmup_steps=5000,
        clip_actions=False,
        additional_params={
            # "target_velocity": float(v_eq_max),  # 🔹同步环上稳态速度
            "max_accel": 0.5,
            "max_decel": 0.5,
            "ring_length": RING_LENGTH,
        },
    ),

    # Network parameters
    net=NetParams(
        additional_params={
            "length": RING_LENGTH,
            "lanes": LANES,
            "speed_limit": 30,
            "resolution": 40,
        },
    ),

    veh=vehicles,

    # Initialization parameters
    initial=InitialConfig(
        spacing="uniform",
        shuffle=True,
    ),
)

# print(f"✅ target_velocity in environment = {flow_params['env'].additional_params['target_velocity']:.2f} m/s")
