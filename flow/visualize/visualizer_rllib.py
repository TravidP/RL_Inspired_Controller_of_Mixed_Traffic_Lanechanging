"""Visualizer for rllib experiments.

Attributes
----------
EXAMPLE_USAGE : str
    Example call to the function, which is
    ::

        python ./visualizer_rllib.py /tmp/ray/result_dir 1

parser : ArgumentParser
    Command-line argument parser
"""

import argparse
import gym
import numpy as np
import os
import sys
import time

import ray
try:
    from ray.rllib.agents.agent import get_agent_class
except ImportError:
    from ray.rllib.agents.registry import get_agent_class
from ray.tune.registry import register_env

from flow.core.util import emission_to_csv
from flow.utils.registry import make_create_env
from flow.utils.rllib import get_flow_params
from flow.utils.rllib import get_rllib_config
from flow.utils.rllib import get_rllib_pkl


EXAMPLE_USAGE = """
example usage:
    python ./visualizer_rllib.py /ray_results/experiment_dir/result_dir 1

Here the arguments are:
1 - the path to the simulation results
2 - the number of the checkpoint
"""
import csv
import os
def log_vehicle_states(env):
    """Log all vehicle states at current step inside visualizer (absolute 0~250m)."""
    import csv, os
    kv = env.unwrapped.k.vehicle

    save_dir = "/home/spei/flow_logs"
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, "trajectory_log_2AVCentral.csv")
    # save_path = os.path.join(save_dir, "trajectory_log_1AVLC.csv")

    # 当前仿真 step
    step = getattr(env, "step_counter", 0)

    # 防止同一步重复写入
    flag = "_visualizer_log_step"
    if getattr(env, flag, -1) == step:
        return
    setattr(env, flag, step)

    # 是否第一次写
    first_write = not os.path.exists(save_path)
    ids = kv.get_ids()

    # 内嵌：根据 Flow ring（bottom→right→top→left）计算绝对 0~250 m 位置
    def get_global_position(pid):
        """Return absolute 0~ring_length position."""
        pos_local = kv.get_position(pid)
        if pos_local is None:
            return 0.0

        edge = kv.get_edge(pid)
        ring_length = env.unwrapped.network.net_params.additional_params["length"]
        junction_length = 0.1  # 与 ring 网络定义一致

        # 四段环路绝对起点
        if edge == "bottom":
            edge_start = 0.0
        elif edge == "right":
            edge_start = 0.25 * ring_length + junction_length
        elif edge == "top":
            edge_start = 0.5 * ring_length + 2 * junction_length
        elif edge == "left":
            edge_start = 0.75 * ring_length + 3 * junction_length
        else:
            # 内部段例如 :top_0，SUMO 已经给的是绝对坐标
            return pos_local

        return edge_start + pos_local  # 不取 mod，保持连续

    with open(save_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "id", "type", "step", "lane", "pos_global", "speed"
        ])
        if first_write:
            writer.writeheader()

        for pid in ids:
            vtype = "rl" if "rl" in pid else "human"
            lane_raw = kv.get_lane(pid)
            lane_idx = lane_raw[0] if isinstance(lane_raw, (list, tuple)) else lane_raw

            pos_global = get_global_position(pid)

            writer.writerow({
                "id": pid,
                "type": vtype,
                "step": step,
                "lane": lane_idx,
                "pos_global": pos_global,
                "speed": kv.get_speed(pid) or 0.0,
            })



def visualizer_rllib(args):
    """Visualizer for RLlib experiments.

    This function takes args (see function create_parser below for
    more detailed information on what information can be fed to this
    visualizer), and renders the experiment associated with it.
    """
    result_dir = args.result_dir if args.result_dir[-1] != '/' \
        else args.result_dir[:-1]

    config = get_rllib_config(result_dir)

    # check if we have a multiagent environment but in a
    # backwards compatible way
    if config.get('multiagent', {}).get('policies', None):
        multiagent = True
        pkl = get_rllib_pkl(result_dir)
        config['multiagent'] = pkl['multiagent']
    else:
        multiagent = False

    # Run on only one cpu for rendering purposes
    config['num_workers'] = 0

    flow_params = get_flow_params(config)
    flow_params['env'].additional_params['ring_length'] = 250
    # hack for old pkl files
    # TODO(ev) remove eventually
    sim_params = flow_params['sim']
    setattr(sim_params, 'num_clients', 1)

    # for hacks for old pkl files TODO: remove eventually
    if not hasattr(sim_params, 'use_ballistic'):
        sim_params.use_ballistic = False

    # Determine agent and checkpoint
    config_run = config['env_config']['run'] if 'run' in config['env_config'] \
        else None
    if args.run and config_run:
        if args.run != config_run:
            print('visualizer_rllib.py: error: run argument '
                  + '\'{}\' passed in '.format(args.run)
                  + 'differs from the one stored in params.json '
                  + '\'{}\''.format(config_run))
            sys.exit(1)
    if args.run:
        agent_cls = get_agent_class(args.run)
    elif config_run:
        agent_cls = get_agent_class(config_run)
    else:
        print('visualizer_rllib.py: error: could not find flow parameter '
              '\'run\' in params.json, '
              'add argument --run to provide the algorithm or model used '
              'to train the results\n e.g. '
              'python ./visualizer_rllib.py /tmp/ray/result_dir 1 --run PPO')
        sys.exit(1)

    sim_params.restart_instance = True
    dir_path = os.path.dirname(os.path.realpath(__file__))
    emission_path = '{0}/test_time_rollout/'.format(dir_path)
    sim_params.emission_path = emission_path if args.gen_emission else None

    # pick your rendering mode
    if args.render_mode == 'sumo_web3d':
        sim_params.num_clients = 2
        sim_params.render = False
    elif args.render_mode == 'drgb':
        sim_params.render = 'drgb'
        sim_params.pxpm = 4
    elif args.render_mode == 'sumo_gui':
        sim_params.render = False  # will be set to True below
    elif args.render_mode == 'no_render':
        sim_params.render = False
    if args.save_render:
        if args.render_mode != 'sumo_gui':
            sim_params.render = 'drgb'
            sim_params.pxpm = 4
        sim_params.save_render = True

    # Create and register a gym+rllib env
    create_env, env_name = make_create_env(params=flow_params, version=0)
    register_env(env_name, create_env)

    # check if the environment is a single or multiagent environment, and
    # get the right address accordingly
    # single_agent_envs = [env for env in dir(flow.envs)
    #                      if not env.startswith('__')]

    # if flow_params['env_name'] in single_agent_envs:
    #     env_loc = 'flow.envs'
    # else:
    #     env_loc = 'flow.envs.multiagent'

    # Start the environment with the gui turned on and a path for the
    # emission file
    env_params = flow_params['env']
    env_params.restart_instance = False
    if args.evaluate:
        env_params.evaluate = True

    # lower the horizon if testing
    if args.horizon:
        config['horizon'] = args.horizon
        env_params.horizon = args.horizon

    # create the agent that will be used to compute the actions
    agent = agent_cls(env=env_name, config=config)
    checkpoint = result_dir + '/checkpoint_' + args.checkpoint_num
    checkpoint = checkpoint + '/checkpoint-' + args.checkpoint_num
    agent.restore(checkpoint)

    if hasattr(agent, "local_evaluator") and \
            os.environ.get("TEST_FLAG") != 'True':
        env = agent.local_evaluator.env
    else:
        env = gym.make(env_name)

    if args.render_mode == 'sumo_gui':
        env.sim_params.render = True  # set to True after initializing agent and env

    if multiagent:
        rets = {}
        # map the agent id to its policy
        policy_map_fn = config['multiagent']['policy_mapping_fn']
        for key in config['multiagent']['policies'].keys():
            rets[key] = []
    else:
        rets = []

    if config['model']['use_lstm']:
        use_lstm = True
        if multiagent:
            state_init = {}
            # map the agent id to its policy
            policy_map_fn = config['multiagent']['policy_mapping_fn']
            size = config['model']['lstm_cell_size']
            for key in config['multiagent']['policies'].keys():
                state_init[key] = [np.zeros(size, np.float32),
                                   np.zeros(size, np.float32)]
        else:
            state_init = [
                np.zeros(config['model']['lstm_cell_size'], np.float32),
                np.zeros(config['model']['lstm_cell_size'], np.float32)
            ]
    else:
        use_lstm = False

    # if restart_instance, don't restart here because env.reset will restart later
    if not sim_params.restart_instance:
        env.restart_simulation(sim_params=sim_params, render=sim_params.render)

    # Simulate and collect metrics
    final_outflows = []
    final_inflows = []
    mean_speed = []
    std_speed = []
    for i in range(args.num_rollouts):
        vel = []
        state = env.reset()
        if multiagent:
            ret = {key: [0] for key in rets.keys()}
        else:
            ret = 0
        if use_lstm:
            prev_action = np.zeros(env.action_space.shape, dtype=np.float32)
            prev_reward = 0.0
        for _ in range(env_params.horizon):
            vehicles = env.unwrapped.k.vehicle
            speeds = vehicles.get_speed(vehicles.get_ids())

            # only include non-empty speeds
            if speeds:
                vel.append(np.mean(speeds))

            if multiagent:
                action = {}
                for agent_id in state.keys():
                    if use_lstm:
                        action[agent_id], state_init[agent_id], logits = \
                            agent.compute_action(
                            state[agent_id], state=state_init[agent_id],
                            policy_id=policy_map_fn(agent_id))
                    else:
                        action[agent_id], _, info = agent.compute_action(
                            state[agent_id], policy_id=policy_map_fn(agent_id), full_fetch=True)
                        if "behaviour_logits" in info:
                            action_dim = env.action_space.shape[0]
                            mu = info["behaviour_logits"][:action_dim]
                            a = np.array(mu, dtype=np.float32)
                        action[agent_id] = a

            else:
                if use_lstm:
                    action, state_init, _ = agent.compute_action(
                        state,
                        state=state_init,
                        prev_action=prev_action,
                        prev_reward=prev_reward,
                        full_fetch=True
                    )
                else:
                    action, _, info = agent.compute_action(state,full_fetch=True)
                    # print("action1:", action)
                    if "behaviour_logits" in info:
                        # 离散动作：取最大概率动作
                        logits = info["behaviour_logits"]
                        action_dim = env.action_space.shape[0]   # 对于单车 RL 是 2 (accel, lane_change)
                        action = logits[:action_dim]
                        # 转成 numpy float32，避免环境报错
                        action = np.array(action, dtype=np.float32)
                        # print("action2:", action)
            state, reward, done, _ = env.step(action)
            log_vehicle_states(env)
            if use_lstm:
                prev_action = action
                prev_reward = reward
            if multiagent:
                for actor, rew in reward.items():
                    ret[policy_map_fn(actor)][0] += rew
            else:
                ret += reward
            if multiagent and done['__all__']:
                break
            if not multiagent and done:
                break

        if multiagent:
            for key in rets.keys():
                rets[key].append(ret[key])
        else:
            rets.append(ret)
        outflow = vehicles.get_outflow_rate(500)
        final_outflows.append(outflow)
        inflow = vehicles.get_inflow_rate(500)
        final_inflows.append(inflow)
        if np.all(np.array(final_inflows) > 1e-5):
            throughput_efficiency = [x / y for x, y in
                                     zip(final_outflows, final_inflows)]
        else:
            throughput_efficiency = [0] * len(final_inflows)
        mean_speed.append(np.mean(vel))
        std_speed.append(np.std(vel))
        if multiagent:
            for agent_id, rew in rets.items():
                print('Round {}, Return: {} for agent {}'.format(
                    i, ret, agent_id))
        else:
            print('Round {}, Return: {}'.format(i, ret))

    print('==== Summary of results ====')
    print("Return:")
    print(mean_speed)
    if multiagent:
        for agent_id, rew in rets.items():
            print('For agent', agent_id)
            print(rew)
            print('Average, std return: {}, {} for agent {}'.format(
                np.mean(rew), np.std(rew), agent_id))
    else:
        print(rets)
        print('Average, std: {}, {}'.format(
            np.mean(rets), np.std(rets)))

    print("\nSpeed, mean (m/s):")
    print(mean_speed)
    print('Average, std: {}, {}'.format(np.mean(mean_speed), np.std(
        mean_speed)))
    print("\nSpeed, std (m/s):")
    print(std_speed)
    print('Average, std: {}, {}'.format(np.mean(std_speed), np.std(
        std_speed)))

    # Compute arrival rate of vehicles in the last 500 sec of the run
    print("\nOutflows (veh/hr):")
    print(final_outflows)
    print('Average, std: {}, {}'.format(np.mean(final_outflows),
                                        np.std(final_outflows)))
    # Compute departure rate of vehicles in the last 500 sec of the run
    print("Inflows (veh/hr):")
    print(final_inflows)
    print('Average, std: {}, {}'.format(np.mean(final_inflows),
                                        np.std(final_inflows)))
    # Compute throughput efficiency in the last 500 sec of the
    print("Throughput efficiency (veh/hr):")
    print(throughput_efficiency)
    print('Average, std: {}, {}'.format(np.mean(throughput_efficiency),
                                        np.std(throughput_efficiency)))

    # terminate the environment
    env.unwrapped.terminate()

    # if prompted, convert the emission file into a csv file
    if args.gen_emission:
        time.sleep(0.1)

        dir_path = os.path.dirname(os.path.realpath(__file__))
        emission_filename = '{0}-emission.xml'.format(env.network.name)

        emission_path = \
            '{0}/test_time_rollout/{1}'.format(dir_path, emission_filename)

        # convert the emission file into a csv file
        emission_to_csv(emission_path)

        # print the location of the emission csv file
        emission_path_csv = emission_path[:-4] + ".csv"
        print("\nGenerated emission file at " + emission_path_csv)

        # delete the .xml version of the emission file
        os.remove(emission_path)


def create_parser():
    """Create the parser to capture CLI arguments."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='[Flow] Evaluates a reinforcement learning agent '
                    'given a checkpoint.',
        epilog=EXAMPLE_USAGE)

    # required input parameters
    parser.add_argument(
        'result_dir', type=str, help='Directory containing results')
    parser.add_argument('checkpoint_num', type=str, help='Checkpoint number.')

    # optional input parameters
    parser.add_argument(
        '--run',
        type=str,
        help='The algorithm or model to train. This may refer to '
             'the name of a built-on algorithm (e.g. RLLib\'s DQN '
             'or PPO), or a user-defined trainable function or '
             'class registered in the tune registry. '
             'Required for results trained with flow-0.2.0 and before.')
    parser.add_argument(
        '--num_rollouts',
        type=int,
        default=1,
        help='The number of rollouts to visualize.')
    parser.add_argument(
        '--gen_emission',
        action='store_true',
        help='Specifies whether to generate an emission file from the '
             'simulation')
    parser.add_argument(
        '--evaluate',
        action='store_true',
        help='Specifies whether to use the \'evaluate\' reward '
             'for the environment.')
    parser.add_argument(
        '--render_mode',
        type=str,
        default='sumo_gui',
        help='Pick the render mode. Options include sumo_web3d, '
             'rgbd and sumo_gui')
    parser.add_argument(
        '--save_render',
        action='store_true',
        help='Saves a rendered video to a file. NOTE: Overrides render_mode '
             'with pyglet rendering.')
    parser.add_argument(
        '--horizon',
        type=int,
        help='Specifies the horizon.')
    return parser


if __name__ == '__main__':
    parser = create_parser()
    args = parser.parse_args()
    ray.init(num_cpus=1)
    visualizer_rllib(args)
