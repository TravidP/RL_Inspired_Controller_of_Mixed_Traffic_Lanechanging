# Experiment Examples

This folder contains the runnable experiment entry points for the mixed-autonomy lane-changing study.

Run commands from this directory:

```bash
cd examples
```

The scripts assume that Flow, SUMO, TraCI, and the project dependencies are already available in the active Python environment.

## Runners

| File | Purpose |
| --- | --- |
| `simulate.py` | Runs non-RL experiments from `exp_configs/non_rl/`. |
| `train.py` | Trains RL policies from `exp_configs/rl/singleagent/`. |

## Non-RL Simulations

Available configurations:

| Configuration | Description |
| --- | --- |
| `AllHumanDrivers_IDM44` | All-human baseline with IDM car-following and LC2013 lane-changing. |
| `hand_craft` | Hand-crafted AV baseline with independent controller behavior. |
| `paired_ring` | Proposed Pair-Aligned Rule-Based Controller (PARC). |

Run one simulation:

```bash
python simulate.py paired_ring
```

Run without the SUMO GUI:

```bash
python simulate.py paired_ring --no_render
```

Run multiple rollouts:

```bash
python simulate.py paired_ring --num_runs 3 --no_render
```

Generate emission outputs:

```bash
python simulate.py paired_ring --gen_emission
```

## RL Training

Available configurations:

| Configuration | Description |
| --- | --- |
| `singleagent_ring` | Single RL-controlled AV in the two-lane ring road. |
| `singleagent_ring2AV` | Two RL-controlled AVs for cooperative paired behavior. |

Train with RLlib:

```bash
python train.py singleagent_ring --rl_trainer rllib
python train.py singleagent_ring2AV --rl_trainer rllib
```

Adjust training resources and horizon:

```bash
python train.py singleagent_ring2AV --rl_trainer rllib --num_cpus 5 --num_steps 5000 --rollout_size 1000
```

Optional trainer backends are also wired into `train.py`:

```bash
python train.py singleagent_ring --rl_trainer stable-baselines
python train.py singleagent_ring --rl_trainer h-baselines
```

Use those only after installing the corresponding optional package.

## Outputs

- RLlib results are written to Ray's default result directory.
- Stable-Baselines outputs, when used, are written under `~/baseline_results`.
- Emission files from `--gen_emission` are written under `examples/data`.

## Notes

- Use `--no_render` on servers or headless environments.
- Rendering requires a working SUMO GUI installation.
- The experiment configs rely on Flow classes such as `RingNetwork`, `WaveAttenuationPOEnv`, `WaveAttenuationPOEnvCentral`, and the AV controllers used in the paper.
