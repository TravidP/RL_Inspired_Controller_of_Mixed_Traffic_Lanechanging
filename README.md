<!-- ```markdown -->
# RL-Inspired Controller of Mixed Traffic with Lane-Changing

[![Build Status](https://travis-ci.com/flow-project/flow.svg?branch=master)](https://travis-ci.com/flow-project/flow)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.md)

This repository contains the code, simulation environments, and experimental configurations for the paper **"Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior."** This project extends the Flow computational framework to address the destabilizing effects of human lane-changing in multi-lane mixed-autonomy traffic. We introduce a novel rule-based, pair-aligned control strategy that synchronizes the motion of two Autonomous Vehicles (AVs) across lanes. By coupling two lanes into a single virtual lane, this controller successfully suppresses human lane-changing and mitigates stop-and-go oscillations, increasing the stabilized average speed by 7.4% compared to independent single-lane controllers.

## Table of Contents
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Running Experiments](#running-experiments)
  - [Rule-Based Controllers (Non-RL)](#rule-based-controllers-non-rl)
  - [Reinforcement Learning (RL)](#reinforcement-learning-rl)
- [Citation](#citation)

## Repository Structure

Based on the project's `.gitignore`, the core framework files from the original Flow repository are excluded to maintain focus on the novel traffic control experiments. The core contributions are located within the `examples/exp_configs/` directory:

* `examples/exp_configs/non_rl/`
    * `paired_ring.py`: Implementation of the proposed **Pair-Aligned Rule-based Controller (PARC)** coordinating two AVs.
    * `hand_craft.py`: A handcrafted controller enforcing synchronized motion across two lanes.
    * `AllHumanDrivers_IDM44.py`: Baseline environment featuring 44 human-driven vehicles utilizing the IDM and LC2013 lane-changing models.
* `examples/exp_configs/rl/singleagent/`
    * `singleagent_ring.py`: Single AV RL training environment.
    * `singleagent_ring2AV.py`: Cooperative centralized RL environment for two AVs.

## Installation

This project requires Python 3.7.3 and relies on SUMO for microscopic traffic simulation. 

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/TravidP/RL_Inspired_Controller_of_Mixed_Traffic_Lanechanging.git](https://github.com/TravidP/RL_Inspired_Controller_of_Mixed_Traffic_Lanechanging.git)
   cd RL_Inspired_Controller_of_Mixed_Traffic_Lanechanging
   ```

2. **Set up the Conda environment:**
   We provide an `environment.yml` file to handle all Python dependencies (including Ray, TensorFlow, and SUMO tools).
   ```bash
   conda env create -f environment.yml
   conda activate flow
   ```

3. **SUMO Setup:**
   Ensure SUMO is installed on your system and the `SUMO_HOME` environment variable is set. If you encounter XSD schema errors during simulation, you can run the provided fix script:
   ```bash
   bash examples/fix_sumo_xsd.sh
   ```

## Running Experiments

### Rule-Based Controllers (Non-RL)

To simulate the rule-based environments (such as our proposed PARC controller), use the `simulate.py` runner. The command below executes the pair-aligned ring road scenario:

```bash
python examples/simulate.py paired_ring
```

**Optional arguments:**
* `--num_runs INT`: Number of consecutive simulations to run.
* `--no_render`: Disable the SUMO GUI (useful for headless servers).

### Reinforcement Learning (RL)

To train the RL policies using RLlib, use the `train.py` runner. For example, to train the cooperative two-AV centralized controller:

```bash
python examples/train.py singleagent_ring2AV --rl_trainer rllib
```

## Citation

If you use this code or our modified simulation environments in your academic research, please cite our paper accepted at the 23rd International Federation of Automatic Control (IFAC) World Congress (Busan, South Korea, 2026):

```bibtex
@inproceedings{pei2026control,
  title={Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior},
  author={Pei, Shuwei and Sayin, Muhammed O. and Ahmed, Saeed},
  booktitle={23rd International Federation of Automatic Control (IFAC) World Congress},
  year={2026},
  address={Busan, South Korea}
}
```

## Acknowledgments
This repository utilizes the Flow framework originally developed by the Mobile Sensing Lab at UC Berkeley.
