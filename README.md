<!-- ```markdown -->
# RL-Inspired Controller of Mixed Traffic with Lane-Changing


[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.md)

**📄 Read our paper:** [Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior](Docs/ifac6pages.pdf)

This repository contains the code, simulation environments, and experimental configurations for the paper **"Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior."** This project extends the Flow computational framework to address the destabilizing effects of human lane-changing in multi-lane mixed-autonomy traffic. We introduce a novel rule-based, pair-aligned control strategy that synchronizes the motion of two Autonomous Vehicles (AVs) across lanes. By coupling two lanes into a single virtual lane, this controller successfully suppresses human lane-changing and mitigates stop-and-go oscillations, increasing the stabilized average speed by 7.4% compared to independent single-lane controllers.

<p align="center">
  <img src="Docs/The overview of paired controller for stabilizing flow.png" width="800">
  <br>
  <em>Paired-controller design. The proposed rule-based controller coordinates the AVs, suppresses disruptive lane changes, and achieves stability and high efficiency.</em>
</p>

## Table of Contents
- [Visualizations and Results](#visualizations-and-results)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Running Experiments](#running-experiments)
  - [Rule-Based Controllers (Non-RL)](#rule-based-controllers-non-rl)
  - [Reinforcement Learning (RL)](#reinforcement-learning-rl)
- [Citation](#citation)

## Visualizations and Results

Below are the time-space diagrams and simulation recordings demonstrating the performance of different controllers evaluated in our study on the double-lane ring road (see ![Network Topology](Docs/doublelaneringnetwork_2.png)).

### 1. Stop-and-Go Oscillations (All Human Drivers)
Persistent oscillations driven by human lane-changing.
* **Trajectory**: ![Stop-and-Go](Docs/trajectory_log_allhuman44.png)
* **Video**: [🎥 Watch Video](Docs/stop_go_original.mp4)

### 2. Single-Agent RL Controller
The single AV minimizes headways to suppress lane changes, but oscillations persist.
* **Trajectory**: ![Single RL AV](Docs/trajectory_log_1AVLC.png)
* **Video**: [🎥 Watch Video](Docs/RL_1AV.mp4)

### 3. Cooperative RL Controller
Two AVs form a cross-lane paired structure, acting as a unified moving bottleneck to eliminate merging gaps.
* **Trajectory**: ![Cooperative RL](Docs/trajectory_log_2AVCentral.png)
* **Video**: [🎥 Watch Video](Docs/RL_Coop_2AV.mp4)

### 4. Proposed Pair-Aligned Rule-Based Controller (PARC)
Couples both lanes into a virtual lane to maximize throughput and stability.
* **Flowchart**: ![Logic](Docs/The flowchart of paired controller.png)
* **Trajectory**: ![PARC Result](Docs/trajectory_log_paired_controller.png)
* **Video**: [🎥 Watch Video](Docs/PARC.mp4)

### 5. Baselines and Comparisons
* **1 AV (Single-lane Controller)**: ![1AV-SLC](Docs/trajectory_log_handcraft.png) | [🎥 Video](Docs/1AV-SLC.mp4)
* **2 AVs (Independent Single-lane Controllers)**: ![2AV-SLC](Docs/trajectory_log_handcraft2av.png) | [🎥 Video](Docs/2AV-SLC.mp4)
* **Average Speed Comparison**: ![Avg Speed Comparison](Docs/avg_speed_comparison.png)

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
