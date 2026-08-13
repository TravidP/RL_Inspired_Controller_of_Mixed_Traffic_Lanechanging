# RL-Inspired Controller of Mixed Traffic with Lane-Changing

[![Framework: Flow](https://img.shields.io/badge/Framework-Flow-blue.svg)](https://flow-project.github.io/)
[![Simulator: SUMO](https://img.shields.io/badge/Simulator-SUMO-green.svg)](https://www.eclipse.org/sumo/)
[![Python: 3.7.3](https://img.shields.io/badge/Python-3.7.3-yellow.svg)](https://www.python.org/)

This repository contains simulation scripts, experiment configurations, and visual results for **Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior**.

The project studies a double-lane ring road with human lane-changing, where heterogeneous human drivers can trigger persistent stop-and-go oscillations. It builds on the [Flow](https://flow-project.github.io/) traffic-control framework and investigates how autonomous vehicles (AVs) can stabilize mixed traffic under realistic lane-changing behavior.

## Highlights

- Models a two-lane mixed-autonomy ring road with 44 vehicles and human lane-changing behavior.
- Compares all-human traffic, single-AV RL, cooperative two-AV RL, independent hand-crafted AV controllers, and the proposed paired controller.
- Introduces a **Pair-Aligned Rule-Based Controller (PARC)** inspired by cooperative RL behavior.
- Coordinates two AVs across lanes so that they form a paired moving structure and reduce disruptive lane-changing gaps.
- Improves stabilized average speed by **7.4%** compared with two independent single-lane controllers.

## Paper

- [IFAC manuscript PDF](Docs/ifac6pages.pdf)
- [Full repository paper PDF](Control_of_Mixed_Autonomy_Traffic_via_Autonomous_Vehicles_with_Lane_Changing_Behavior.pdf)

## Method At A Glance

PARC synchronizes two AVs in adjacent lanes and treats the two-lane road as a virtual single-lane system. This paired formation is designed to suppress unnecessary human lane changes, damp stop-and-go waves, and preserve higher average speed than conservative independent controllers.

<table>
  <tr>
    <td width="50%" valign="top">
      <img src="Docs/doublelaneringnetwork_2.png" width="100%" alt="Double-lane ring road topology">
      <br>
      <em>Double-lane ring road used in the experiments.</em>
    </td>
    <td width="50%" valign="top">
      <img src="Docs/The%20overview%20of%20paired%20controller%20for%20stabilizing%20flow.png" width="100%" alt="Overview of the paired controller">
      <br>
      <em>Paired-controller concept for stabilizing mixed traffic.</em>
    </td>
  </tr>
</table>

## Repository Layout

```text
.
|-- Docs/
|   |-- *.png                 # Figures, time-space diagrams, and result plots
|   |-- *.mp4                 # Simulation recordings
|   |-- ifac6pages.pdf        # Paper manuscript
|   `-- ifac6pages.tex        # Paper source
|-- examples/
|   |-- simulate.py           # Runner for non-RL simulations
|   |-- train.py              # Runner for RL training experiments
|   `-- exp_configs/
|       |-- non_rl/           # All-human, hand-crafted, and PARC configs
|       `-- rl/singleagent/   # Single-AV and two-AV RL configs
|-- environment.yml           # Conda environment for Flow-style setup
|-- environment1.yml          # Alternative Conda environment name
|-- requirements.txt          # Python dependencies
|-- Dockerfile                # Reference Flow/SUMO Docker setup
`-- setup.py                 # Flow-style package metadata
```

## Experiment Configurations

| Configuration | Type | Purpose |
| --- | --- | --- |
| `AllHumanDrivers_IDM44` | Non-RL | All-human baseline with lane-changing IDM drivers. |
| `hand_craft` | Non-RL | Hand-crafted AV baseline with independent single-lane-style control. |
| `paired_ring` | Non-RL | Proposed PARC controller with two paired AVs. |
| `singleagent_ring` | RL | Single RL-controlled AV in the double-lane ring road. |
| `singleagent_ring2AV` | RL | Two RL-controlled AVs used to study cooperative paired behavior. |

## Visual Results

All figures below are shown at one-column width in a two-column layout.

### Controller Design And Metrics

<table>
  <tr>
    <td width="50%" valign="top">
      <img src="Docs/The%20flowchart%20of%20paired%20controller.png" width="100%" alt="PARC flowchart">
      <br>
      <em>PARC flowchart.</em>
    </td>
    <td width="50%" valign="top">
      <img src="Docs/avg_speed_comparison.png" width="100%" alt="Average speed comparison">
      <br>
      <em>Average speed comparison across controllers.</em>
    </td>
  </tr>
</table>

### Time-Space Diagrams

<table>
  <tr>
    <td width="50%" valign="top">
      <img src="Docs/trajectory_log_allhuman44.png" width="100%" alt="All-human stop-and-go trajectory">
      <br>
      <em>All-human traffic: persistent stop-and-go oscillations.</em>
    </td>
    <td width="50%" valign="top">
      <img src="Docs/trajectory_log_paired_controller.png" width="100%" alt="PARC trajectory">
      <br>
      <em>PARC: stabilized traffic with paired AV coordination.</em>
    </td>
  </tr>
  <tr>
    <td width="50%" valign="top">
      <img src="Docs/trajectory_log_1AVLC.png" width="100%" alt="Single AV RL trajectory">
      <br>
      <em>Single RL AV: lane-changing pressure remains difficult to suppress.</em>
    </td>
    <td width="50%" valign="top">
      <img src="Docs/trajectory_log_2AVCentral.png" width="100%" alt="Cooperative two-AV RL trajectory">
      <br>
      <em>Cooperative two-AV RL: emergent cross-lane paired behavior.</em>
    </td>
  </tr>
  <tr>
    <td width="50%" valign="top">
      <img src="Docs/trajectory_log_handcraft.png" width="100%" alt="One-AV single-lane controller trajectory">
      <br>
      <em>One-AV single-lane controller baseline.</em>
    </td>
    <td width="50%" valign="top">
      <img src="Docs/trajectory_log_handcraft2av.png" width="100%" alt="Two-AV independent controller trajectory">
      <br>
      <em>Two independent single-lane controllers baseline.</em>
    </td>
  </tr>
</table>

### Simulation Videos

The video thumbnails link to the hosted demos. Local copies are also stored in `Docs/*.mp4`.

<table>
  <tr>
    <td width="50%" valign="top">
      <a href="https://youtu.be/lSaD2bFGrEA">
        <img src="https://img.youtube.com/vi/lSaD2bFGrEA/hqdefault.jpg" width="100%" alt="All-human stop-and-go video">
      </a>
      <br>
      <em>All-human stop-and-go baseline.</em>
    </td>
    <td width="50%" valign="top">
      <a href="https://youtube.com/shorts/GptLjUW8utM?feature=share">      
        <img src="https://img.youtube.com/vi/W-hXR4RJz5o/hqdefault.jpg" width="100%" alt="PARC video">
      </a>
      <br>
      <em>PARC controller.</em>
    </td>
  </tr>
  <tr>
    <td width="50%" valign="top">
      <a href="https://youtu.be/QN3oOsjQgOg">
        <img src="https://img.youtube.com/vi/QN3oOsjQgOg/hqdefault.jpg" width="100%" alt="Single RL AV video">
      </a>
      <br>
      <em>Single RL AV.</em>
    </td>
    <td width="50%" valign="top">
      <a href="https://youtu.be/2ezlDmujKCk">
        <img src="https://img.youtube.com/vi/2ezlDmujKCk/hqdefault.jpg" width="100%" alt="Cooperative two-AV RL video">
      </a>
      <br>
      <em>Cooperative two-AV RL.</em>
    </td>
  </tr>
  <tr>
    <td width="50%" valign="top">
      <a href="https://youtu.be/ETK8Jipwp1Q">
        <img src="https://img.youtube.com/vi/ETK8Jipwp1Q/hqdefault.jpg" width="100%" alt="One-AV single-lane controller video">
      </a>
      <br>
      <em>One-AV single-lane controller.</em>
    </td>
    <td width="50%" valign="top">
      <a href="https://youtu.be/Bmx-kGFCUmU">
        <img src="https://img.youtube.com/vi/Bmx-kGFCUmU/hqdefault.jpg" width="100%" alt="Two-AV single-lane controller video">
      </a>
      <br>
      <em>Two-AV independent single-lane controllers.</em>
    </td>
  </tr>
</table>

## Installation

This project targets **Python 3.7.3**, [Flow](https://flow-project.github.io/), and [SUMO](https://www.eclipse.org/sumo/). The experiment scripts assume that Flow and SUMO are installed and importable in the active environment.

### 1. Clone The Repository

```bash
git clone https://github.com/TravidP/RL_Inspired_Controller_of_Mixed_Traffic_Lanechanging.git
cd RL_Inspired_Controller_of_Mixed_Traffic_Lanechanging
```

### 2. Create The Conda Environment

```bash
conda env create -f environment.yml
conda activate flow
```

The repository also includes `environment1.yml`, which defines the same dependency set under the environment name `newflow`.

### 3. Install Flow And SUMO

Follow the Flow setup instructions for your platform:

- Flow setup: https://flow.readthedocs.io/en/latest/flow_setup.html
- SUMO setup: https://sumo.dlr.de/docs/Installing/index.html

Make sure the active environment can import Flow modules and access SUMO tools:

```bash
python -c "import flow; print('Flow import OK')"
python -c "import traci; print('TraCI import OK')"
```

## Running Experiments

Run commands from the `examples/` directory so that the experiment configuration modules are found correctly.

```bash
cd examples
```

### Non-RL Simulations

```bash
python simulate.py AllHumanDrivers_IDM44
python simulate.py hand_craft
python simulate.py paired_ring
```

Useful options:

```bash
python simulate.py paired_ring --num_runs 3 --no_render
python simulate.py paired_ring --gen_emission
```

### RL Training

```bash
python train.py singleagent_ring --rl_trainer rllib
python train.py singleagent_ring2AV --rl_trainer rllib
```

Optional training controls:

```bash
python train.py singleagent_ring2AV --rl_trainer rllib --num_cpus 5 --num_steps 5000 --rollout_size 1000
```

`train.py` also accepts `--rl_trainer stable-baselines` and `--rl_trainer h-baselines` when those optional packages are installed.

## Reproducibility Notes

- Non-RL configurations use fixed seeds where available to improve repeatability.
- Several configurations randomize human-driver parameters to represent heterogeneous lane-changing behavior.
- Rendering requires a working SUMO GUI installation. Use `--no_render` for headless runs.
- `--gen_emission` stores emission outputs under `examples/data`.
<!-- 
## Related Files

- [examples/README.md](examples/README.md) gives a focused guide to the runnable experiment scripts.
- [Docs/ifac6pages.tex](Docs/ifac6pages.tex) contains the paper source.
- [Dockerfile](Dockerfile) documents a reference Flow/SUMO container setup. -->

## Citation

If this repository helps your research, please cite the accompanying paper:

```bibtex
@inproceedings{pei_control_mixed_autonomy_lane_changing,
  title = {Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior},
  author = {Pei, Shuwei and Sayin, Muhammed O. and Ahmed, Saeed},
  note = {Manuscript available in this repository}
}
```
