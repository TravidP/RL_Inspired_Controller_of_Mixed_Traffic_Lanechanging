# Related Work Review for Future PARC Research

This document reviews related research around mixed-autonomy traffic, RL-based AV control, multi-lane lane-changing, scalability, decentralization, and multi-objective evaluation. It focuses on how each literature group can support future research after the current PARC paper.

中文简述：这不是完整文献综述章节，而是围绕当前项目和博士研究方向整理的“可用文献地图”。每组文献都说明它能支持什么研究点，以及当前项目还能补什么gap。

## 1. Foundations: Stop-and-Go Wave Dissipation and Mixed-Autonomy Testbeds

| Work | Main Idea | Relevance to Current Project | Gap to Extend |
| --- | --- | --- | --- |
| Stern et al., [Dissipation of stop-and-go waves via control of autonomous vehicles: Field experiments](https://doi.org/10.1016/j.trc.2018.02.005) | Shows experimentally that a small number of AVs can smooth traffic waves in circular-road traffic. | Provides the physical motivation for AV-based wave damping and metrics such as speed smoothing and fuel economy. | Mostly single-lane/longitudinal; does not address HDV lane-changing as the dominant disturbance. |
| Wu et al., [Flow: A Modular Learning Framework for Mixed Autonomy Traffic](https://arxiv.org/abs/1710.05465) | Introduces Flow, combining SUMO with RL frameworks for mixed-autonomy traffic experiments. | Your repo uses Flow/SUMO-style configs, ring roads, RL training, and mixed-autonomy evaluation. | The framework enables many tasks, but PARC-specific cross-lane formation is a new control mechanism. |
| Zheng et al., [Smoothing Traffic Flow via Control of Autonomous Vehicles](https://arxiv.org/abs/1812.09544) | Studies control strategies and stabilizability for AV smoothing in mixed traffic. | Supports the control-theoretic side of traffic-wave damping and motivates AV acceleration control. | Lane-changing and cross-lane coordination are not the core focus. |
| Yan et al., [Unified Automatic Control of Vehicular Systems With Reinforcement Learning](https://doi.org/10.1109/TASE.2022.3168621) | Uses RL to derive/learn automatic vehicular control policies in traffic systems. | Reviewer 6 explicitly sees the current paper as an extension of this style of work. | Current paper adds human lane-changing and two-AV cross-lane pairing, but needs clearer reward feasibility and weight analysis. |
| Chou et al., [The Lord of the Ring Road: A Review and Evaluation of Autonomous Control Policies for Traffic in a Ring Road](https://doi.org/10.1145/3494577) | Reviews and evaluates many ring-road AV control policies and metrics. | Useful for benchmark design, metric selection, and avoiding overly narrow comparisons. | Ring-road policy review should be extended to multi-lane lane-changing and paired formations. |

**How this supports a PhD direction:** These works establish that AVs can smooth waves and that Flow/SUMO is a credible experimental base. Your contribution can be framed as moving the field from **single-lane longitudinal smoothing** to **multi-lane lane-changing-aware coordination**.

中文简述：基础文献证明“AV可以抑制波”和“Flow/SUMO是合理平台”。你的差异点是把问题推进到human lane-changing和cross-lane coordination。

## 2. Multi-Lane Mixed Autonomy and Lane-Changing

| Work | Main Idea | Relevance to Current Project | Gap to Extend |
| --- | --- | --- | --- |
| Kreidieh et al., [Learning Generalizable Multi-Lane Mixed-Autonomy Traffic Control](https://arxiv.org/abs/2111.06318) | Studies multi-lane mixed-autonomy control and representations that transfer across lane settings. | Directly related to reviewer concern about scaling beyond a two-lane ring. | Focuses on generalizable control, not specifically on pair-aligned suppression of HDV lane-changing gaps. |
| Zhou et al., [Multi-Agent Reinforcement Learning for Cooperative Lane Changing of Connected and Autonomous Vehicles in Mixed Traffic](https://doi.org/10.1007/s42979-022-01154-3) | Uses MARL for cooperative CAV lane-changing, considering efficiency, safety, fuel, and comfort. | Supports multi-agent learning, lane-changing, and multi-objective metrics for future experiments. | Its target is cooperative CAV lane-changing, while PARC targets stabilization by shaping cross-lane gaps and HDV behavior. |
| Li, Dong, and Wu, [Hybrid System Stability and Traffic Stabilization of Mixed-Autonomy Multi-Lane Traffic](https://arxiv.org/abs/2504.04691) | Provides a hybrid-system view of stability in multi-lane mixed-autonomy traffic with lane-switching effects. | Highly relevant for building theory around when lane-changing breaks stability and how AV control can restore it. | Can be connected to PARC by analyzing paired AVs as a hybrid control structure across lanes. |
| Li, Armijos, and Cassandras, [Robust Optimal Lane-Changing Control for Connected and Automated Vehicles](https://doi.org/10.1016/j.automatica.2024.111817) | Develops robust optimal control for CAV lane-changing in the presence of HDVs. | Useful for optimality, safety constraints, and robust treatment of HDV interactions. | Focuses on maneuver-level optimal lane-changing rather than network-level stop-and-go stabilization. |

**How this supports a PhD direction:** This literature can support a chapter on **multi-lane generalization**. The important distinction is that your current work treats lane-changing not only as an AV maneuver decision, but as a system disturbance created by HDVs exploiting gaps.

中文简述：多车道文献很多研究“AV如何换道”，而你的工作更关注“HDV换道如何破坏稳定，以及AV如何通过跨车道结构抑制这种扰动”。

## 3. Decentralization, Scalability, and Large-Scale Mixed Traffic

| Work | Main Idea | Relevance to Current Project | Gap to Extend |
| --- | --- | --- | --- |
| Vinitsky et al., [Optimizing Mixed Autonomy Traffic Flow With Decentralized Autonomous Vehicles and Multi-Agent RL](https://arxiv.org/abs/1804.03764) | Uses decentralized AV control and MARL to improve mixed traffic flow in larger scenarios. | Directly supports moving from centralized two-AV RL to decentralized coordination. | Does not focus on PARC-style cross-lane paired formations under HDV lane-changing pressure. |
| Liu et al., [Large Scale Mixed Traffic Control Using Decentralized Multi-Agent Reinforcement Learning](https://arxiv.org/abs/2504.19869) | Studies large-scale mixed traffic control with decentralized MARL. | Useful for arguing that scalability and decentralization are central open problems. | Needs adaptation from large-scale intersection/network control to lane-changing-induced wave suppression. |
| Islam and Li, [Multi-Objective Decentralized Coordination of Connected Autonomous Vehicles in Large-Scale Mixed Traffic](https://arxiv.org/abs/2505.03255) | Studies decentralized multi-objective CAV coordination with system-level metrics. | Supports future work on fairness, safety, emissions, and large-scale coordination. | Future PARC research can contribute a specific cross-lane formation mechanism within this broader multi-objective setting. |
| Valiente et al., [Robustness and Adaptability of Reinforcement Learning-Based Cooperative Autonomous Driving in Mixed-Autonomy Traffic](https://arxiv.org/abs/2202.00881) | Studies robustness and adaptability of cooperative RL under varying mixed-traffic conditions. | Supports experiments on HDV heterogeneity, generalization, and robustness. | Pair-aligned control and lane-changing suppression can be added as a concrete robust-coordination case. |

**How this supports a PhD direction:** These works justify a thesis chapter on **decentralized scalable PARC/MARL**. Reviewer 5's centralization criticism can become a research question: how much information sharing is necessary to stabilize lane-changing mixed traffic?

中文简述：去中心化和大规模MARL文献可以支撑你的扩展方向，但你的独特机制仍然是cross-lane paired/gap control。

## 4. Reward Design, Interpretability, and RL-to-Rule Extraction

The current paper uses cooperative RL as a discovery tool and PARC as an interpretable distilled controller. This is a strong idea, but Reviewer 8 points out that the learned pair alignment is partly encouraged by reward terms that penalize longitudinal separation and velocity mismatch.

Useful research framing:

- Treat RL not as proof of spontaneous emergence, but as a **policy search instrument**.
- Use reward ablation to identify which reward terms are necessary for pairing.
- Use policy distillation to convert learned patterns into rule-based controllers.
- Report when learned behavior is robust under reward perturbation, seed changes, and HDV model changes.

Related support:

- Wu et al.'s Flow work supports RL experimentation in traffic control.
- Yan et al.'s unified RL work supports learning-based controller discovery.
- Chou et al.'s ring-road evaluation work supports systematic benchmark design.
- Valiente et al.'s robustness work supports evaluation across environment shifts.

**How to extend current work:** Add reward weights as explicit experimental variables and produce a "policy morphology" map: unstable, conservative platoon, paired-low-speed, paired-efficient, and high-speed-unstable regimes.

中文简述：建议不要只强调“emergent”，而是研究“reward shaping如何产生可解释控制结构”。这会让论文更严谨，也更有博士研究深度。

## 5. Optimality and Benchmarking

Reviewer 8 asks whether PARC is system-optimal or near-optimal. The current paper only shows that PARC improves over selected baselines. That is still valuable, but a PhD project can strengthen the claim by adding benchmark upper bounds.

Relevant benchmark ideas:

| Benchmark Type | What It Provides | How to Use With PARC |
| --- | --- | --- |
| Finite-horizon MPC | A model-based baseline with constraints. | Compare PARC against MPC using the same ring state and AV limits. |
| Offline trajectory optimization | Approximate upper bound under simplified HDV dynamics. | Estimate how much performance PARC leaves on the table. |
| Exhaustive/grid search over PARC gains | Best hand-tuned PARC variant. | Separate controller-structure value from tuning quality. |
| RL upper bound | High-capacity centralized controller. | Compare interpretable PARC to learned high-performance policies. |
| Simplified hybrid-system analysis | Stability insight. | Connect lane-changing events and pair formation to formal stability regions. |

Useful related work includes Zheng et al. for control-theoretic smoothing, Li/Dong/Wu for hybrid multi-lane stability, and Li/Armijos/Cassandras for robust optimal lane-changing control.

中文简述：最优性方向不一定要证明PARC全局最优。更现实的目标是量化gap：PARC相比MPC、offline optimum或best tuned controller差多少。

## 6. Energy, Emissions, Comfort, Safety, and Fairness Metrics

Reviewer 5 notes that smoothness penalties exist but explicit fuel, energy, and comfort metrics are missing. This is a high-impact and relatively feasible extension because the repo already supports SUMO emission generation through `--gen_emission`.

Recommended metric set:

| Metric | Why It Matters | Implementation Path |
| --- | --- | --- |
| Average speed and throughput | Efficiency baseline. | Already reported; extend with confidence intervals across seeds. |
| Wave amplitude | Direct stop-and-go severity. | Compute speed variance or low-speed wave propagation over time-space trajectories. |
| Lane-change count | Main disturbance mechanism in this paper. | Extract from SUMO lane index changes or trajectory logs. |
| Fuel and CO2 | Environmental impact. | Use SUMO emission output from `python simulate.py paired_ring --gen_emission --no_render`. |
| Acceleration RMS and jerk | Passenger comfort and smoothness. | Compute from vehicle acceleration time series. |
| Hard braking events | Safety and comfort. | Count acceleration below a threshold such as -2 or -3 m/s^2. |
| TTC / surrogate safety | Collision-risk proxy. | Compute time-to-collision from leader gap and relative speed where available. |
| Fairness across lanes/vehicles | Avoid sacrificing one lane or vehicle class. | Compare per-lane and per-vehicle speed, delay, and braking distributions. |

Related literature support:

- Stern et al. and Chou et al. motivate fuel economy and ring-road benchmark metrics.
- Zhou et al. explicitly includes efficiency, safety, fuel consumption, and comfort in cooperative lane-changing MARL.
- Islam and Li support multi-objective large-scale coordination and fairness-style evaluation.

中文简述：评价指标扩展是最容易快速补强论文的方向之一。建议从emission、jerk、braking、lane-change count、wave amplitude开始。

## 7. Recommended Literature-to-Experiment Map

| Research Point | Most Useful Related Work | First Experiment |
| --- | --- | --- |
| Decentralized PARC | Vinitsky et al.; Liu et al. 2025; Valiente et al. | Delayed/noisy/dropped partner-state experiment. |
| Multi-lane scaling | Kreidieh et al.; Li/Dong/Wu | 3-lane and 4-lane ring with AV formations. |
| Reward ablation | Wu et al.; Yan et al.; Valiente et al. | Remove position/sync/speed/smoothness terms and retrain. |
| Optimality gap | Zheng et al.; Li/Armijos/Cassandras; Li/Dong/Wu | MPC or offline optimal benchmark in the two-lane ring. |
| Eco-comfort PARC | Stern et al.; Chou et al.; Zhou et al. | SUMO emissions plus jerk/braking/TTC post-processing. |
| Robust HDV behavior | Valiente et al.; Zhou et al.; LC2013/SUMO literature | Controlled sweeps over LC2013 aggressiveness and cooperation. |
| Open networks | Flow; Vinitsky et al.; Liu et al. 2025 | Bottleneck/merge/weaving scenario with lane-change guidance. |

## References and Links

- Stern et al., "Dissipation of stop-and-go waves via control of autonomous vehicles: Field experiments." Transportation Research Part C, 2018. https://doi.org/10.1016/j.trc.2018.02.005
- Wu et al., "Flow: A Modular Learning Framework for Mixed Autonomy Traffic." arXiv, 2017. https://arxiv.org/abs/1710.05465
- Zheng et al., "Smoothing Traffic Flow via Control of Autonomous Vehicles." arXiv, 2018. https://arxiv.org/abs/1812.09544
- Yan et al., "Unified Automatic Control of Vehicular Systems With Reinforcement Learning." IEEE T-ASE, 2023. https://doi.org/10.1109/TASE.2022.3168621
- Chou et al., "The Lord of the Ring Road: A Review and Evaluation of Autonomous Control Policies for Traffic in a Ring Road." ACM Transactions on Cyber-Physical Systems, 2022. https://doi.org/10.1145/3494577
- Kreidieh et al., "Learning Generalizable Multi-Lane Mixed-Autonomy Traffic Control." arXiv, 2021. https://arxiv.org/abs/2111.06318
- Zhou et al., "Multi-Agent Reinforcement Learning for Cooperative Lane Changing of Connected and Autonomous Vehicles in Mixed Traffic." SN Computer Science, 2022. https://doi.org/10.1007/s42979-022-01154-3
- Li, Dong, and Wu, "Hybrid System Stability and Traffic Stabilization of Mixed-Autonomy Multi-Lane Traffic." arXiv, 2025. https://arxiv.org/abs/2504.04691
- Li, Armijos, and Cassandras, "Robust Optimal Lane-Changing Control for Connected and Automated Vehicles." Automatica, 2025. https://doi.org/10.1016/j.automatica.2024.111817
- Vinitsky et al., "Optimizing Mixed Autonomy Traffic Flow With Decentralized Autonomous Vehicles and Multi-Agent Reinforcement Learning." arXiv, 2018. https://arxiv.org/abs/1804.03764
- Liu et al., "Large Scale Mixed Traffic Control Using Decentralized Multi-Agent Reinforcement Learning." arXiv, 2025. https://arxiv.org/abs/2504.19869
- Islam and Li, "Multi-Objective Decentralized Coordination of Connected Autonomous Vehicles in Large-Scale Mixed Traffic." arXiv, 2025. https://arxiv.org/abs/2505.03255
- Valiente et al., "Robustness and Adaptability of Reinforcement Learning-Based Cooperative Autonomous Driving in Mixed-Autonomy Traffic." arXiv, 2022. https://arxiv.org/abs/2202.00881

中文简述：这些文献可以作为博士开题或论文related work的骨架。后续写正式论文时，可以再补充SUMO LC2013、IDM、CACC、MPC和交通流稳定性理论的经典文献。

