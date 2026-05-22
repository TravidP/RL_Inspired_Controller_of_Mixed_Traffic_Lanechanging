# PhD Research Directions After PARC

This roadmap turns the current paper and reviewer comments into possible PhD research directions. The goal is not to choose one narrow topic immediately, but to identify several strong directions that naturally continue from the current project.

中文简述：这份文档提供多个博士研究方向。每个方向都连接当前项目的PARC、two-AV cooperative RL、SUMO/Flow、LC2013 lane-changing和审稿意见，并给出可立即开始的下一步实验。

## Current Research Anchor

Your current work establishes a valuable starting point:

- Human lane-changing can destroy the stabilizing effect of a single AV in a two-lane ring.
- Centralized cooperative RL with two AVs produces a cross-lane paired behavior under synchronization-aware reward terms.
- PARC turns that behavior into an interpretable rule-based controller.
- PARC stabilizes both lanes and improves average speed compared with two independent single-lane controllers.

The reviewers now point toward a larger thesis question:

> How can AVs coordinate across lanes to stabilize mixed traffic when information is local, human lane-changing is heterogeneous, AV penetration varies, and performance must include efficiency, safety, energy, and comfort?

中文简述：当前论文回答了“two lanes + two AVs + perfect coordination”下pairing有效。博士阶段可以研究“不完美信息、多车道、多目标、多场景”下cross-lane coordination的理论和方法。

## Direction 1: Decentralized Cross-Lane Coordination Under Limited Communication

**Core question:** Can AVs reproduce the benefits of PARC without centralized observations or perfect partner-state access?

**Why it matters:** Reviewer 5 questions real-world shared information, and Reviewer 6 asks about cooperative RL feasibility. This direction directly turns those concerns into a thesis contribution.

**Research ideas:**

- Replace centralized observations with local leader/follower sensing plus optional V2V messages.
- Study delay, packet loss, noise, limited communication range, and asynchronous updates.
- Compare centralized RL, decentralized MARL, PARC with perfect partner state, PARC with estimated partner state, and no-communication baselines.
- Design event-triggered communication: only transmit partner state when alignment error or speed mismatch exceeds a threshold.

**Concrete next experiment in this repo:**

- Add a communication wrapper around the partner state used by PARC.
- Run four variants: perfect partner state, delayed state, noisy state, and dropped messages.
- Metrics: alignment error, lane-change count, average speed, wave amplitude, jerk, hard braking, and emissions.

**Potential thesis contribution:** A deployable cross-lane coordination framework that quantifies how much communication is actually needed for multi-lane mixed-traffic stabilization.

中文简述：这个方向最直接回应审稿人关于“centralized/shared information是否现实”的问题，也最容易发展成完整博士主线。

## Direction 2: Scalable PARC for Multi-Lane and Multi-AV Traffic

**Core question:** Does pair-aligned control generalize beyond two lanes and exactly two AVs?

**Why it matters:** Reviewer 5 explicitly notes that the paper only evaluates a two-lane ring with two AVs. A PhD thesis can study formation patterns across many lanes and AV densities.

**Research ideas:**

- Extend from pairs to cross-lane groups: pair, chain, platoon wall, staggered formation, and adaptive formation.
- Study odd numbers of lanes and uneven AV placement.
- Vary AV penetration rate, traffic density, lane-changing aggressiveness, and road length.
- Learn when blocking lane-changing gaps helps and when it becomes too restrictive.

**Concrete next experiment in this repo:**

- Create 3-lane and 4-lane ring configs using the current `RingNetwork` setup.
- Test several AV placement rules: one AV per lane, alternating lanes, random AVs, and AV groups.
- Compare PARC-style formations against independent single-lane controllers and centralized RL.

**Potential thesis contribution:** A scalable theory and controller design for cross-lane AV formations in multi-lane mixed traffic.

中文简述：从two-lane pair扩展到multi-lane formation，是从会议论文走向博士论文的自然升级。

## Direction 3: Reward Ablation and Interpretable Policy Discovery

**Core question:** Is the paired behavior truly discovered, or mainly induced by reward design?

**Why it matters:** Reviewer 8 correctly points out that position and speed synchronization rewards directly encourage alignment. This is a weakness for the current emergence claim, but it is also a strong research opening.

**Research ideas:**

- Run reward ablations: remove position penalty, velocity synchronization penalty, speed reward, and smoothness penalty.
- Sweep reward weights and map how policies transition between conservative platoons, unstable control, and efficient pairing.
- Use learned policies as a discovery tool, then distill them into interpretable rule-based controllers.
- Develop criteria for when RL-discovered structures are robust enough to become control laws.

**Concrete next experiment in this repo:**

- Add configurable reward weights for cooperative RL.
- Train a small matrix of policies: full reward, no position term, no sync term, no speed term, no smoothness term, and speed-dominant reward.
- Evaluate whether alignment appears, how quickly it forms, and whether lane changes stop.

**Potential thesis contribution:** A methodology for extracting interpretable traffic-control mechanisms from RL while separating reward-induced behavior from robust system-level structure.

中文简述：这个方向可以把审稿人的批评变成亮点：不是简单说“emergent”，而是系统研究reward shaping如何产生可解释控制结构。

## Direction 4: System-Optimal and Near-Optimal Benchmarks

**Core question:** How close is PARC to the best possible control strategy?

**Why it matters:** Reviewer 8 notes that the current paper shows relative improvement, not optimality. A strong PhD contribution would quantify the optimality gap.

**Research ideas:**

- Build simplified optimal-control benchmarks using IDM/LC approximations.
- Use MPC, offline trajectory optimization, or model-based search to estimate upper bounds.
- Compare objectives: maximum speed, minimum wave amplitude, minimum lane changes, minimum fuel, and comfort constraints.
- Study when PARC is near-optimal and when a different formation is better.

**Concrete next experiment in this repo:**

- Start with a finite-horizon MPC surrogate in the same ring scenario, using AV acceleration constraints and measured HDV dynamics.
- Compare PARC to MPC under identical initial conditions and random seeds.
- Report optimality gap in average speed, lane-change suppression, acceleration cost, and emissions.

**Potential thesis contribution:** A benchmark framework that turns "PARC works better than baselines" into "PARC achieves a quantified fraction of system-optimal performance."

中文简述：如果要提升论文理论深度，最优性gap是很有价值的方向。

## Direction 5: Energy-, Safety-, and Comfort-Aware Mixed-Traffic Control

**Core question:** Can cross-lane coordination stabilize traffic while also reducing fuel use, emissions, safety risk, and passenger discomfort?

**Why it matters:** Reviewer 5 points out that smoothness penalties are present but explicit fuel, energy, and comfort metrics are missing.

**Research ideas:**

- Add multi-objective evaluation: speed, throughput, fuel, CO2, acceleration RMS, jerk, braking events, TTC, and fairness.
- Study trade-offs between "blocking lane changes" and "maintaining comfort."
- Use Pareto fronts rather than one scalar reward.
- Compare eco-PARC, comfort-PARC, and safety-PARC variants.

**Concrete next experiment in this repo:**

- Use the existing `--gen_emission` option in `examples/simulate.py`.
- Generate emissions for all-human, single-AV RL, cooperative RL, independent SLC, and PARC.
- Add a post-processing script to compute jerk, hard braking, wave amplitude, lane-change count, and TTC-style safety indicators from trajectory/emission logs.

**Potential thesis contribution:** A multi-objective AV coordination framework that shows stabilization is not enough; the controller must also be eco-safe and comfortable.

中文简述：这个方向很适合补充论文评价指标，也能连接交通工程和控制领域更广泛的关注点。

## Direction 6: Robustness to HDV Heterogeneity and Real Driver Data

**Core question:** Does PARC still work when human driver behavior is more diverse or calibrated from real trajectory data?

**Why it matters:** The current project already randomizes IDM and LC2013 parameters. A thesis can turn this into systematic robustness analysis.

**Research ideas:**

- Sweep LC2013 assertiveness, cooperation, speed-gain, keep-right behavior, acceleration, deceleration, reaction noise, and min-gap parameters.
- Use domain randomization during RL training.
- Calibrate HDV behavior from datasets such as NGSIM, highD, or inD, then test whether PARC still suppresses lane-changing waves.
- Evaluate worst-case and distribution-shift performance.

**Concrete next experiment in this repo:**

- Convert the existing random HDV parameter generation in `paired_ring.py` into a controlled experiment grid.
- Run multiple seeds per driver population: cautious, aggressive, heterogeneous, high lane-change pressure, and low lane-change pressure.
- Report robustness bands instead of only one average speed.

**Potential thesis contribution:** A robust mixed-traffic control method with evidence across realistic human-driving distributions.

中文简述：当前代码已经有随机HDV参数，这是做鲁棒性研究的好基础。

## Direction 7: From Ring Roads to Bottlenecks, Merges, Weaving, and Intersections

**Core question:** Can cross-lane AV coordination stabilize mixed traffic in open networks where lane-changing has a purpose, not only a disturbance?

**Why it matters:** Ring roads are clean scientific testbeds, but real networks include merges, exits, weaving zones, bottlenecks, and intersections.

**Research ideas:**

- Test PARC-like coordination in on-ramps, lane drops, bottlenecks, and weaving segments.
- Distinguish harmful lane changes from necessary route-following lane changes.
- Combine AV control with infrastructure support such as variable speed limits or ramp metering.
- Study whether AVs should suppress, guide, or facilitate HDV lane changes depending on network context.

**Concrete next experiment in this repo:**

- Add a SUMO/Flow bottleneck or merge scenario.
- Compare "lane-change suppression" versus "lane-change guidance" policies.
- Track throughput, queue length, safety risk, lane-change success, and emissions.

**Potential thesis contribution:** A generalized cross-lane mixed-autonomy framework for realistic traffic networks.

中文简述：这个方向面向真实道路，难度更高，但也最容易体现博士论文的广度。

## Recommended Thesis Storylines

| Storyline | Best For | Central Contribution |
| --- | --- | --- |
| Distributed Cross-Lane Coordination for Multi-Lane Mixed Autonomy | Strong control/RL thesis | Move from centralized two-AV pairing to deployable decentralized coordination. |
| Interpretable RL-to-Rule Control for Lane-Changing Mixed Traffic | Strong learning/control bridge | Use RL to discover structures, ablate rewards, and distill interpretable controllers. |
| Eco-Safe Multi-Objective Mixed-Traffic Stabilization | Strong transportation systems thesis | Stabilize traffic while optimizing energy, emissions, comfort, and safety. |
| Scalable Formation Control for AVs in Multi-Lane Networks | Strong generalization thesis | Extend PARC from pairs to adaptive multi-AV formations across realistic networks. |

My recommended broad PhD spine is:

> **Scalable and decentralized cross-lane coordination for mixed-autonomy traffic with human lane-changing.**

This spine can include reward ablation, optimality benchmarks, emissions/comfort metrics, and robustness as chapters rather than disconnected side projects.

中文简述：最推荐的主线是“可扩展、去中心化、跨车道协同控制”。它能自然吸收所有审稿意见，并保持和当前PARC工作的连续性。

## First Six-Month Research Plan

1. **Month 1-2: Evaluation framework.** Add lane-change count, wave amplitude, acceleration RMS, jerk, hard braking, emissions, TTC-style safety, and alignment error.
2. **Month 2-3: Reward ablations.** Train cooperative RL variants and quantify when pairing appears.
3. **Month 3-4: PARC sensitivity.** Sweep gains/thresholds and report robustness across HDV seeds.
4. **Month 4-5: Communication realism.** Add delayed/noisy/dropped partner-state experiments.
5. **Month 5-6: Multi-lane pilot.** Run 3-lane and 4-lane ring scenarios with simple formation rules.

These experiments would directly answer the reviewers and generate the first chapter of a larger PhD thesis.

