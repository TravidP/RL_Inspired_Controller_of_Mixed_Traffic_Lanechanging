# Reviewer Comments Summary and Research Opportunities

This note summarizes the IFAC WC 2026 reviewer comments for **Control of Mixed-Autonomy Traffic via Autonomous Vehicles with Lane-Changing Behavior** and turns them into concrete revision actions and research opportunities.

中文简述：这份文档把审稿意见整理成“论文修改点”和“博士研究机会”，重点围绕集中式协同、多车道扩展、奖励设计、最优性、能耗舒适性和鲁棒性展开。

## Current Paper Context

The current project studies a two-lane 250 m ring road with 44 vehicles, Flow/SUMO simulation, LC2013 human lane-changing behavior, single-AV RL, centralized two-AV RL, independent rule-based controllers, and the proposed Pair-Aligned Rule-Based Controller (PARC). The main result is that a single AV is not enough when HDVs can change lanes, while coordinated cross-lane AV pairing can suppress lane-changing gaps and stabilize both lanes.

Relevant local artifacts:

- Paper source: [`ifac6pages.tex`](ifac6pages.tex)
- Project overview: [`../README.md`](../README.md)
- Reviewer text: [`../reviewercomments.txt`](../reviewercomments.txt)
- PARC config: [`../examples/exp_configs/non_rl/paired_ring.py`](../examples/exp_configs/non_rl/paired_ring.py)
- Cooperative RL config: [`../examples/exp_configs/rl/singleagent/singleagent_ring2AV.py`](../examples/exp_configs/rl/singleagent/singleagent_ring2AV.py)

## Overall Reviewer Signal

The review set is broadly positive. Reviewers recognize the paper's core insight: **human lane-changing is a fundamental destabilizing mechanism in multi-lane mixed traffic, and cross-lane AV coordination can suppress it**. The strongest concerns are not about whether the idea works in the presented setting, but about how far the claim can be generalized and how rigorously the learned pairing behavior has been explained.

中文简述：审稿人总体认可故事线和结果，但希望看到更强的证据：协同信息是否现实、能否扩展到更多车道、奖励项是否直接“设计出”pairing、控制器参数是否敏感、以及PARC距离系统最优还有多远。

## Review-by-Review Summary

| Reviewer | Positive Feedback | Main Concern | Actionable Response |
| --- | --- | --- | --- |
| Reviewer 5 | Clear motivation, realistic lane-changing setup, useful cooperative controller. | Centralized/shared information may be hard in deployment; only two-lane/two-AV setting; missing fuel, energy, comfort metrics; no sensitivity analysis. | Add a limitations paragraph now; plan experiments for decentralized/noisy communication, multi-lane scaling, emissions/jerk metrics, and PARC parameter sweeps. |
| Reviewer 6 | Valuable extension of Yan et al. style work; relevant topic. | Reward weights in equations (10)-(13) are not justified; feasibility of cooperative RL is unclear. | Add reward-weight rationale and a small ablation/sweep plan; report training cost, observation assumptions, convergence behavior, and deployment feasibility. |
| Reviewer 7 | Strongly supports the central message; accepts that one AV fails under lane-changing; praises RL-inspired PARC. | No major criticism. | Preserve the paper's simple causal story: lane-changing creates instability, pairing removes exploitable gaps, coordination matters. |
| Reviewer 8 | Paper is clear and motivated; PARC outperforms baselines. | "Emergent" pairing is not fully justified because reward terms explicitly penalize AV separation and speed mismatch; no reward ablation; no proof or benchmark of system optimality. | Soften the emergence claim or run ablations; distinguish "relative improvement" from "optimality"; add an optimal-control/MPC or exhaustive benchmark direction. |
| Reviewer 11 | Finds the work interesting, convincing, readable, and useful for future generalized environments. | No major criticism. | Use this as support for a broader PhD agenda on generalized multi-lane mixed traffic. |

## Cross-Cutting Themes

| Theme | Reviewer Pressure | Current Paper Evidence | Research Opportunity |
| --- | --- | --- | --- |
| Centralized information and communication | Reviewer 5, Reviewer 6 | Two-AV RL uses centralized observations and shared reward; PARC assumes partner state. | Study decentralized PARC/MARL with local sensing, V2V messages, delay, packet loss, and observation noise. |
| Scalability | Reviewer 5, Reviewer 11 | Only a two-lane ring with exactly two AVs is evaluated. | Extend to 3+ lanes, higher AV densities, longer rings, open roads, bottlenecks, merges, and weaving sections. |
| Reward design | Reviewer 6, Reviewer 8 | Pairing is encouraged through position and velocity synchronization terms. | Run reward ablations and weight sweeps to separate learned discovery from reward-shaped behavior. |
| Optimality | Reviewer 8 | PARC beats selected baselines but no system-optimal benchmark is provided. | Compare PARC to finite-horizon MPC, offline optimal control, dynamic programming on simplified models, or upper-bound search. |
| Energy, emissions, comfort, safety | Reviewer 5 | Smoothness penalty exists, but no explicit fuel/comfort/safety metrics are reported. | Add SUMO emissions, acceleration RMS, jerk, braking events, TTC, lane-change counts, and wave-amplitude metrics. |
| Parameter sensitivity | Reviewer 5, Reviewer 6 | PARC gains and thresholds are hand selected; RL weights are not systematically justified. | Use grid search, Latin hypercube sampling, or Bayesian optimization over controller gains and reward weights. |
| Feasibility of cooperative RL | Reviewer 6 | Cooperative RL is used as a discovery tool, but deployment requirements are not fully discussed. | Separate "RL as scientific microscope" from "RL as deployable controller"; report training cost and runtime assumptions. |

中文简述：这些主题可以直接转化为博士课题。最有价值的方向不是简单再做一个controller，而是研究“在信息不完美、多车道、多目标、多场景下，cross-lane coordination什么时候有效、为什么有效、如何可部署”。

## Recommended Paper Revision Actions

1. **Soften the emergence wording.** Replace strong phrases like "emergent cross-lane paired formation" with "reward-shaped learned paired behavior" unless ablation results are added. A safer wording is: "The cooperative RL policy consistently converges to a paired structure under a reward that includes synchronization incentives."

2. **Add an explicit limitations paragraph.** Mention that the current experiments use two lanes, two AVs, centralized/pairwise information, and one ring-road geometry. State that these are controlled first steps rather than proof of network-wide scalability.

3. **Add a reward-weight explanation.** Explain that position and synchronization weights encourage pairing, speed weight prevents overly conservative control, and smoothness weight penalizes aggressive acceleration. If no experiments are added yet, describe weight selection as empirical and identify sensitivity analysis as future work.

4. **Add metrics definitions even before full results.** Define fuel/emissions, acceleration RMS, jerk, braking frequency, TTC, lane-change count, wave amplitude, average speed, and throughput as the next evaluation set.

5. **Clarify optimality claims.** State that PARC demonstrates relative improvement over selected baselines, not system optimality. Avoid saying the paired structure is theoretically optimal unless a benchmark or proof is added.

6. **Explain cooperative RL feasibility.** Frame cooperative RL as a policy-discovery tool that motivates PARC. This reduces the deployment burden because the final PARC controller is simpler, but PARC still needs partner-state access or a reliable estimator.

## High-Priority Experiments

| Experiment | Reviewer Concern Addressed | Implementation Sketch in Current Project |
| --- | --- | --- |
| Reward ablation | Emergence claim; reward weights | In `singleagent_ring2AV.py` / environment reward, train variants without position, sync, speed, or smoothness terms. Compare alignment error, lane-change count, speed, jerk, and wave amplitude. |
| PARC sensitivity | Controller gains and thresholds | Sweep `k_pair`, `k_sync`, `k_v`, `d_th`, `delta_v`, and acceleration limits in the PARC controller. Use multiple random HDV seeds. |
| Communication realism | Centralized/shared information | Replace perfect partner state with delayed, noisy, or dropped V2V messages. Compare centralized PARC, local-estimated PARC, and no-communication baselines. |
| Multi-lane scaling | Generalization | Increase `lanes` to 3 or 4, vary AV density, and test pair, chain, and group formations. |
| Eco-comfort evaluation | Fuel/energy/comfort | Run `python simulate.py paired_ring --gen_emission --no_render`; compute fuel, CO2, acceleration RMS, jerk, hard braking, TTC, and lane-change count. |
| Optimality benchmark | System optimum gap | Build a finite-horizon MPC or offline search on a simplified IDM/LC model, then compare PARC against the best achievable speed/stability trade-off. |

## Best Research Framing

The strongest future-work framing is:

> Current PARC shows that cross-lane AV coordination can suppress lane-changing-induced waves in a controlled two-lane ring. The next research challenge is to discover when such cross-lane coordination remains effective under realistic information limits, larger networks, heterogeneous HDVs, and multi-objective constraints.

中文简述：博士论文可以从“PARC作为一个现象”出发，进一步回答三个深层问题：为什么pairing有效、在什么条件下有效、如何在真实多车道网络中可靠部署。

