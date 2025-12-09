from gym.spaces import Dict as GymDict, Box
import numpy as np
from flow.envs.ring.lane_change_accel import LaneChangeAccelEnv

ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration for autonomous vehicles, in m/s^2
    "max_accel": 3,
    # maximum deceleration for autonomous vehicles, in m/s^2
    "max_decel": 3,
    # lane change duration for autonomous vehicles, in s. Autonomous vehicles
    # reject new lane changing commands for this duration after successfully
    # changing lanes.
    "lane_change_duration": 1,
    # desired velocity for all vehicles in the network, in m/s
    "target_velocity": 30,
    # specifies whether vehicles are to be sorted by position during a
    # simulation step. If set to True, the environment parameter
    # self.sorted_ids will return a list of all vehicles sorted in accordance
    # with the environment
    "sort_vehicles": False,
    "ring_length": 250
}
V_MAX = 30.0
A_MAX = 3.0
EDGE_TYPE_DIM = 10   # [follow_fwd, follow_back, lateral_fwd, lateral_back, knn]
# K_NEIGH = 2         # kNN左右各k个
NUM_LANES = 2

class LaneChangeAccelCentralizedGNNEnv(LaneChangeAccelEnv):
    """Centralized GNN observation over the full 44-vehicle graph.
    - Observation: dict with x, edge_index, edge_attr, rl_idx (+ sizes)
    - Action: shape (num_rl, 2): [accel_norm, lc_intent]
    """

    # ------------- spaces -------------

    @property
    def observation_space(self):
        # 由于 Gym 需要静态 shape，这里做 padding 的上界（按 44 车估个上限）
        max_nodes = self.initial_vehicles.num_vehicles
        max_edges = self.initial_vehicles.num_rl_vehicles * 10  # 保守些 3+3+2+2
        d_x = 1 + NUM_LANES + 4    # s_norm + lane1hot2 + [v,a,lc_cool,is_rl]
        d_e = 3 + EDGE_TYPE_DIM    # [Δv,gap, is_self_lane] + type one-hot(10)
        return GymDict({
            "x":          Box(-np.inf, np.inf, shape=(max_nodes, d_x), dtype=np.float32),
            "edge_index": Box(0, max_nodes, shape=(2, max_edges), dtype=np.int64),
            "edge_attr":  Box(-np.inf, np.inf, shape=(max_edges, d_e), dtype=np.float32),
            "num_nodes":  Box(0, max_nodes, shape=(), dtype=np.int32),
            "num_edges":  Box(0, max_edges, shape=(), dtype=np.int32),
            "rl_idx":     Box(0, max_nodes, shape=(self.initial_vehicles.num_rl_vehicles,), dtype=np.int64),
        })
    
    @property
    def action_space(self):
        # (num_rl, 2): accel in [-1,1] 映射到 ±max_accel; lc_intent in [-1,1]
        n_rl = self.initial_vehicles.num_rl_vehicles
        return Box(low=-1.0, high=1.0, shape=(n_rl*2,), dtype=np.float32)

 # ------------- state builder (centralized graph) -------------
    def get_state(self):
        veh_ids = self.k.vehicle.get_ids()
        self.observe = {}
        x, edge_index, edge_attr, rl_idx = self._build_graph_full()
        src, dst = edge_index
        for ridx in rl_idx:   # 遍历 RL 节点 index
            rl_id = veh_ids[ridx]
            # 找出所有 RL → 目标 的边
            mask = (src == ridx)
            neigh_idx = dst[mask]
            neigh_ids = [veh_ids[j] for j in neigh_idx]

            self.observe[rl_id] = np.array(neigh_ids, dtype=object)
        # padding 到 observation_space 的固定尺寸
        max_nodes = self.observation_space["x"].shape[0]
        max_edges = self.observation_space["edge_attr"].shape[0]
        d_x = self.observation_space["x"].shape[1]
        d_e = self.observation_space["edge_attr"].shape[1]

        N = x.shape[0]; E = edge_attr.shape[0]
        x_pad  = np.zeros((max_nodes, d_x), dtype=np.float32); x_pad[:N] = x
        ei_pad = np.zeros((2, max_edges), dtype=np.int64);    ei_pad[:, :E] = edge_index
        ea_pad = np.zeros((max_edges, d_e), dtype=np.float32);ea_pad[:E] = edge_attr

        return {
            "x": x_pad,
            "edge_index": ei_pad,
            "edge_attr": ea_pad,
            "num_nodes": np.int32(N),
            "num_edges": np.int32(E),
            "rl_idx": rl_idx.astype(np.int64),
        }

    # ------------- apply actions (for 8 RL cars at once) -------------
    def _apply_rl_actions(self, actions):
        # actions: [num_rl, 2]
        actions = np.asarray(actions, dtype=np.float32).reshape(n_rl, 2)
        assert actions.shape == (self.k.vehicle.num_rl_vehicles, 2), \
            f"Expect {(self.k.vehicle.num_rl_vehicles,2)}, got {actions.shape}"

        max_accel = self.env_params.additional_params["max_accel"]
        accel = np.clip(actions[:, 0], -1, 1) * max_accel
        lc_intent = np.clip(actions[:, 1], -1, 1)

        rl_ids = self.k.vehicle.get_rl_ids()  # 顺序必须和 rl_idx 对应（get_state里就是按此顺序建的）
        # 离散化 lane-change 意图 + 冷却约束
        dir_disc = np.zeros_like(lc_intent, dtype=int)
        dir_disc[lc_intent >  1/3] =  1
        dir_disc[lc_intent < -1/3] = -1

        cooldown = self.env_params.additional_params["lane_change_duration"]
        for i, vid in enumerate(rl_ids):
            if self.time_counter <= cooldown + self.k.vehicle.get_last_lc(vid):
                dir_disc[i] = 0  # 冷却期禁止变道

        self.k.vehicle.apply_acceleration(rl_ids, acc=accel)
        self.k.vehicle.apply_lane_change(rl_ids, direction=dir_disc)

    # ------------- helpers: build graph -------------
    def _build_graph_full(self):
        """完整信息构图（整张 44 车图）。"""
        veh_ids = self.k.vehicle.get_ids()
        rl_ids  = self.k.vehicle.get_rl_ids()
        N = len(veh_ids)
        L = getattr(self.k.network, "length")()
        if L is None:
            L = self.env_params.additional_params.get("ring_length", 250.0)
        id2idx = {vid: i for i, vid in enumerate(veh_ids)}
        # 节点特征
        s_all = np.array(self.k.vehicle.get_position(veh_ids), dtype=np.float32)
        s_norm  = (s_all / L).astype(np.float32)                                          # [0,1)
        lanes = np.array(self.k.vehicle.get_lane(veh_ids), dtype=np.int64)
        lane_oh = np.eye(NUM_LANES, dtype=np.float32)[np.clip(lanes,0,NUM_LANES-1)]
        v_all = np.array(self.k.vehicle.get_speed(veh_ids), dtype=np.float32) / V_MAX
        # accel 兼容不同 API
        a_all = []
        for vid in veh_ids:
            a = 0.0
            if hasattr(self.k.vehicle, "get_accel"):
                try: a = float(self.k.vehicle.get_accel(vid))
                except: a = 0.0
            elif hasattr(self.k.vehicle, "get_acceleration"):
                try: a = float(self.k.vehicle.get_acceleration(vid))
                except: a = 0.0
            a_all.append(a / A_MAX)
        a_all = np.asarray(a_all, dtype=np.float32)
        # lane-change flag
        last_lc = np.array([getattr(self.k.vehicle, "get_last_lc", lambda _ : 10.0)(vid) for vid in veh_ids], dtype=np.float32)
        lc_cool = (last_lc < 3.0).astype(np.float32)
        is_rl = np.isin(veh_ids, rl_ids).astype(np.float32)

        x = np.concatenate([
            s_norm[:,None],
            lane_oh,
            v_all[:,None], a_all[:,None],
            lc_cool[:,None], is_rl[:,None]
        ], axis=1).astype(np.float32)

        TAG_TO_OFF = {'F1':0, 'F2':1, 'F3':2, 'B1':3, 'B2':4}
        def onehot_lane_type(lane_j, tag):
            oh = np.zeros(EDGE_TYPE_DIM, dtype=np.float32)
            base = int(lane_j) * 5
            oh[base + TAG_TO_OFF[tag]] = 1.0
            return oh
       
        # --- 工具：取 leader/follower 的 'id'（有些 Flow 版本可能返回 (id, dist)）---
        def norm_id(x):
            if x is None: 
                return None
            if isinstance(x, (list, tuple)) and len(x) > 0:
                return x[0]
            return x
        def forward_arc(i, j):
            return float(((s_all[j] - s_all[i]) % L))

        src, dst, eattr = [], [], []
        

        for vid in rl_ids:
            if vid not in id2idx:
                continue
            i = id2idx[vid]

            # 遍历车道顺序（0..NUM_LANES-1）= 从最左到最右
            for ln in range(NUM_LANES):
                # ========= 前向链：F1/F2/F3，gap 用 headways 迭代累加 =========
                # 起点：i 在 ln 车道上的第一辆前车
                cur = vid
                cum_gap = 0.0
                for tag in ['F1', 'F2', 'F3']:
                    leaders  = self.k.vehicle.get_lane_leaders(cur)      # list per lane
                    headways = self.k.vehicle.get_lane_headways(cur)     # list per lane
                    lead_id  = norm_id(leaders[ln]) if (leaders and len(leaders) > ln) else None
                    # 这一步从 cur 到 lead_id 的距离
                    step_gap = headways[ln] if (headways and len(headways) > ln) else float('inf')

                    if lead_id is None:
                        break
                    if not np.isfinite(step_gap):
                        # 兜底：用 s 的环向差
                        j_tmp = id2idx[lead_id]
                        step_gap = forward_arc(id2idx[cur], j_tmp)

                    cum_gap += float(step_gap)  # 物理距离累加
                    j = id2idx.get(lead_id, None)
                    if j is None:
                        break

                    g_norm = float(cum_gap / L)
                    dv     = float(v_all[j] - v_all[i])     # 已归一
                    same   = 1.0 if lanes[i] == lanes[j] else 0.0
                    typ    = onehot_lane_type(lanes[j], tag)
                    feat   = np.concatenate([[g_norm, dv, same], typ]).astype(np.float32)

                    src.append(i); dst.append(j); eattr.append(feat)

                    # 下一层继续以 lead_id 为参考，但仍固定看 ln 车道
                    cur = lead_id

                # ===== 后向 B1/B2：用 get_lane_followers/ tailways 累加 =====
                cur = vid
                cum_gap = 0.0
                for tag in ['B1', 'B2']:
                    followers = self.k.vehicle.get_lane_followers(cur)
                    tailways  = self.k.vehicle.get_lane_tailways(cur)
                    foll_id   = norm_id(followers[ln]) if (followers and len(followers) > ln) else None
                    step_gap  = tailways[ln] if (tailways and len(tailways) > ln) else float('inf')

                    if foll_id is None:
                        break
                    if not np.isfinite(step_gap):
                        # 兜底：用环向前向差 (i->j) 的逆转：prev -> foll
                        j_tmp = id2idx[foll_id]
                        # 从 cur 到 foll 的“后向”距离 = (s_cur - s_foll) 正向环距
                        step_gap = forward_arc(id2idx[foll_id], id2idx[cur])

                    cum_gap += float(step_gap)
                    j = id2idx.get(foll_id, None)
                    if j is None:
                        break

                    g_norm = float(cum_gap / L)
                    dv     = float(v_all[j] - v_all[i])
                    same   = 1.0 if lanes[i] == lanes[j] else 0.0
                    typ    = onehot_lane_type(lanes[j], tag)
                    feat   = np.concatenate([[g_norm, dv, same], typ]).astype(np.float32)

                    src.append(i); dst.append(j); eattr.append(feat)

                    cur = foll_id

        edge_index = (np.stack([np.asarray(src, np.int64), np.asarray(dst, np.int64)], axis=0)
                    if src else np.zeros((2, 0), dtype=np.int64))
        d_e = 3 + EDGE_TYPE_DIM  # [gap_norm, dv_norm, is_same_lane] + type_onehot
        edge_attr  = (np.stack(eattr, axis=0).astype(np.float32)
                    if eattr else np.zeros((0, d_e), dtype=np.float32))

        rl_idx = np.array([id2idx[rid] for rid in rl_ids if rid in id2idx], dtype=np.int64)
        return x, edge_index, edge_attr, rl_idx
    

    def additional_command(self):
        """高亮 RL 车辆和它们的邻居。"""
        for rl_id in self.k.vehicle.get_rl_ids():

            neighs = self.observe.get(rl_id, [])
            for vid in neighs:
                try:
                    self.k.vehicle.set_observed(vid)
                except Exception:
                    pass
