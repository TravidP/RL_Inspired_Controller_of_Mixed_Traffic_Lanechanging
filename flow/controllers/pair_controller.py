from flow.controllers.base_controller import BaseController
import numpy as np
from scipy.optimize import fsolve

def v_eq_max_function(v, *args):
    """求解环路在给定车辆密度下的平衡最大速度"""
    num_vehicles, length = args
    # 假设车长 5 m
    s_eq_max = (length - num_vehicles * 5) / (num_vehicles - 1)

    v0 = 30
    s0 = 2
    tau = 1
    gamma = 4

    return s_eq_max - (s0 + v * tau) * (1 - (v / v0) ** gamma) ** -0.5


def _signed_ring_delta(x_target, x_self, L):
    """
    返回在环形上目标相对自身的有符号最短距离 (范围 [-L/2, L/2))
    正值表示目标在前方，负值表示目标在后方。
    """
    delta = (x_target - x_self + L / 2) % L - L / 2
    return delta




class PairAlignRuleController(BaseController):
    """
    两辆 AV 速度同步 + 距离对齐 控制器
    支持自动计算环路稳态速度 v_eq_max 作为目标巡航速度
    """
    def __init__(self,
                 veh_id,
                 car_following_params,
                 pair_id,
                 ring_length=250.0,
                 num_vehicles_total=44,    # 用于求稳态速度
                 k_sync=0.8,
                 k_pair=0.01,
                 k_front=0.3,
                 k_back=0.15,
                 k_v=0.3,
                 safe_gap=7,
                 hard_brake=-2.0,
                 max_accel=0.5,
                 max_decel=0.5,
                 **kwargs):
        super().__init__(veh_id, car_following_params, **kwargs)
        self.pair_id = pair_id
        self.L = float(ring_length)
        self.k_sync = k_sync
        self.k_pair = k_pair
        self.k_front = k_front
        self.k_back = k_back
        self.k_v = k_v
        self.safe_gap = safe_gap
        self.hard_brake = float(hard_brake)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)

        # ===== 🔹自动计算环路稳态速度 v_eq_max =====
        try:
            v_guess = 10
            # 默认每车道数量约为总车数/2
            num_per_lane = max(3, int(num_vehicles_total / 2))
            v_eq_max = fsolve(v_eq_max_function, np.array(v_guess),
                              args=(num_per_lane, ring_length))[0]
            self.v_star = float(v_eq_max)*1.2
            print(f"[{veh_id}] calculated v_star = {self.v_star:.2f} m/s")
        except Exception as e:
            print(f"[{veh_id}] failed to compute v_eq_max: {e}")
            self.v_star = 12.0  # fallback
        # =========================================

    # def get_accel(self, env):
    #     kv = env.k.vehicle
    #     vid = self.veh_id

    #     v = kv.get_speed(vid) or 0.0
    #     x = kv.get_x_by_id(vid) or 0.0

    #     v_p, d_pair = 0.0, 0.0
    #     if self.pair_id in kv.get_ids():
    #         v_p = kv.get_speed(self.pair_id) or 0.0
    #         x_p = kv.get_x_by_id(self.pair_id) or 0.0
    #         d_pair = _signed_ring_delta(x_p, x, self.L)

    #     lead = kv.get_leader(vid)
    #     v_f = kv.get_speed(lead) if lead else v
    #     headway = kv.get_headway(vid) or 1e9

    #     foll = kv.get_follower(vid)
    #     v_b = kv.get_speed(foll) if foll else v

    #     a_cmd = 0.0
    #     a_cmd += self.k_sync * (v_p - v)
    #     a_cmd += self.k_pair * (-d_pair)
    #     a_cmd += self.k_front * (v_f - v)
    #     a_cmd += self.k_back * (v - v_b)
    #     a_cmd += self.k_v * (self.v_star - v)   # 使用自动求出的 v_star

    #     if headway < self.safe_gap:
    #         a_cmd = min(a_cmd, self.hard_brake)

    #     a_cmd = np.clip(a_cmd, -self.max_decel, self.max_accel)
    #     return float(a_cmd)
    # def get_accel(self, env):
    #     kv = env.k.vehicle
    #     vid = self.veh_id

    #     # ==============================================================
    #     # 🚗 Warm-up 阶段延迟接管：在前 N 步仅执行 IDM 控制逻辑
    #     # ==============================================================
    #     warmup_steps = getattr(env.env_params, "warmup_steps", 0)
    #     if getattr(env, "step_counter", 0) < warmup_steps:
    #         lead = kv.get_leader(vid)
    #         v = kv.get_speed(vid) or 0.0
    #         v_lead = kv.get_speed(lead) if lead else v
    #         headway = kv.get_headway(vid) or 1e9

    #         # --- 简单 IDM 模型 ---
    #         v0, s0, T, a, b = 30, 2, 1.0, 1.5, 2.0
    #         s_star = s0 + max(0, v * T + v * (v - v_lead) / (2 * np.sqrt(a * b)))
    #         a_idm = a * (1 - (v / v0) ** 4 - (s_star / headway) ** 2)

    #         return float(np.clip(a_idm, -self.max_decel, self.max_accel))

    #     # ==============================================================
    #     # 🚀 Warm-up 结束后使用 Pair-Align 控制策略
    #     # ==============================================================
    #     v = kv.get_speed(vid) or 0.0
    #     x = kv.get_x_by_id(vid) or 0.0

    #     v_p, d_pair = 0.0, 0.0
    #     if self.pair_id in kv.get_ids():
    #         v_p = kv.get_speed(self.pair_id) or 0.0
    #         x_p = kv.get_x_by_id(self.pair_id) or 0.0
    #         d_pair = _signed_ring_delta(x_p, x, self.L)

    #     # lead = kv.get_leader(vid)
    #     # v_f = kv.get_speed(lead) if lead else v
    #     # headway = kv.get_headway(vid) or 1e9

    #     # foll = kv.get_follower(vid)
    #     # v_b = kv.get_speed(foll) if foll else v

    #     # --- 控制律：速度同步 + 位置配对 + 稳流 + 巡航 ---
    #     a_cmd = 0.0
    #     a_cmd += self.k_sync * (v_p - v)
    #     a_cmd += self.k_pair * (-d_pair)
    #     # a_cmd += self.k_front * (v_f - v)
    #     # a_cmd += self.k_back  * (v - v_b)
    #     a_cmd += self.k_v * (self.v_star - v)   # 使用自动求出的 v_star

    #     # --- 安全机制：前向距离过小时强制减速 ---
    #     # if headway < self.safe_gap:
    #     #     a_cmd = min(a_cmd, self.hard_brake)

    #     # --- 饱和限制 ---
    #     a_cmd = np.clip(a_cmd, -self.max_decel, self.max_accel)
    #     print(a_cmd)
    #     return float(a_cmd)
    def get_accel(self, env):
                # ==============================================================
        # 📝 Step-level 全局 CSV 记录（所有车辆） - 只在 step 开始时记录一次
        # ==============================================================
        import csv, os

        kv = env.k.vehicle
        vid = self.veh_id
        # 日志目录
        save_dir = "/home/spei/flow_logs"
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "trajectory_log_paired_controller.csv")

        # 当前 step
        step = getattr(env, "step_counter", 0)

        # 防止重复写入，同一 step 全部 controller 共用
        flag_attr = "_csv_logged_step"
        last_logged = getattr(env, flag_attr, -1)

        if last_logged != step:
            setattr(env, flag_attr, step)

            first_write = not os.path.exists(save_path)
            ids = kv.get_ids()

            # ==== 内嵌 get_global_position()：专门适配 Flow RingNetwork ====
            def get_global_position(pid):
                """
                Returns absolute 0~ring_length position for Flow's 4-edge ring:
                
                bottom → right → top → left → bottom
                """
                lane_pos = kv.get_position(pid)
                # print(lane_pos)
                if lane_pos is None:
                    return 0.0

                edge = kv.get_edge(pid)
                ring_length = 250
                junction_length = 0.1  # ring.py default junction length
                # print(edge)
                # Four edges offsets
                if edge == "bottom":
                    edge_start = 0.0
                elif edge == "right":
                    edge_start = 0.25 * ring_length + junction_length
                elif edge == "top":
                    edge_start = 0.5 * ring_length + 2 * junction_length
                elif edge == "left":
                    edge_start = 0.75 * ring_length + 3 * junction_length
                else:
                    # fallback for internal edges like ":right_0"
                    return lane_pos
                
                return edge_start + lane_pos  # absolute position (no mod)

            # ==============================================================
            with open(save_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=[
                    "id", "type", "step", "lane_id", "lane_position_global", "speed"
                ])

                if first_write:
                    writer.writeheader()

                for pid in ids:
                    vtype = "rl" if "rl" in pid else "human"
                    lane_id = kv.get_lane(pid) or ""

                    # ⭐ 使用绝对 0~250 的位置
                    abs_pos = get_global_position(pid)
                    # print(abs_pos)
                    # 修复 speed 获取：必须使用 pid（你原来写的是 vid = bug）
                    speed = kv.get_speed(pid) or 0.0

                    writer.writerow({
                        "id": pid,
                        "type": vtype,
                        "step": step,
                        "lane_id": lane_id,
                        "lane_position_global": abs_pos,
                        "speed": speed,
                    })



        # ==============================================================
        # 🚗 Warm-up 阶段延迟接管：在前 N 步仅执行 IDM 控制逻辑
        # ==============================================================
        warmup_steps = getattr(env.env_params, "warmup_steps", 0)
        if getattr(env, "step_counter", 0) < warmup_steps:
            lead = kv.get_leader(vid)
            v = kv.get_speed(vid) or 0.0
            v_lead = kv.get_speed(lead) if lead else v
            headway = kv.get_headway(vid) or 1e9

            # --- 简单 IDM 模型 ---
            v0, s0, T, a, b = 30, 2, 1.0, 1.5, 2.0
            s_star = s0 + max(0, v * T + v * (v - v_lead) / (2 * np.sqrt(a * b)))
            a_idm = a * (1 - (v / v0) ** 4 - (s_star / headway) ** 2)

            return float(np.clip(a_idm, -self.max_decel, self.max_accel))

        # ==============================================================
        # 🚀 Warm-up 结束后使用 Pair-Formation + Cruise 控制策略
        # ==============================================================
        v = kv.get_speed(vid) or 0.0
        x = kv.get_x_by_id(vid) or 0.0

        # 读取配对车辆状态
        v_p, d_pair = 0.0, 0.0
        if self.pair_id in kv.get_ids():
            v_p = kv.get_speed(self.pair_id) or 0.0
            x_p = kv.get_x_by_id(self.pair_id) or 0.0
            d_pair = _signed_ring_delta(x_p, x, self.L)

        # ==============================================================
        # ⚙️ 三阶段控制逻辑
        # ==============================================================
        # (1) Formation阶段：位置差大，主动对齐位置
        if abs(d_pair) > 5.0:
            a_cmd = self.k_pair * (d_pair) 
        # (2) Synchronization阶段：位置差小但速度差较大
        elif abs(v_p - v) > 0.5:
            a_cmd = self.k_sync * (v_p - v)
        # (3) Cruise阶段：位置与速度都稳定，保持目标巡航速度
        else:
            # 加一个 baseline acceleration 避免死区（静止不动）
            a_cmd = self.k_v * (self.v_star - v)
            if v < 0.5:
                a_cmd += 0.3  # 主动起步项

        # ==============================================================
        # 🔒 饱和限制
        # ==============================================================
        a_cmd = np.clip(a_cmd, -self.max_decel, self.max_accel)

        # Debug打印方便你观察每阶段状态
        # print(f"[{vid}] v={v:.2f}, v_p={v_p:.2f}, d_pair={d_pair:.2f}, a={a_cmd:.2f}")

        return float(a_cmd)

# class PairFormationCruiseController(BaseController):
#     """
#     双AV配对 + 共速控制器
#     目标：两车先并排（位置对齐） -> 然后共同巡航 (v_star)
#     """
#     def __init__(self,
#                  veh_id,
#                  car_following_params,
#                  pair_id,
#                  ring_length=250.0,
#                  num_vehicles_total=44,
#                  v_star=None,
#                  k_d=0.08,    # 距离对齐增益
#                  k_v=0.6,     # 速度同步增益
#                  k_c=0.4,     # 巡航速度增益
#                  safe_gap=2.0,
#                  hard_brake=-2.0,
#                  max_accel=1.0,
#                  max_decel=1.0,
#                  **kwargs):
#         super().__init__(veh_id, car_following_params, **kwargs)
#         self.pair_id = pair_id
#         self.L = float(ring_length)
#         self.k_d = k_d
#         self.k_v = k_v
#         self.k_c = k_c
#         self.safe_gap = safe_gap
#         self.hard_brake = hard_brake
#         self.max_accel = max_accel
#         self.max_decel = max_decel

#         # 自动求稳态速度
#         if v_star is None:
#             try:
#                 from scipy.optimize import fsolve
#                 v_guess = 8
#                 num_per_lane = max(3, int(num_vehicles_total / 2))
#                 v_eq_max = fsolve(v_eq_max_function, np.array(v_guess),
#                                   args=(num_per_lane, ring_length))[0]
#                 self.v_star = float(v_eq_max)
#                 print(f"[{veh_id}] computed v_star = {self.v_star:.2f} m/s")
#             except Exception as e:
#                 print(f"[{veh_id}] fallback v_star, error: {e}")
#                 self.v_star = 8.0
#         else:
#             self.v_star = v_star

#     def get_accel(self, env):
#         kv = env.k.vehicle
#         vid = self.veh_id

#         v = kv.get_speed(vid) or 0.0
#         x = kv.get_x_by_id(vid) or 0.0

#         # 获取配对车辆状态
#         if self.pair_id not in kv.get_ids():
#             return 0.0
#         v_p = kv.get_speed(self.pair_id) or 0.0
#         x_p = kv.get_x_by_id(self.pair_id) or 0.0

#         # --- 环形相对位置差（正：配对车在前）---
#         d_pair = _signed_ring_delta(x_p, x, self.L)

#         # --- 三阶段控制逻辑 ---
#         if abs(d_pair) > 5.0:   # 阶段1：配对阶段
#             a_cmd = self.k_d * (-d_pair) + 0.2 * (v_p - v)

#         elif abs(v_p - v) > 0.5:  # 阶段2：速度同步
#             a_cmd = self.k_v * (v_p - v)

#         else:  # 阶段3：共同巡航
#             a_cmd = self.k_c * (self.v_star - v)

#         # --- 安全约束（防追尾）---
#         lead = kv.get_leader(vid)
#         headway = kv.get_headway(vid) or 1e9
#         if headway < self.safe_gap:
#             a_cmd = min(a_cmd, self.hard_brake)
#         print(a_cmd)
#         return float(np.clip(a_cmd, -self.max_decel, self.max_accel))
