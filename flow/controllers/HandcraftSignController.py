from flow.controllers.base_controller import BaseController
import numpy as np

class HandcraftSignController(BaseController):
    """
    A simple sign-based controller:

        accel_raw = (0.75 * sign(handcraft - v_rl) + 1) / 2
        accel = (accel_raw * 2 - 1) * (max_accel if accel_raw > 0.5 else max_decel)

    - handcraft: target desired speed (scalar float)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 handcraft=4.36,
                 max_accel=0.5,
                 max_decel=0.5,
                 **kwargs):

        super().__init__(veh_id, car_following_params, **kwargs)

        self.handcraft = float(handcraft)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)

class HandcraftSignController(BaseController):
    """
    Sign-based controller with warmup IDM + global per-step CSV logging.

    控制律：
        accel_raw = (0.75 * sign(handcraft - v_rl) + 1) / 2
        accel = (accel_raw * 2 - 1) * (max_accel if accel_raw > 0.5 else max_decel)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 handcraft=10.0,
                 max_accel=1.0,
                 max_decel=1.0,
                 **kwargs):

        super().__init__(veh_id, car_following_params, **kwargs)

        self.handcraft = float(handcraft)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)

    def get_accel(self, env):

        kv = env.k.vehicle
        vid = self.veh_id

        # ==============================================================
        # 📝 Step-level 全局 CSV 记录（所有车辆） - 每个 step 只写一次
        # ==============================================================
# ==============================================================
        import csv, os

        # 日志目录
        save_dir = "/home/spei/flow_logs"
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "trajectory_log_handcraft2av.csv")

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
        # 🚗 Warm-up 阶段：使用 IDM
        # ==============================================================
        warmup_steps = getattr(env.env_params, "warmup_steps", 0)
        if getattr(env, "step_counter", 0) < warmup_steps:

            lead = kv.get_leader(vid)
            v = kv.get_speed(vid) or 0.0
            v_lead = kv.get_speed(lead) if lead else v
            headway = kv.get_headway(vid) or 1e9

            v0, s0, T, a, b = 30, 2, 1.0, 1.5, 2.0
            s_star = s0 + max(0, v*T + v*(v - v_lead) / (2*np.sqrt(a*b)))

            a_idm = a * (1 - (v / v0)**4 - (s_star / headway)**2)

            return float(np.clip(a_idm, -self.max_decel, self.max_accel))

        # ==============================================================
        # 🚀 正式控制阶段：sign-based controller
        # ==============================================================
        v = kv.get_speed(vid) or 0.0

        # sign(handcraft - v)
        diff = self.handcraft - v
        sgn = np.sign(diff) if abs(diff) > 1e-6 else 0.0

        # accel_raw ∈ [0, 1]
        accel_raw = (0.75 * sgn + 1) / 2

        # [-1, 1]
        core = accel_raw * 2 - 1

        if accel_raw > 0.5:
            # accelerate
            a_cmd = core * self.max_accel
        else:
            # brake
            a_cmd = core * self.max_decel

        return float(a_cmd)
