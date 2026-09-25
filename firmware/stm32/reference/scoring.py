#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标选择打分函数 — 参考实现
===========================

对应文档：docs/07-软件架构与决策策略.md §4

这是**整套策略的核心**。它必须反映赛题的**层级式平局判据**：

    总分 (Σ分值)        <- 第一优先
      → 进球总数 (枚数)  <- 第二判据，有独立价值
        → 帅/将归属      <- 第三判据，但是支配项
          → 60 秒金球

运行自测：
    python firmware/stm32/reference/scoring.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 棋子与权重
# ==========================================================================

class Piece:
    """一枚待选目标棋子。坐标单位 mm（场地坐标系）。"""

    def __init__(self, x, y, value=None, is_king=False, color=None,
                 known=True, name=""):
        self.x = x
        self.y = y
        self.value = value        # 分值 1-10；None = 未知（未读字）
        self.is_king = is_king
        self.color = color        # 1=红 2=黑
        self.known = known        # 是否已识别
        self.name = name

    def __repr__(self):
        v = f"{self.value}分" if self.value is not None else "未读"
        k = "帅/将" if self.is_king else ""
        return f"<{self.name or '棋子'} ({self.x},{self.y}) {v}{k}>"


class Weights:
    """
    打分权重。初值来自 docs/07 §4.2，之后按实测调。
    """

    def __init__(self):
        self.w_value = 1.0        # 分值（第一判据）
        self.w_count = 3.0        # 枚数（第二判据，独立价值）
        self.w_king = 50.0        # 帅/将支配项
        self.w_dist = 0.02        # 距离惩罚（每 mm）
        self.w_corridor = 15.0    # 走廊占用（每个棋子）
        self.w_risk = 200.0       # 送分风险（每枚在对方半场）
        self.w_opponent = 10.0    # 对手接近

        # 未知分值时的估计（未读字的棋子按期望值估）
        self.unknown_value_prior = 4.0


# ==========================================================================
# 场地模型
# ==========================================================================

class Field:
    """
    场地几何。

    ⚠️ 球门位置依赖待确认项 Q4（见 docs/09）。这里按"球门在 2m 端、
       长轴为 3m"的倾向解读实现。
    """

    def __init__(self, length_mm=3000.0, width_mm=2000.0,
                 wall_mm=80.0, car_half_width_mm=75.0,
                 push_clearance_mm=5.0):
        self.L = length_mm
        self.W = width_mm
        self.wall = wall_mm
        self.car_half = car_half_width_mm
        self.push_clearance = push_clearance_mm

    def own_goal(self, side=0):
        """己方球门中心。side=0 表示己方球门在 x=0 端。"""
        return (0.0 if side == 0 else self.L, self.W / 2.0)

    def is_in_own_half(self, p, side=0):
        half = self.L / 2.0
        return p.x < half if side == 0 else p.x > half

    def is_wall_hugging(self, p):
        """棋子是否贴侧墙（推板难以罩住）"""
        return (p.y - self.car_half < self.wall + self.push_clearance or
                p.y + self.car_half > self.W - self.wall - self.push_clearance)

    def distance_to_own_goal(self, p, side=0):
        gx, gy = self.own_goal(side)
        return math.hypot(p.x - gx, p.y - gy)


# ==========================================================================
# 打分函数
# ==========================================================================

class Scorer:

    def __init__(self, field: Field, weights: Weights, own_side=0):
        self.field = field
        self.w = weights
        self.own_side = own_side

    # ---- 辅助：走廊占用 ----
    def corridor_congestion(self, p, all_pieces, corridor_width_mm=180.0):
        """
        统计"棋子 -> 己方球门"这条推进走廊里的**其它**棋子数量。

        为什么重要：走廊里有别的棋子会被撞散，可能把高分棋子撞歪到对面球门。
        """
        gx, gy = self.field.own_goal(self.own_side)
        dx = gx - p.x
        dy = gy - p.y
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0
        ux, uy = dx / length, dy / length

        count = 0
        for q in all_pieces:
            if q is p:
                continue
            # q 相对 p 的向量
            vx, vy = q.x - p.x, q.y - p.y
            proj = vx * ux + vy * uy              # 沿走廊方向的分量
            if not (0.0 < proj < length):         # 必须在 p 与球门之间
                continue
            perp = abs(-vx * uy + vy * ux)        # 垂直走廊方向的距离
            if perp < corridor_width_mm / 2.0:
                count += 1
        return count

    # ---- 辅助：送分风险 ----
    def risk_to_opponent_goal(self, p):
        """
        ★ 推进这枚棋子的"把分送给对方"的风险。

        赛题是"把棋子推入**己方**球门区得分"。所以：
          - 若棋子已在对方半场，推向己方球门要横穿整个场地
          - 推程越长、越靠近对方球门，中途失控把它拱进对面门的风险越大
          - 一次失误就是 -2x 分值摆动（对方得分 = 我方相对损失）

        返回 0.0（无风险）到 1.0（极高风险）。
        """
        if self.field.is_in_own_half(p, self.own_side):
            return 0.0

        # 在对方半场：距离己方球门越远，风险越高
        d = self.field.distance_to_own_goal(p, self.own_side)
        max_d = math.hypot(self.field.L, self.field.W)
        return min(d / max_d, 1.0)

    # ---- 辅助：贴墙可行性 ----
    def wall_hug_penalty(self, p):
        """
        贴墙棋子的惩罚。

        见 docs/02 §3.2：贴墙时推板容错窗口很窄。若还不在己方半场，
        贴墙纵推会把它推向对方球门 = 送分，必须跳过。
        """
        if not self.field.is_wall_hugging(p):
            return 0.0
        if self.field.is_in_own_half(p, self.own_side):
            return 1.0        # 可行但较难
        return 100.0          # ★ 绝对不做：会送分

    # ---- 主打分 ----
    def score(self, p, all_pieces, opponent_pos=None, opponent_radius_mm=400.0):
        """
        返回 (总分, 分项字典)。分数越高越优先。
        """
        w = self.w

        # 分值项：未读字时用先验期望值
        value = p.value if p.value is not None else w.unknown_value_prior
        s_value = w.w_value * value

        # 枚数项（第二判据，独立价值）
        s_count = w.w_count

        # 帅/将支配项
        s_king = w.w_king if p.is_king else 0.0

        # 距离惩罚
        d = self.field.distance_to_own_goal(p, self.own_side)
        s_dist = -w.w_dist * d

        # 走廊占用
        cong = self.corridor_congestion(p, all_pieces)
        s_corr = -w.w_corridor * cong

        # 送分风险
        risk = self.risk_to_opponent_goal(p)
        s_risk = -w.w_risk * risk

        # 贴墙
        hug = self.wall_hug_penalty(p)
        s_hug = -30.0 * hug

        # 对手接近
        s_opp = 0.0
        if opponent_pos is not None:
            dop = math.hypot(p.x - opponent_pos[0], p.y - opponent_pos[1])
            if dop < opponent_radius_mm:
                s_opp = -w.w_opponent * (1.0 - dop / opponent_radius_mm)

        total = (s_value + s_count + s_king + s_dist +
                 s_corr + s_risk + s_hug + s_opp)

        return total, {
            "分值": s_value, "枚数": s_count, "帅将": s_king,
            "距离": s_dist, "走廊": s_corr, "送分风险": s_risk,
            "贴墙": s_hug, "对手": s_opp,
        }

    # ---- 选择 ----
    def select(self, pieces, opponent_pos=None):
        """
        返回按优先级排序的 (棋子, 总分, 分项) 列表，并过滤掉"绝不做"的。
        """
        scored = []
        for p in pieces:
            total, detail = self.score(p, pieces, opponent_pos)
            # 过滤：贴墙且在对方半场（会送分）
            if self.wall_hug_penalty(p) >= 100.0:
                continue
            scored.append((p, total, detail))
        scored.sort(key=lambda t: -t[1])
        return scored

    # ---- 帅/将的"可达 + 不送分"判定 ----
    def king_is_worth_chasing(self, king: Piece, all_pieces,
                              max_risk=0.35):
        """
        ★ 帅/将的正确表述不是"开局必抢"，而是"可达且不送分时必抢"。

        见 docs/07 §4.3：鲁莽直冲可能送 10 分给对面，比不拿更糟。
        """
        if not king.is_king:
            return False, "不是帅/将"
        risk = self.risk_to_opponent_goal(king)
        if risk > max_risk:
            return False, f"送分风险过高 ({risk:.2f} > {max_risk})"
        if self.wall_hug_penalty(king) >= 100.0:
            return False, "贴墙且在对方半场"
        return True, "可达且风险可控"


# ==========================================================================
# 降级策略 B
# ==========================================================================

def select_degraded(pieces, field: Field, own_side=0):
    """
    降级策略 B：不读字、不做分值识别。

    目标 = 离己方球门最近的棋子，优先抢「进球总数」（平局第一判据）。

    见 docs/07 §5。**这是必须有的兜底** ——
    nncase 可能失败，棋子也可能背面朝上。
    """
    candidates = [p for p in pieces
                  if field.is_in_own_half(p, own_side)
                  and field.wall_hug_penalty(p) < 100.0]
    candidates.sort(key=lambda p: field.distance_to_own_goal(p, own_side))
    return candidates


Field.wall_hug_penalty = lambda self, p: 100.0 if (
    self.is_wall_hugging(p) and not self.is_in_own_half(p)) else 0.0


# ==========================================================================
# 自测
# ==========================================================================

def selftest():
    print("=" * 78)
    print("目标打分函数 自测")
    print("=" * 78)

    field = Field()
    w = Weights()
    scorer = Scorer(field, w, own_side=0)     # 己方球门在 x=0 端

    # ---- 1. 帅/将支配项 ----
    print("\n[1] 帅/将的支配作用（同位置、同分值，只有 is_king 不同）")
    # 注意：必须放在**己方半场**，否则两者都会吃送分惩罚，
    #       掩盖了帅/将支配项的差异。
    king = Piece(800, 1000, value=10, is_king=True, name="帅")
    chariot = Piece(800, 1000, value=10, is_king=False, name="车")
    pieces = [king, chariot]
    sk, _ = scorer.score(king, pieces)
    sc, _ = scorer.score(chariot, pieces)
    print(f"  帅（10分，己方半场）= {sk:.1f}")
    print(f"  车（10分，同位置）  = {sc:.1f}")
    print(f"  帅比车高 {sk-sc:.1f} 分  "
          f"{'通过' if sk > sc + 40 else '失败'}")
    assert sk > sc + 40

    # ---- 2. 送分风险 ----
    print("\n[2] 送分风险：对方半场的棋子应被大幅降权")
    near = Piece(400, 1000, value=10, name="己方半场10分")
    far = Piece(2600, 1000, value=10, name="对方半场10分")
    pieces = [near, far]
    sn, dn = scorer.score(near, pieces)
    sf, df = scorer.score(far, pieces)
    print(f"  己方半场(400,1000)  = {sn:.1f}   风险项 {dn['送分风险']:.1f}")
    print(f"  对方半场(2600,1000) = {sf:.1f}   风险项 {df['送分风险']:.1f}")
    print(f"  对方半场被降权 {sn-sf:.1f} 分  "
          f"{'通过' if sn > sf + 100 else '失败'}")
    assert sn > sf + 100

    # ---- 3. 贴墙且在对方半场 -> 绝对排除 ----
    print("\n[3] 贴墙 + 对方半场 -> 必须排除（否则送分）")
    hug_far = Piece(2600, 60, value=10, name="贴墙对方半场")
    pieces = [hug_far, near]
    sel = scorer.select(pieces)
    names = [p.name for p, _, _ in sel]
    print(f"  候选: {[p.name for p in pieces]}")
    print(f"  选中: {names}")
    ok = "贴墙对方半场" not in names
    print(f"  贴墙+对方半场被排除  {'通过' if ok else '失败'}")
    assert ok

    # ---- 4. 走廊占用 ----
    print("\n[4] 走廊占用：推进路线上有其它棋子应降权")
    # 注意：走廊是"棋子 -> 己方球门"的**连线**，所以阻挡棋子必须放在
    #       这条线上，不能随便放在旁边。
    #       本场景：blocked(600,400) -> 球门(0,1000)，线中点 = (300,700)
    blocked = Piece(600, 400, value=8, name="走廊有阻挡")
    blocker = Piece(300, 700, value=1, name="挡在走廊线上")
    clear = Piece(600, 1200, value=8, name="走廊干净")
    # clear 的走廊是 (600,1200)->(0,1000)，中点是 (300,1100)，
    # 而 blocker 在 (300,700)，远离这条线。
    all_p = [clear, blocked, blocker]
    s_clear, d_clear = scorer.score(clear, all_p)
    s_block, d_block = scorer.score(blocked, all_p)
    print(f"  走廊干净   = {s_clear:.1f}   走廊项 {d_clear['走廊']:.1f}")
    print(f"  走廊有阻挡 = {s_block:.1f}   走廊项 {d_block['走廊']:.1f}")
    ok = d_block["走廊"] < 0 and d_clear["走廊"] == 0
    print(f"  阻挡被降权、干净的不受影响  {'通过' if ok else '失败'}")
    assert ok, f"走廊项: clear={d_clear['走廊']}, block={d_block['走廊']}"

    # ---- 5. 帅/将"可达且不送分"判定 ----
    print("\n[5] 帅/将的'可达且不送分'判定")
    king_near = Piece(800, 1000, value=10, is_king=True, name="帅(己方半场)")
    king_far = Piece(2600, 1000, value=10, is_king=True, name="帅(对方半场)")
    for k in [king_near, king_far]:
        worth, why = scorer.king_is_worth_chasing(k, [k])
        print(f"  {k.name:<16} -> {'值得抢' if worth else '不抢'}: {why}")
    w1, _ = scorer.king_is_worth_chasing(king_near, [king_near])
    w2, _ = scorer.king_is_worth_chasing(king_far, [king_far])
    print(f"  己方半场帅值得抢、对方半场帅不抢  "
          f"{'通过' if w1 and not w2 else '失败'}")
    assert w1 and not w2

    # ---- 6. 完整排序 ----
    print("\n[6] 完整场景排序")
    pieces = [
        Piece(500, 1000, value=10, is_king=True, name="帅"),
        Piece(700, 800, value=9, name="车"),
        Piece(900, 1200, value=5, name="马"),
        Piece(400, 300, value=1, name="兵"),
        Piece(2500, 1500, value=10, name="对方半场炮"),
        Piece(100, 100, value=5, name="贴墙对方区"),
    ]
    sel = scorer.select(pieces)
    print(f"  {'棋子':<12} {'总分':>8}   分项")
    print("  " + "-" * 68)
    for p, total, d in sel:
        parts = " ".join(f"{k}={v:.0f}" for k, v in d.items() if abs(v) > 0.5)
        print(f"  {p.name:<12} {total:>8.1f}   {parts}")

    # 验证：帅应该排第一（己方半场、可达）
    print(f"\n  第一名 = {sel[0][0].name}  "
          f"{'通过' if sel[0][0].name == '帅' else '失败'}")
    assert sel[0][0].name == "帅"

    # 验证：对方半场的高分棋子不应排在前列
    idx_far = next(i for i, (p, _, _) in enumerate(sel) if p.name == "对方半场炮")
    print(f"  对方半场炮排名 = 第 {idx_far+1} 位（共 {len(sel)}）  "
          f"{'通过' if idx_far >= 3 else '失败'}")
    assert idx_far >= 3

    # ---- 7. 降级策略 B ----
    print("\n[7] 降级策略 B（不读字，按距离抢枚数）")
    pieces_b = [
        Piece(500, 1000, value=None, name="A"),
        Piece(300, 1000, value=None, name="B"),
        Piece(800, 1000, value=None, name="C"),
        Piece(2500, 1000, value=None, name="D(对方半场)"),
    ]
    ordered = select_degraded(pieces_b, field, own_side=0)
    names = [p.name for p in ordered]
    print(f"  场地: 己方球门在 x=0 端")
    print(f"  候选: {[p.name for p in pieces_b]}")
    print(f"  排序: {names}")
    ok = names == ["B", "A", "C"]
    print(f"  按离己方球门距离升序、排除对方半场  {'通过' if ok else '失败'}")
    assert ok

    # ---- 8. 未知分值的先验 ----
    print("\n[8] 未读字棋子的先验估计")
    unknown = Piece(600, 1000, value=None, name="未读")
    known1 = Piece(600, 1000, value=1, name="已知1分")
    known9 = Piece(600, 1000, value=9, name="已知9分")
    su, _ = scorer.score(unknown, [unknown])
    s1, _ = scorer.score(known1, [known1])
    s9, _ = scorer.score(known9, [known9])
    print(f"  未读字（先验 {w.unknown_value_prior} 分） = {su:.1f}")
    print(f"  已知 1 分 = {s1:.1f}")
    print(f"  已知 9 分 = {s9:.1f}")
    ok = s1 < su < s9
    print(f"  未读字介于低分与高分之间  {'通过' if ok else '失败'}")
    assert ok

    print("\n" + "=" * 78)
    print("全部自测通过。")
    print("=" * 78)
    print("\n提示：权重的初值来自 docs/07 §4.2，需按实测调整。")
    print("      改权重后重跑本脚本，确认排序仍符合策略意图。")


if __name__ == "__main__":
    selftest()
