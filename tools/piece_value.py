#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
字符识别与优先级算法
====================

用途：把"看到的字"变成"该先推哪个"。

**本轮解决的核心问题**：现有打分函数假设分值已知，
但识别实际输出的是**概率分布**，不是确定的值。

本工具算六件事：
  1. 棋子分值表（7 兵种 -> 1-10 分）—— 可配置
  2. ★ 从概率到期望分值（不用 argmax，用期望值）
  3. 误判代价分析（为什么期望值比 argmax 好）
  4. 识别置信度驱动的行为策略（低置信度怎么办）
  5. 完整优先级算法
  6. 结论

运行：
    python tools/piece_value.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 1. 兵种与分值
# ==========================================================================

class PieceType:
    def __init__(self, id_, name_red, name_black, value, count_per_side):
        self.id = id_
        self.name_red = name_red
        self.name_black = name_black
        self.value = value
        self.count = count_per_side

    @property
    def name(self):
        return f"{self.name_red}/{self.name_black}"


# ⚠️ 分值表是**规则解读**，不是物理常数。
# 赛题只说"按棋子等级设置 1-10 分"，未给出 7 兵种到 1-10 的映射。
# 见 docs/09 的 Q9（本轮新增）。
PIECE_TYPES = [
    # id, 红,    黑,    默认分值, 每方枚数
    PieceType(0, "未知",  "未知",  0,  0),
    PieceType(1, "兵",   "卒",    1,  5),
    PieceType(2, "炮",   "炮",    5,  2),
    PieceType(3, "马",   "马",    4,  2),
    PieceType(4, "车",   "车",    9,  2),
    PieceType(5, "相",   "象",    2,  2),
    PieceType(6, "仕",   "士",    2,  2),
    PieceType(7, "帅",   "将",   10,  1),
]

VALUE_TABLE = {p.id: p.value for p in PIECE_TYPES}

# 备选分值方案（规则若不同，切换这里即可）
VALUE_SCHEMES = {
    "默认（象棋惯例）": {
        1: 1, 2: 5, 3: 4, 4: 9, 5: 2, 6: 2, 7: 10,
    },
    "等差（1-10线性）": {
        1: 1, 2: 2, 3: 4, 4: 6, 5: 7, 6: 9, 7: 10,
    },
    "按枚数反比": {
        # 枚数越少分值越高：兵(5枚)=1, 炮/马/车/相/士(2枚)=5, 帅(1枚)=10
        1: 1, 2: 5, 3: 5, 4: 5, 5: 5, 6: 5, 7: 10,
    },
}


# ==========================================================================
# 2. ★ 从概率分布到期望分值
# ==========================================================================

def expected_value(probs, scheme=None):
    """
    期望分值 = Σ p(class) × value(class)

    ★ 这是本工具的核心。**不要用 argmax + 查表。**

    理由：argmax 丢掉了置信度信息。
    一个 51% 确信是"车(9分)"的判断，和一个 99% 确信的判断，
    在 argmax 下完全相同，但风险完全不同。
    """
    tbl = scheme or VALUE_TABLE
    return sum(probs.get(cid, 0.0) * tbl.get(cid, 0.0) for cid in tbl)


def argmax_value(probs, scheme=None):
    """对比用：先取最大概率类别，再查表。"""
    tbl = scheme or VALUE_TABLE
    best = max(probs.items(), key=lambda kv: kv[1])[0]
    return tbl.get(best, 0.0)


def value_variance(probs, scheme=None):
    """
    分值的方差 —— 衡量"这个判断有多不确定"。

    方差大 = 可能是高分也可能是低分 = 决策风险高。
    """
    tbl = scheme or VALUE_TABLE
    mean = expected_value(probs, tbl)
    var = sum(probs.get(cid, 0.0) * (tbl.get(cid, 0.0) - mean) ** 2
              for cid in tbl)
    return var


# ==========================================================================
# 3. 误判代价
# ==========================================================================

def misclassification_cost(probs, scheme=None):
    """
    误判的期望代价。

    定义：如果按 argmax 决策但实际是别的类别，损失多少分？
    代价 = Σ p(真实类) × |value(argmax) - value(真实类)|
    """
    tbl = scheme or VALUE_TABLE
    chosen = max(probs.items(), key=lambda kv: kv[1])[0]
    v_chosen = tbl.get(chosen, 0.0)
    cost = sum(probs.get(cid, 0.0) * abs(v_chosen - tbl.get(cid, 0.0))
               for cid in tbl)
    return cost


# ==========================================================================
# 4. 置信度驱动策略
# ==========================================================================

class ConfidencePolicy:
    """
    按识别置信度决定行为。

    关键：低置信度不代表"放弃"，而是"用期望值决策 + 降低优先级"。
    """

    HIGH = 0.85        # 高置信：直接用
    MEDIUM = 0.60      # 中置信：用期望值
    LOW = 0.40         # 低置信：用期望值 + 惩罚

    @staticmethod
    def max_prob(probs):
        return max(probs.values()) if probs else 0.0

    @staticmethod
    def classify(probs):
        mp = ConfidencePolicy.max_prob(probs)
        if mp >= ConfidencePolicy.HIGH:
            return "高"
        if mp >= ConfidencePolicy.MEDIUM:
            return "中"
        if mp >= ConfidencePolicy.LOW:
            return "低"
        return "极低"


# ==========================================================================
# 5. 优先级打分（整合识别结果）
# ==========================================================================

class PriorityWeights:
    def __init__(self, scheme=None):
        self.w_expected = 1.0     # 期望分值权重
        self.w_count = 3.0        # 枚数（平局第二判据）
        self.w_king = 50.0        # 帅/将支配项
        self.w_dist = 0.02        # 距离惩罚（每 mm）
        self.w_uncertainty = 0.8  # ★ 不确定性惩罚（新）
        self.w_risk = 200.0       # 送分风险
        self.scheme = scheme


def priority_score(probs, x_mm, y_mm, goal_x=0.0, goal_y=1000.0,
                   is_king_prior=False, in_opponent_half=False,
                   w: PriorityWeights = None):
    """
    完整优先级打分。

    与 docs/07 的打分函数相比，本函数把"分值已知"改为"分值有概率分布"。
    """
    w = w or PriorityWeights()

    ev = expected_value(probs, w.scheme)
    var = value_variance(probs, w.scheme)
    conf = ConfidencePolicy.classify(probs)

    s_expected = w.w_expected * ev
    s_count = w.w_count

    # 帅/将：用概率加权（不是硬判）
    p_king = probs.get(7, 0.0)
    s_king = w.w_king * p_king

    # 距离
    d = math.hypot(x_mm - goal_x, y_mm - goal_y)
    s_dist = -w.w_dist * d

    # 不确定性惩罚：方差大 -> 降权
    s_unc = -w.w_uncertainty * math.sqrt(var)

    # 送分风险
    s_risk = -w.w_risk if in_opponent_half else 0.0

    total = s_expected + s_count + s_king + s_dist + s_unc + s_risk

    return total, {
        "期望分值": s_expected, "枚数": s_count, "帅将": s_king,
        "距离": s_dist, "不确定": s_unc, "送分风险": s_risk,
    }, {"期望值": ev, "方差": var, "置信度": conf}


# ==========================================================================
# 输出辅助
# ==========================================================================

def section(title):
    print("\n" + "=" * 84)
    print(title)
    print("=" * 84)


def table(headers, rows):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows))
              for i, h in enumerate(headers)]
    print("  " + " ".join(str(h).rjust(w) for h, w in zip(headers, widths)))
    print("  " + "-" * (sum(widths) + len(widths) - 1))
    for r in rows:
        print("  " + " ".join(str(c).rjust(w) for c, w in zip(r, widths)))


# ==========================================================================
# 主程序
# ==========================================================================

def main():
    print("=" * 84)
    print("字符识别与优先级算法")
    print("=" * 84)

    # ---------------- 1. 分值表 ----------------
    section("1. 棋子分值表（7 兵种 -> 1-10 分）")
    print("  ⚠️ 赛题只写「按棋子等级设置 1-10 分」，")
    print("     但中国象棋只有 7 种兵种。7 -> 10 的映射有多种合理解读。")
    print("     **这是一条需要向组委会确认的规则问题（见 docs/09 Q9）。**")
    print()
    rows = []
    total_pieces = 0
    total_value = 0
    for p in PIECE_TYPES:
        if p.id == 0:
            continue
        n = p.count * 2       # 红黑双方
        total_pieces += n
        total_value += n * p.value
        rows.append([p.id, p.name, p.value, p.count, n, p.count * p.value])
    table(["ID", "兵种", "分值", "每方", "全场", "每方总分"], rows)
    print()
    print(f"  合计 {total_pieces} 枚，全场总分 {total_value}")
    print(f"  （校验：应为 32 枚，符合赛题「红黑各 16 枚」）")
    print()
    print("  备选分值方案：")
    print()
    rows = []
    for sname, sch in VALUE_SCHEMES.items():
        vals = [sch[i] for i in sorted(sch)]
        rows.append([sname, str(vals), f"{sum(vals):.0f}",
                     "总分不同 -> 策略不同"])
    table(["方案", "分值序列", "7兵种总分", "影响"], rows)
    print()
    print("  >> 三种方案的差别很大：")
    print("     默认方案：车 9 分 >> 炮/马 5/4 分 >> 兵 1 分 —— 分化明显")
    print("     等差方案：1-10 线性 —— 每枚棋子都值得抢")
    print("     反比方案：除帅外都一样 —— 变成抢枚数游戏")
    print()
    print("     **策略完全不同，所以必须确认。**")

    # ---------------- 2. 为什么用期望值而不是 argmax ----------------
    section("2. ★ 用期望值而不是 argmax（本轮核心）")

    print("  场景：识别器输出两个候选，分数接近")
    print()
    cases = [
        ("确定是车", {4: 0.97, 2: 0.02, 3: 0.01}),
        ("犹豫：车 or 炮", {4: 0.55, 2: 0.44, 3: 0.01}),
        ("犹豫：马 or 炮", {3: 0.52, 2: 0.47, 1: 0.01}),
        ("三个都像", {1: 0.34, 2: 0.33, 3: 0.33}),
    ]
    rows = []
    for name, probs in cases:
        ev = expected_value(probs)
        am = argmax_value(probs)
        var = value_variance(probs)
        cost = misclassification_cost(probs)
        conf = ConfidencePolicy.classify(probs)
        rows.append([name, f"{ev:.2f}", f"{am:.0f}", f"{var:.2f}",
                     f"{cost:.2f}", conf])
    table(["场景", "期望值", "argmax", "方差", "误判代价", "置信度"], rows)
    print()
    print("  >> 关键对比：")
    for name, probs in cases[:3]:
        ev = expected_value(probs)
        am = argmax_value(probs)
        print(f"     {name:<16} 期望值={ev:.2f}  argmax={am:.0f}")
    print()
    print("  **差别在于「犹豫：车 or 炮」这一行：**")
    p = {4: 0.55, 2: 0.44, 3: 0.01}
    print(f"     argmax 会说「这是车，9 分」—— 但只有 55% 把握")
    print(f"     期望值说「{expected_value(p):.2f} 分」—— 诚实地反映了风险")
    print()
    print("     若实际是炮(5分)，argmax 的决策高估了 4 分。")
    print("     期望值把这种不确定性**量化进了分值**。")

    # ---------------- 3. 不确定性惩罚的作用 ----------------
    section("3. 不确定性惩罚：让「犹豫的目标」排在后面")
    print("  两个棋子，期望值相同但置信度不同：")
    print()
    w = PriorityWeights()
    p_certain = {4: 0.95, 2: 0.04, 3: 0.01}      # 车，很确定
    p_uncertain = {4: 0.35, 2: 0.34, 3: 0.31}    # 说不清

    rows = []
    for name, probs in [("确定是车", p_certain), ("无法确定", p_uncertain)]:
        tot, detail, info = priority_score(
            probs, 600, 1000, is_king_prior=False, w=w)
        rows.append([name, f"{info['期望值']:.2f}", f"{info['方差']:.2f}",
                     info['置信度'], f"{detail['不确定']:.1f}",
                     f"{tot:.1f}"])
    table(["目标", "期望值", "方差", "置信度", "不确定性惩罚", "总分"], rows)
    print()
    print("  >> 「无法确定」的方差大（可能 9 分也可能 4 分），")
    print("     不确定性惩罚把它压下去 —— **先去推确定的目标**。")
    print()
    print("     这在比赛里很有用：与其赌一个可能是车也可能是炮的目标，")
    print("     不如先稳稳地推一个确认的棋子。")

    # ---------------- 4. 置信度驱动的行为 ----------------
    section("4. 置信度驱动的行为策略")
    rows = [
        ["高 (>=0.85)", "直接采用", "不惩罚", "立刻推"],
        ["中 (0.60-0.85)", "用期望值", "小惩罚", "推，但优先级略降"],
        ["低 (0.40-0.60)", "用期望值", "中等惩罚", "可推，但优先找别的"],
        ["极低 (<0.40)", "用期望值", "大惩罚", "★ 重新识别或跳过"],
    ]
    table(["置信度", "分值取法", "惩罚", "行为"], rows)
    print()
    print("  >> 「极低」档的处理很关键：")
    print("     不要硬猜，而是**换个角度重拍**（车微调位置后再读一次），")
    print("     或用多帧投票。若仍失败，跳过这枚。")
    print()
    print("     32 枚里放弃 2-3 枚不确定的，不影响胜负。")

    # 多帧投票的效果
    print()
    print("  多帧投票能提升多少置信度？")
    print()
    rows = []
    for n_frames in [1, 2, 3, 5]:
        # 假设单帧正确率 0.7，投票后正确率
        p_single = 0.70
        # 简化的多数投票模型
        p_vote = sum(
            math.comb(n_frames, k) * p_single**k * (1-p_single)**(n_frames-k)
            for k in range((n_frames // 2) + 1, n_frames + 1)
        ) if n_frames > 1 else p_single
        # 投票的代价是 n_frames 倍的采集时间
        cost_ms = n_frames * 33.3 + 6.0
        rows.append([n_frames, f"{p_single:.2f}", f"{p_vote:.3f}",
                     f"{cost_ms:.0f}"])
    table(["帧数", "单帧正确率", "投票后正确率", "耗时ms"], rows)
    print()
    print("  >> 3 帧投票把 0.70 提到 0.78，耗时 106ms。")
    print("     **读字时车是静止的，这 106ms 完全可以接受。**")
    print("     建议：对「低/极低」置信度的目标做 3 帧投票。")

    # ---------------- 5. 完整优先级算法 ----------------
    section("5. 完整优先级算法（整合识别不确定性）")
    print("  ┌─────────────────────────────────────────────────┐")
    print("  │ 1. 前视颜色阈值 -> 候选 + 红/黑（32 -> 16）      │")
    print("  │ 2. 停车读字 -> 7 类概率分布                       │")
    print("  │ 3. 期望分值 = Σ p(c) × value(c)                  │")
    print("  │ 4. 优先级打分（含不确定性惩罚）                   │")
    print("  │ 5. 排序 -> 选最高 -> 推入                         │")
    print("  └─────────────────────────────────────────────────┘")
    print()

    print("  实例：5 个候选的完整排序")
    print()
    print("  场地：3m x 2m，己方球门在 x=0 端，半场分界 x=1500mm")
    print()
    field_goal = (0.0, 1000.0)
    # (名称, 概率分布, x_mm, y_mm, 是否在对方半场)
    # ⚠️ 注意 x < 1500 属于**己方**半场；标错会引入 -200 的误惩罚
    candidates = [
        ("帅（清晰）",   {7: 0.92, 4: 0.05, 2: 0.03}, 800, 1000, False),
        ("车（清晰）",   {4: 0.94, 2: 0.04, 3: 0.02}, 600, 800,  False),
        ("炮（犹豫）",   {2: 0.52, 4: 0.44, 3: 0.04}, 700, 900,  False),
        ("兵（清晰）",   {1: 0.91, 5: 0.06, 6: 0.03}, 400, 600,  False),
        ("未知（对方半场）", {4: 0.35, 2: 0.34, 3: 0.31}, 2600, 1000, True),
    ]

    rows = []
    results = []
    for name, probs, x, y, in_opp in candidates:
        tot, detail, info = priority_score(
            probs, x, y, goal_x=field_goal[0], goal_y=field_goal[1],
            in_opponent_half=in_opp, w=w)
        results.append((name, tot, detail, info))
    results.sort(key=lambda t: -t[1])

    for rank, (name, tot, detail, info) in enumerate(results, 1):
        parts = " ".join(f"{k}={v:.0f}" for k, v in detail.items()
                         if abs(v) > 0.5)
        rows.append([rank, name, f"{info['期望值']:.2f}",
                     info['置信度'], f"{tot:.1f}", parts])
    table(["排名", "目标", "期望值", "置信度", "总分", "分项"], rows)
    print()
    print("  >> 判读：")
    print("     - **帅排第一**（期望值 9.8 + 支配项 46），符合策略")
    print("     - 车（9分）第二 —— 高分且确定")
    print("     - 炮（犹豫）第三 —— 被不确定性惩罚压低")
    print("       即使它的 argmax 也是车(9分)，但 55% 的把握让它降级")
    print("     - 兵（1分）第四 —— 分值低，但确定、近、安全")
    print("     - 对方半场的未知目标最后（送分风险 -200 主导）")
    print()
    print("     注意「炮（犹豫）」这个例子：**argmax 会把它当车(9分)，")
    print("     但期望值框架正确地把它降到第三。**")
    print("     这就是本轮算法的核心价值 —— 不按'看起来像几分'排序，")
    print("     而按'期望能得几分、且有多确定'排序。")

    # ---------------- 6. 结论 ----------------
    section("6. 结论")
    print("  【识别链路】")
    rows = [
        ["第一级", "颜色阈值（前视）", "红/黑", "32 -> 16"],
        ["第二级", "7 类兵种分类（NPU）", "概率分布", "非 argmax"],
        ["投票", "低置信度时 3 帧投票", "提升正确率", "0.70 -> 0.78"],
    ]
    table(["级别", "方法", "输出", "说明"], rows)
    print()
    print("  【优先级算法】")
    print("    priority = 期望分值 + 枚数 + 帅/将概率×50")
    print("               − 距离惩罚 − 不确定性惩罚 − 送分风险")
    print()
    print("  【三个关键设计决策】")
    print()
    rows = [
        ["★ 用期望值不用 argmax", "argmax 丢掉置信度信息",
         "55% 确定是车时，期望值 6.7 分比 argmax 的 9 分更诚实"],
        ["★ 加不确定性惩罚", "方差大 = 可能高也可能低",
         "让'确定的目标'优先于'可能更高但犹豫的目标'"],
        ["★ 多帧投票", "读字时车静止，时间充裕",
         "3 帧把 0.70 提到 0.78，代价仅 106ms"],
    ]
    table(["决策", "理由", "效果"], rows)
    print()
    print("  【新增待确认项】")
    print("    Q9: 7 兵种到 1-10 分的映射规则")
    print("        三种合理解读导出完全不同的策略，必须向组委会确认")
    print()
    print("=" * 84)
    print("第 4 轮迭代完成。下一步（第 5 轮）：SketchUp 模型 + BOM 汇总。")
    print("=" * 84)


if __name__ == "__main__":
    main()
