#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文档一致性审计工具
==================

用途：扫描全部文档，找出**仍在引用已弃用旧方案**的位置。

背景：平台已从「五轮迭代方案」切换到「实际下单方案」：
  底盘   差速 2 轮        -> 麦克纳姆轮 4 轮
  电机   JGB37-520 x2     -> MG513X x4（1:28 / 370rpm）
  驱动   BTS7960 x2       -> TB6612 四路带稳压 D24A
  主控   STM32H723        -> STM32G474VET6
  视觉   RK3588 + 双IMX219 -> K230D 128M + 单 OV5640
  电池   3S 1500mAh       -> 3S 2600mAh
  轮径   φ65 橡胶轮        -> φ60 麦轮

**不只看关键词**：还要区分
  - 必须改：当前设计文档（00-12、16-18）
  - 保留但需标注：历史迭代记录（13/14/15 是第 2/3/4 轮的推导过程）

用法：
    python tools/audit_docs.py            # 全量审计
    python tools/audit_docs.py --brief    # 只看汇总
"""

import argparse
import io
import os
import re
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs")


# ==========================================================================
# 旧方案特征词 -> 新方案对应
# ==========================================================================

REPLACEMENTS = [
    # (旧, 新, 类别)
    ("RK3588", "K230D", "视觉平台"),
    ("IMX219", "OV5640", "相机"),
    ("STM32H723", "STM32G474VET6", "主控"),
    ("H723", "G474", "主控"),
    ("STM32G431", "STM32G474VET6", "主控"),
    ("BTS7960", "TB6612 四路 D24A", "驱动"),
    ("IBT-2", "TB6612 四路 D24A", "驱动"),
    ("JGB37-520", "MG513X", "电机"),
    ("JGB37", "MG513X", "电机"),
    ("麦克纳姆轮", "麦克纳姆轮", "底盘"),   # 新方案用麦轮（原本反对）
    ("差速", "麦轮全向", "底盘"),
    ("3S 1500mAh", "3S 2600mAh", "电池"),
    ("φ65", "φ60", "轮径"),
    ("1500mAh 25C", "2600mAh", "电池"),
]

# 这些文档是**历史迭代记录**，保留旧值有意义，只需顶部标注
HISTORICAL = {
    "13-主控与算力平台.md": "第 2 轮：CPU vs NPU 瓶颈诊断（当时结论）",
    "14-1080p视觉管线.md": "第 3 轮：RGA 降采样与传感器余量（当时结论）",
    "15-字符识别与优先级算法.md": "第 4 轮：期望值与不确定性惩罚（算法仍适用）",
    "17-实际电机参数核对.md": "2026-09 电机参数核对（后被 MG513X 取代）",
}

# 必须使用新方案的文档
CURRENT = {
    "00-赛题解读与关键结论.md", "01-总体方案.md", "02-机械系统设计.md",
    "03-电控与驱动系统.md", "04-视觉识别系统.md", "05-定位与导航.md",
    "06-电源系统.md", "07-软件架构与决策策略.md", "08-BOM物料清单.md",
    "09-待确认事项与风险.md", "10-开发计划与里程碑.md", "11-通信协议.md",
    "12-帧率与实时性.md", "16-最终商品清单.md", "18-采购记录_20260928.md",
}


# ==========================================================================
# 审计
# ==========================================================================

def read_docs():
    out = {}
    for f in sorted(os.listdir(DOCS)):
        if f.endswith(".md"):
            out[f] = io.open(os.path.join(DOCS, f), encoding="utf-8").read()
    return out


def has_supersede_notice(text):
    """
    文档顶部是否已有「以新方案为准 / 旧版仅备查」的标注。

    识别多种措辞，避免把已经改好的文档误判为待处理。
    """
    head = text[:2000]
    patterns = [
        r"已弃用", r"已被.*取代", r"以\s*\d+\s*为准", r"历史档案",
        r"平台分叉", r"方案裁决", r"已按.*实际.*回写", r"采购修订版",
        r"实际到手硬件", r"实际下单", r"没有删除.*保留",
        r"历史档案", r"有意保留", r"不代表当前设计",
    ]
    return any(re.search(p, head) for p in patterns)


def in_archive_section(text, pos):
    """
    判断位置 pos 是否处于「旧版备查 / 归档」区块内。

    这类区块**有意保留旧值**，不算待修。
    识别：往上找最近的二级标题，看标题是否含"归档/备查/旧版/历史"。
    """
    head = text[:pos]
    # 最近的二级标题
    idx = head.rfind("\n## ")
    if idx < 0:
        return False
    line_end = head.find("\n", idx + 1)
    title = head[idx:line_end if line_end > 0 else len(head)]
    return bool(re.search(r"归档|备查|旧版|历史|原估算|弃用", title))


def is_comparison_context(text, keyword, window=200):
    """
    判断某关键词的出现是否处于「对照/裁决」语境（即有意保留旧值）。

    例如 docs/18 的冲突对照表、docs/10 的'不再保留差速分支'，
    这些是**正确用法**，不应算作待修。
    """
    for m in re.finditer(re.escape(keyword), text):
        if in_archive_section(text, m.start()):
            continue        # 归档区块内，有意保留
        s = max(0, m.start() - window)
        e = min(len(text), m.end() + window)
        ctx = text[s:e]
        if re.search(r"旧|原（|备查|对照|不再|弃用|冲突|差异|曾|前版|原估算"
                     r"|备 [A-Z]|降级|相对差速|相比|优于|劣势|优势"
                     r"|非 \d+ 章的|归档"
                     r"|非 16 章|vs 原|原方案|原 H|原需|早期|已修正"
                     r"|无法实现|改为|降级为|遗留|风险清单", ctx):
            continue
        return False        # 有一处不在对照语境
    return True             # 全部都在对照语境


def audit(brief=False):
    docs = read_docs()
    report = {}

    for name, text in docs.items():
        hits = {}
        for old, new, cat in REPLACEMENTS:
            if old == new:
                continue
            n = len(re.findall(re.escape(old), text))
            if n:
                # 区分：是否全部处于对照/裁决语境
                cmp_ctx = is_comparison_context(text, old)
                hits.setdefault(cat, []).append((old, n, cmp_ctx))
        if hits:
            report[name] = {
                "hits": hits,
                "historical": name in HISTORICAL,
                "current": name in CURRENT,
                "noticed": has_supersede_notice(text),
            }

    print("=" * 84)
    print("文档一致性审计：旧方案引用扫描")
    print("=" * 84)
    print(f"\n扫描 {len(docs)} 篇文档，{len(report)} 篇含旧方案关键词\n")

    need_fix = []
    ok_historical = []
    ok_noticed = []
    suspect = []

    for name in sorted(report):
        r = report[name]
        total = sum(n for v in r["hits"].values() for _, n, _c in v)
        # 所有命中是否都在对照/裁决语境里
        all_cmp = all(c for v in r["hits"].values() for _, _n, c in v)

        if r["historical"]:
            tag = "历史记录·可保留"
            if r["noticed"]:
                ok_historical.append(name)
            else:
                need_fix.append((name, "历史文档但**缺顶部标注**"))
                tag = "历史记录·**缺标注**"
        elif r["current"]:
            if all_cmp:
                # 所有命中都在对照/归档语境 —— 无论有无顶部标注都算 OK
                ok_noticed.append(name)
                tag = "当前文档·旧值仅在对照语境"
            elif r["noticed"]:
                # 有标注，但仍有未识别为对照语境的旧值。
                # ⚠️ 这类**误报率高**（技术对比、差异说明都会命中），
                #    因此归为"需人工复核"而非"确定漏改"。
                suspect.append(name)
                tag = "有标注·待人工复核（可能是技术对比）"
            else:
                need_fix.append((name, "当前文档仍引用旧方案且无标注"))
                tag = "**当前文档·需改**"
        else:
            tag = "其它"
            need_fix.append((name, "未分类"))

        if not brief:
            print(f"  {name}")
            print(f"    引用 {total} 处  [{tag}]")
            for cat, items in sorted(r["hits"].items()):
                parts = []
                for o, n, c in items:
                    parts.append(f"{o}×{n}" + ("(对照)" if c else "**"))
                print(f"      {cat}: {', '.join(parts)}")
            print()

    print("=" * 84)
    print("汇总")
    print("=" * 84)
    print(f"  历史文档且已标注: {len(ok_historical)}")
    for n in ok_historical:
        print(f"    ✓ {n}")
    print(f"  当前文档且已标注: {len(ok_noticed)}")
    for n in ok_noticed:
        print(f"    ✓ {n}")
    if suspect:
        print(f"  待人工复核: {len(suspect)}（可能有误报）")
        for n in suspect:
            print(f"    ? {n}")
    print(f"  确定需处理: {len(need_fix)}")
    for n, why in need_fix:
        print(f"    ⚠️ {n} —— {why}")

    return need_fix


# ==========================================================================
# 数值一致性（质量预算的关键矛盾）
# ==========================================================================

def audit_mass():
    """
    审计质量预算是否一致。

    ★ 这是最要紧的：新方案电机 4 个占 680g，
    而旧文档里的质量预算全是按 2 电机差速算的。
    """
    print("\n" + "=" * 84)
    print("质量预算一致性")
    print("=" * 84)

    # 扫描各文档里声称的总质量
    pats = [
        (r"总质量[^\n]*?(\d{3,4})\s*g", "总质量"),
        (r"\*\*(\d{3,4})\s*g\*\*", "粗体质量"),
        (r"(\d{3,4})\s*g\s*/\s*1500", "占上限"),
    ]
    found = {}
    for f in sorted(os.listdir(DOCS)):
        if not f.endswith(".md"):
            continue
        text = io.open(os.path.join(DOCS, f), encoding="utf-8").read()
        vals = set()
        for pat, _ in pats:
            for m in re.finditer(pat, text):
                v = int(m.group(1))
                if 200 <= v <= 1500:
                    vals.add(v)
        if vals:
            found[f] = sorted(vals)

    print(f"\n各文档声称的质量数值：")
    for f, vals in sorted(found.items()):
        mark = ""
        if any(v in (987, 1134) for v in vals):
            mark = "  <- 旧方案推算值"
        print(f"  {f:<44} {vals}{mark}")

    print(f"""
  >> 判读：
     旧方案（2 电机差速）推算总质量 987g / 1134g
     新方案（4 电机麦轮）仅电机就 680g，电池约 200g
       -> 已用 880g，**剩 620g 给全部结构件**

     ⚠️ 旧文档里的 987g / 1134g **全部不适用**，
        必须按新方案重新称重核算（docs/16 已标 '待实测'）""")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brief", action="store_true")
    args = ap.parse_args()

    need = audit(args.brief)
    audit_mass()

    print("\n" + "=" * 84)
    print(f"审计完成：{len(need)} 项待处理")
    print("=" * 84)
    return 0


if __name__ == "__main__":
    sys.exit(main())
