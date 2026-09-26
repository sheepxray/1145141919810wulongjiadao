#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GitHub Pages 站点生成器
========================

把 docs/ 的 17 篇设计文档、tools/ 的 9 个校核脚本、硬件模型与 BOM
生成一个完整的静态站点。

**为什么自己写而不是用 MkDocs**
MkDocs Material 需要一长串依赖（mkdocs、material、pygments、pymdownx...），
在当前网络环境下安装耗时长且易失败。本生成器只依赖 `markdown` 一个包，
能**本地实际构建验证**，且不需要 CI 也能出结果。

用法：
    python tools/build_site.py              # 构建到 site/
    python tools/build_site.py --serve      # 构建并用本地服务器预览
    python tools/build_site.py --check      # 只做链接完整性检查

输出：
    site/                    完整的静态站点，可直接部署
"""

import argparse
import html
import io
import os
import re
import shutil
import sys
import unicodedata
from datetime import datetime

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

try:
    import markdown
except ImportError:
    print("需要 markdown 库：pip install markdown")
    sys.exit(1)


# ==========================================================================
# 路径
# ==========================================================================

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs")
SITE = os.path.join(ROOT, "site")
REPO_URL = "https://github.com/sheepxray/1145141919810wulongjiadao"


# ==========================================================================
# ★ 站点结构 —— 手工编排，因为文档之间有逻辑分组
# ==========================================================================

NAV = [
    ("开始", [
        ("index", "index.md", "首页"),
        ("00", "00-赛题解读与关键结论.md", "赛题解读"),
    ]),
    ("五轮迭代", [
        ("06", "06-电源系统.md", "第 1 轮 · 电源与电流"),
        ("13", "13-主控与算力平台.md", "第 2 轮 · 主控与算力"),
        ("14", "14-1080p视觉管线.md", "第 3 轮 · 1080p 视觉"),
        ("15", "15-字符识别与优先级算法.md", "第 4 轮 · 字符识别与优先级"),
        ("16", "16-最终商品清单.md", "第 5 轮 · 最终商品清单"),
    ]),
    ("系统设计", [
        ("01", "01-总体方案.md", "总体方案"),
        ("02", "02-机械系统设计.md", "机械系统"),
        ("03", "03-电控与驱动系统.md", "电控与驱动"),
        ("04", "04-视觉识别系统.md", "视觉识别"),
        ("05", "05-定位与导航.md", "定位与导航"),
        ("11", "11-通信协议.md", "通信协议"),
        ("07", "07-软件架构与决策策略.md", "软件架构与决策"),
        ("12", "12-帧率与实时性.md", "帧率与实时性"),
    ]),
    ("项目管理", [
        ("09", "09-待确认事项与风险.md", "待确认事项与风险"),
        ("10", "10-开发计划与里程碑.md", "开发计划"),
        ("08", "08-BOM物料清单.md", "BOM 物料清单"),
    ]),
    ("更新记录", [
        ("17", "17-实际电机参数核对.md", "实际电机参数核对"),
    ]),
]

# 页面 slug -> 源文件
PAGES = {}
for _grp, _items in NAV:
    for _slug, _path, _title in _items:
        if _slug != "index":
            PAGES[_slug] = _path


# ==========================================================================
# 工具数据（从脚本提取，供"工具"页展示）
# ==========================================================================

TOOLS_INFO = [
    ("power_budget.py", "电源与电流预算", "第 1 轮",
     "电机电气模型、堵转热分析、电池压降、线径保险丝、限流验证"),
    ("compute_budget.py", "主控算力预算", "第 2 轮",
     "控制环 CPU 占用率、内存预算、架构方案对比、CPU vs NPU 瓶颈诊断"),
    ("vision_pipeline.py", "1080p 视觉管线", "第 3 轮",
     "传感器帧率余量、内存带宽、RGA 降采样、端到端延迟"),
    ("piece_value.py", "字符识别与优先级", "第 4 轮",
     "分值表、期望值 vs argmax、不确定性惩罚、多帧投票"),
    ("sketchup_model.py", "SketchUp 模型生成器", "第 5 轮",
     "参数化生成 Ruby 建模脚本、尺寸自洽性校验、三视图输出"),
    ("geometry.py", "相机几何校核", "基础",
     "挡板可见性、测距精度、汉字可读性、推板贴墙容错"),
    ("mass_budget.py", "质量与预算校核", "基础",
     "赛题 1.5kg / ¥2000 上限检查、尺寸校核、迭代差异对比"),
    ("framerate.py", "帧率预算分析", "基础",
     "通用帧率方法论、延迟分析、降级阶梯（部分被 vision_pipeline 取代）"),
    ("protocol.py", "通信协议实现", "基础",
     "CRC16、9 种消息编解码、流式解析器、粘包/半包/噪声自测"),
]

FIRMWARE_INFO = [
    ("firmware/stm32/reference/kinematics.py", "差速运动学 / PID / 航迹推算",
     "正逆运动学、抗积分饱和 PID、航迹推算、轮径标定"),
    ("firmware/stm32/reference/scoring.py", "目标打分函数",
     "帅/将支配项、送分规避、走廊占用、降级策略 B"),
    ("firmware/k230/profiler.py", "K230 真机帧率实测", "在设备上测量各阶段耗时（PC 上安全降级）"),
]

# 关键指标（首页展示）
KEY_METRICS = [
    ("总质量", "1134", "g", "上限 1500 g"),
    ("总成本", "1486", "元", "预算 2000 元"),
    ("外廓", "223×195×160", "mm", "上限 250×200×180"),
    ("视觉帧率", "1080p@30", "", "SEARCH 58 / SERVO 244 FPS"),
    ("推入段延迟", "40.4", "ms", "位置滞后 10 mm"),
    ("贴墙容错", "40.0", "mm", "推板悬伸 22.5mm/侧"),
]

# 五轮迭代（首页展示）
ITERATIONS = [
    ("第 1 轮", "06", "电源与电流",
     "真正的风险是电机烧毁（堵转 86 秒），解法是限流而不是换大电池",
     "新增 INA240 电流采样 ×2"),
    ("第 2 轮", "13", "主控与算力",
     "瓶颈在视觉平台的 CPU（非 NPU）；G431 跑控制环只占 2%",
     "RK3588 + STM32H723"),
    ("第 3 轮", "14", "1080p 视觉",
     "关键是把降采样交给 RGA 硬件（CPU 降采样会让帧率掉到 28 FPS）",
     "IMX219（1080p@60）"),
    ("第 4 轮", "15", "字符识别",
     "识别输出概率分布，优先级须用期望值而非 argmax",
     "期望值 + 不确定性惩罚"),
    ("第 5 轮", "16", "建模与 BOM",
     "相机立柱使总高从 99.5 修正为 160mm（原模型漏算 50mm）",
     "参数化 SketchUp 模型"),
]

# 反直觉发现（首页展示）
INSIGHTS = [
    ("实际电机比设计假设小 5 倍",
     "实测 24Ω / 堵转 0.5A，而第一轮按 4.8Ω / 2.5A 设计。"
     "「堵转 86 秒烧毁」的结论在该电机上不成立 —— 但数据手册自身矛盾 2.2 倍，需量电阻定论。",
     "power_budget.py · docs/17"),
    ("把车体做宽对贴墙容错完全无效",
     "窗口 = 棋子半径 + 推板悬伸 − 离墙间隙，车体半宽被消掉。"
     "推板开口从 150 排到 200mm，窗口恒为 17.5mm。",
     "geometry.py"),
    ("挡板测距公式不能无条件使用",
     "d = H·f/Δy 只在相机水平时精确；俯角 30°、相机高 250mm 时误差 17.3%。",
     "geometry.py"),
    ("1080p@30 的瓶颈是 CPU 不是 NPU",
     "K230 的 NPU 有 6 TOPS 却跑不动，因为图像预处理在 CPU 侧"
     "（800 vs 20000 MOPS）。",
     "compute_budget.py"),
    ("换更大的电池解决不了电流问题",
     "堵转电流会让电机发热，只能用限流解决 —— 但实际电机发热比假设小 5 倍。",
     "power_budget.py"),
    ("下视相机不能垂直向下装",
     "垂直时可见地面仅 0–76mm，看不到推板前方 100–250mm 的棋子。"
     "必须前倾 60°。",
     "geometry.py"),
    ("argmax 会高估不确定的目标",
     "55% 确定是车时，argmax 说 9 分，期望值说 7.19 分 —— 后者才诚实。",
     "piece_value.py"),
]

# 自测捕获的真实缺陷
BUGS = [
    ("挡板测距公式「与俯仰角无关」有误", "俯角 30° 时误差 17.3%", "geometry.py"),
    ("协议 MAX_PAYLOAD 与自身规范矛盾", "TARGET_LIST 满 8 项需 70 字节 > 64", "protocol.py"),
    ("串口解析器被假帧头吃掉真帧", "导致「一帧噪声后持续丢包」", "protocol.py"),
    ("STM32 引脚冲突", "IMU 的 SPI1 与编码器 TIM3 冲突", "画引脚表"),
    ("相机立柱漏算", "总高 99.5 → 160mm", "sketchup_model.py"),
    ("电池内阻被称「相当」", "实际差 29%", "power_budget.py"),
    ("线径被判「不够」却推荐", "混淆毫秒级峰值与持续载流量", "power_budget.py"),
]

# 待确认事项
OPEN_QUESTIONS = [
    ("🔴 Q1", "球门区归属规则", "己方球门区收己方颜色棋子，还是任何颜色都收？"),
    ("🔴 Q2", "棋子能否背面朝上", "若能，颜色/分值/帅将全部失效"),
    ("🔴 Q9", "7 兵种到 1-10 分的映射", "三种合理解读导出完全不同的打法"),
    ("🟡 Q3", "比赛时长与赛制", "决定电池容量与时间策略"),
    ("🟡 Q4", "球门位置", "2m 端还是 3m 长边，决定推进轴向"),
    ("🟡 Q5", "棋子物理参数", "直径/厚度/材质/刻字方式"),
]


# ==========================================================================
# Markdown 渲染
# ==========================================================================

MD_EXTENSIONS = [
    "extra",          # 表格、围栏代码、属性列表
    "toc",            # 目录锚点
    "sane_lists",
    "smarty",
    "admonition",
    "nl2br",
]

MD_CONFIG = {
    "toc": {"permalink": True, "permalink_title": "链接到此节"},
}


def slugify(text):
    """与 python-markdown toc 扩展一致的锚点算法"""
    text = unicodedata.normalize("NFKD", text)
    text = re.sub(r"[^\w\s-]", "", text, flags=re.UNICODE).strip().lower()
    return re.sub(r"[-\s]+", "-", text)


class DocLinkRewriter:
    """
    把文档间的相对链接改写为站点内链接。

    docs/ 里用的是 `07-软件架构与决策策略.md` 这类文件名，
    站点里对应 `07.html`。
    """

    def __init__(self):
        # 文件名 -> slug
        self.name_to_slug = {}
        for slug, path, title in [(s, p, t) for _, items in NAV for s, p, t in items]:
            self.name_to_slug[os.path.basename(path)] = slug

    def rewrite(self, html_body):
        def repl(m):
            href = m.group(1)
            # 只看本地 .md 链接
            base = href.split("#")[0]
            frag = ("#" + href.split("#", 1)[1]) if "#" in href else ""
            if not base.endswith(".md"):
                return m.group(0)
            name = os.path.basename(base)
            slug = self.name_to_slug.get(name)
            if slug:
                return f'href="{slug}.html{frag}"'
            return m.group(0)

        return re.sub(r'href="([^"]+)"', repl, html_body)


# ==========================================================================
# 模板
# ==========================================================================

BASE_CSS = """
:root{
  --bg:#0d1117; --bg2:#161b22; --bg3:#21262d; --bd:#30363d;
  --tx:#e6edf3; --tx2:#8b949e; --ac:#58a6ff; --ac2:#3fb950;
  --wn:#d29922; --er:#f85149; --pu:#bc8cff;
  --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,"Liberation Mono",monospace;
}
*{box-sizing:border-box}
html{scroll-behavior:smooth;scroll-padding-top:70px}
body{margin:0;background:var(--bg);color:var(--tx);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans SC",
  "PingFang SC","Microsoft YaHei",sans-serif;font-size:16px;line-height:1.75}
a{color:var(--ac);text-decoration:none}
a:hover{text-decoration:underline}

/* 顶栏 */
.topbar{position:sticky;top:0;z-index:100;display:flex;align-items:center;gap:16px;
  height:56px;padding:0 20px;background:rgba(13,17,23,.95);
  backdrop-filter:blur(12px);border-bottom:1px solid var(--bd)}
.brand{display:flex;align-items:center;gap:10px;font-weight:600;
  color:var(--tx);white-space:nowrap}
.brand .dot{width:9px;height:9px;border-radius:50%;background:var(--ac2);
  box-shadow:0 0 10px var(--ac2)}
.brand small{color:var(--tx2);font-weight:400;font-size:12px}
.topbar .spacer{flex:1}
.topbar .btn{padding:5px 11px;border:1px solid var(--bd);border-radius:6px;
  font-size:13px;color:var(--tx2)}
.topbar .btn:hover{background:var(--bg3);color:var(--tx);text-decoration:none}

/* 布局 */
.wrap{display:grid;grid-template-columns:268px minmax(0,1fr);max-width:1500px;
  margin:0 auto;gap:0}
nav.side{position:sticky;top:56px;height:calc(100vh - 56px);overflow-y:auto;
  padding:20px 12px 40px;border-right:1px solid var(--bd);background:var(--bg)}
nav.side::-webkit-scrollbar{width:8px}
nav.side::-webkit-scrollbar-thumb{background:var(--bg3);border-radius:4px}
nav.side .grp{margin-bottom:20px}
nav.side .grp>h4{margin:0 0 7px;padding:0 10px;font-size:11px;font-weight:600;
  letter-spacing:.09em;text-transform:uppercase;color:var(--tx2)}
nav.side a{display:block;padding:6px 10px;border-radius:6px;font-size:14px;
  color:var(--tx2);line-height:1.45}
nav.side a:hover{background:var(--bg2);color:var(--tx);text-decoration:none}
nav.side a.on{background:var(--bg3);color:var(--tx);font-weight:500;
  box-shadow:inset 2px 0 0 var(--ac)}

main{padding:32px 40px 100px;min-width:0;max-width:1000px}

/* 内容排版 */
main h1{font-size:30px;margin:0 0 8px;padding-bottom:12px;
  border-bottom:1px solid var(--bd);line-height:1.3}
main h2{font-size:23px;margin:38px 0 14px;padding-bottom:8px;
  border-bottom:1px solid var(--bd)}
main h3{font-size:18px;margin:28px 0 11px}
main h4{font-size:16px;margin:22px 0 9px;color:var(--tx2)}
main p{margin:12px 0}
main ul,main ol{padding-left:26px;margin:12px 0}
main li{margin:5px 0}
main hr{border:0;border-top:1px solid var(--bd);margin:32px 0}

main code{background:var(--bg3);padding:2px 6px;border-radius:5px;
  font-family:var(--mono);font-size:.87em;color:var(--pu)}
main pre{background:var(--bg2);border:1px solid var(--bd);border-radius:8px;
  padding:15px 17px;overflow-x:auto;margin:14px 0}
main pre code{background:none;padding:0;color:var(--tx);font-size:13px;
  line-height:1.6}

main table{border-collapse:collapse;width:100%;margin:15px 0;
  font-size:14px;display:block;overflow-x:auto}
main th,main td{border:1px solid var(--bd);padding:8px 12px;text-align:left}
main th{background:var(--bg2);font-weight:600;white-space:nowrap}
main tr:nth-child(even) td{background:rgba(22,27,34,.5)}

main blockquote{margin:15px 0;padding:11px 17px;border-left:3px solid var(--ac);
  background:var(--bg2);border-radius:0 6px 6px 0;color:var(--tx2)}
main blockquote p{margin:5px 0}
main blockquote strong{color:var(--tx)}

/* 锚点 */
.headerlink{opacity:0;margin-left:9px;font-size:.75em;color:var(--tx2);
  text-decoration:none;transition:opacity .15s}
h1:hover .headerlink,h2:hover .headerlink,h3:hover .headerlink,
h4:hover .headerlink{opacity:1}

/* 首页组件 */
.hero{padding:48px 0 32px;border-bottom:1px solid var(--bd);margin-bottom:32px}
.hero h1{font-size:38px;border:0;padding:0;margin:0 0 12px;line-height:1.25}
.hero .sub{font-size:17px;color:var(--tx2);margin:0 0 20px;max-width:760px}
.hero .tags{display:flex;flex-wrap:wrap;gap:8px;margin-top:18px}
.hero .tags span{padding:4px 11px;background:var(--bg3);border-radius:20px;
  font-size:12.5px;color:var(--tx2);border:1px solid var(--bd)}

.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(185px,1fr));
  gap:13px;margin:26px 0}
.metric{padding:17px 19px;background:var(--bg2);border:1px solid var(--bd);
  border-radius:10px}
.metric .lbl{font-size:12px;color:var(--tx2);margin-bottom:5px}
.metric .val{font-size:27px;font-weight:600;line-height:1.15;
  font-family:var(--mono)}
.metric .val small{font-size:14px;font-weight:400;color:var(--tx2);
  margin-left:4px}
.metric .note{font-size:12px;color:var(--tx2);margin-top:5px}

.iters{display:flex;flex-direction:column;gap:11px;margin:22px 0}
.iter{display:grid;grid-template-columns:96px 1fr;gap:16px;padding:15px 18px;
  background:var(--bg2);border:1px solid var(--bd);border-radius:10px;
  border-left:3px solid var(--ac)}
.iter .n{font-family:var(--mono);font-size:13px;color:var(--ac);
  font-weight:600;padding-top:2px}
.iter h4{margin:0 0 5px;font-size:15.5px;color:var(--tx)}
.iter p{margin:0 0 6px;font-size:14px;color:var(--tx2);line-height:1.6}
.iter .res{font-size:12.5px;color:var(--ac2);font-family:var(--mono)}

.insights{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));
  gap:13px;margin:22px 0}
.insight{padding:16px 18px;background:var(--bg2);border:1px solid var(--bd);
  border-radius:10px}
.insight h4{margin:0 0 8px;font-size:14.5px;color:var(--wn);line-height:1.4}
.insight p{margin:0 0 9px;font-size:13.5px;color:var(--tx2);line-height:1.6}
.insight .src{font-size:11.5px;font-family:var(--mono);color:var(--tx2);
  opacity:.75}

.qlist{margin:20px 0}
.q{display:grid;grid-template-columns:64px 1fr;gap:14px;padding:12px 16px;
  background:var(--bg2);border:1px solid var(--bd);border-radius:8px;
  margin-bottom:9px;align-items:start}
.q .id{font-family:var(--mono);font-size:13px;font-weight:600}
.q.hi{border-left:3px solid var(--er)}
.q.hi .id{color:var(--er)}
.q.md{border-left:3px solid var(--wn)}
.q.md .id{color:var(--wn)}
.q h4{margin:0 0 3px;font-size:14.5px;color:var(--tx)}
.q p{margin:0;font-size:13.5px;color:var(--tx2)}

/* 工具卡片 */
.tools{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));
  gap:13px;margin:22px 0}
.tool{padding:16px 18px;background:var(--bg2);border:1px solid var(--bd);
  border-radius:10px}
.tool h4{margin:0 0 4px;font-size:14.5px;font-family:var(--mono);
  color:var(--ac2)}
.tool .rnd{font-size:11.5px;color:var(--tx2);margin-bottom:8px}
.tool p{margin:0 0 10px;font-size:13.5px;color:var(--tx2);line-height:1.6}
.tool code{display:block;background:var(--bg3);padding:7px 10px;border-radius:6px;
  font-size:12px;color:var(--tx);overflow-x:auto}

/* 页脚 */
footer{max-width:1500px;margin:0 auto;padding:26px 40px 44px;
  border-top:1px solid var(--bd);color:var(--tx2);font-size:13px}
footer a{color:var(--tx2)}
footer a:hover{color:var(--ac)}

/* 移动端 */
.menubtn{display:none;background:none;border:1px solid var(--bd);
  border-radius:6px;color:var(--tx);padding:5px 10px;font-size:17px;cursor:pointer}
@media(max-width:1000px){
  .wrap{grid-template-columns:1fr}
  .menubtn{display:block}
  nav.side{position:fixed;top:56px;left:0;width:280px;z-index:99;
    transform:translateX(-100%);transition:transform .22s;
    box-shadow:6px 0 22px rgba(0,0,0,.5)}
  nav.side.open{transform:none}
  main{padding:22px 18px 70px}
  .hero h1{font-size:28px}
  .iter{grid-template-columns:1fr;gap:6px}
  .q{grid-template-columns:1fr;gap:5px}
}
"""

BASE_JS = """
// 侧栏开关（移动端）
function toggleNav(){
  document.querySelector('nav.side').classList.toggle('open');
}
// 关闭抽屉（点内容区）
document.addEventListener('click', function(e){
  var n = document.querySelector('nav.side');
  if(!n || !n.classList.contains('open')) return;
  if(n.contains(e.target) || e.target.closest('.menubtn')) return;
  n.classList.remove('open');
});
"""


def page(title, body, active_slug=None, is_home=False):
    """套用页面模板"""
    # 侧栏
    nav_html = []
    for grp, items in NAV:
        nav_html.append('<div class="grp">')
        nav_html.append(f"<h4>{html.escape(grp)}</h4>")
        for slug, path, t in items:
            cls = ' class="on"' if slug == active_slug else ""
            nav_html.append(
                f'<a href="{slug}.html"{cls}>{html.escape(t)}</a>')
        nav_html.append("</div>")
    nav_html = "\n".join(nav_html)

    site_title = "光电赛智能车"
    full = f"{title} · {site_title}" if not is_home else \
        f"{site_title} · 第十五届全国大学生光电设计竞赛 赛题 1"

    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(full)}</title>
<meta name="description" content="第十五届全国大学生光电设计竞赛 赛题1 智能车光电象棋对抗 — 设计文档与校核工具">
<style>{BASE_CSS}</style>
</head>
<body>
<header class="topbar">
  <button class="menubtn" onclick="toggleNav()" aria-label="菜单">☰</button>
  <a class="brand" href="index.html">
    <span class="dot"></span>
    <span>{site_title}</span>
    <small>光电象棋对抗</small>
  </a>
  <span class="spacer"></span>
  <a class="btn" href="tools.html">工具</a>
  <a class="btn" href="https://github.com/sheepxray/1145141919810wulongjiadao"
     target="_blank" rel="noopener">GitHub</a>
</header>
<div class="wrap">
<nav class="side">
{nav_html}
</nav>
<main>
{body}
</main>
</div>
<footer>
  <p>
    第十五届全国大学生光电设计竞赛 · 赛题 1 智能车光电象棋对抗 ·
    <a href="https://github.com/sheepxray/1145141919810wulongjiadao"
       target="_blank" rel="noopener">仓库</a> ·
    生成于 {datetime.now().strftime('%Y-%m-%d')}
  </p>
  <p style="opacity:.7">
    所有数值均可用仓库内脚本复现。价格与耗时为预估值，采购前需实时核价与真机实测。
  </p>
</footer>
<script>{BASE_JS}</script>
</body>
</html>"""


# ==========================================================================
# 首页
# ==========================================================================

def build_home():
    m = "".join(f"""
    <div class="metric">
      <div class="lbl">{html.escape(l)}</div>
      <div class="val">{html.escape(v)}<small>{html.escape(u)}</small></div>
      <div class="note">{html.escape(n)}</div>
    </div>""" for l, v, u, n in KEY_METRICS)

    it = "".join(f"""
    <div class="iter">
      <div class="n">{html.escape(n)}</div>
      <div>
        <h4>{html.escape(t)}</h4>
        <p>{html.escape(d)}</p>
        <div class="res">→ {html.escape(r)}</div>
      </div>
    </div>""" for n, slug, t, d, r in ITERATIONS)

    ins = "".join(f"""
    <div class="insight">
      <h4>{html.escape(t)}</h4>
      <p>{html.escape(d)}</p>
      <div class="src">验证：{html.escape(s)}</div>
    </div>""" for t, d, s in INSIGHTS)

    qs = "".join(f"""
    <div class="q {'hi' if qid.startswith('🔴') else 'md'}">
      <div class="id">{html.escape(qid)}</div>
      <div>
        <h4>{html.escape(t)}</h4>
        <p>{html.escape(d)}</p>
      </div>
    </div>""" for qid, t, d in OPEN_QUESTIONS)

    bugs = "".join(
        f"<tr><td>{i}</td><td>{html.escape(t)}</td><td>{html.escape(d)}</td>"
        f"<td><code>{html.escape(s)}</code></td></tr>"
        for i, (t, d, s) in enumerate(BUGS, 1))

    body = f"""
<div class="hero">
  <h1>智能车光电象棋对抗</h1>
  <p class="sub">
    第十五届全国大学生光电设计竞赛 · 赛题 1 的完整设计方案。
    一台差速驱动智能车，用双相机做 1080p@30 视觉识别，自主完成
    「搜索 → 识别 → 计算优先级 → 推入球门」全流程。
  </p>
  <div class="tags">
    <span>只有 3D 打印</span>
    <span>预算 ≤ ¥2000</span>
    <span>3 个月工期</span>
    <span>1080p@30 视觉</span>
    <span>五轮迭代</span>
  </div>
</div>

<h2>最终指标</h2>
<div class="metrics">{m}</div>

<h2>五轮迭代</h2>
<p>每轮都产出一个可运行的校核脚本，并已提交到仓库。</p>
<div class="iters">{it}</div>

<h2>反直觉的发现</h2>
<p>以下结论都经可运行脚本验证，且都与直觉相反 —— 它们改变了设计方向。</p>
<div class="insights">{ins}</div>

<h2>待确认事项</h2>
<p>
  以下规则模糊点会改变整个策略或硬件选型。
  <strong>标红的三条必须在采购前书面向组委会确认。</strong>
</p>
<div class="qlist">{qs}</div>

<h2>自测捕获的真实缺陷</h2>
<p>
  这些脚本不只是文档，它们的自测<strong>实际抓到了 7 个真实缺陷</strong>，
  全部已修正。
</p>
<table>
<thead><tr><th>#</th><th>缺陷</th><th>后果</th><th>发现工具</th></tr></thead>
<tbody>{bugs}</tbody>
</table>

<h2>快速开始</h2>
<pre><code># 校核尺寸与质量（采购前必做）
python tools/sketchup_model.py
python tools/mass_budget.py

# 生成 SketchUp 建模脚本
#   → SketchUp 2026：窗口 → Ruby 控制台 → 粘贴 build_robot.rb

# 五轮迭代的校核工具
python tools/power_budget.py       # 第1轮 电流/堵转热分析
python tools/compute_budget.py     # 第2轮 主控算力
python tools/vision_pipeline.py    # 第3轮 1080p 管线
python tools/piece_value.py        # 第4轮 字符识别优先级
</code></pre>

<p style="margin-top:28px">
  <a href="00.html">开始阅读：赛题解读 →</a> ·
  <a href="16.html">直接看最终商品清单 →</a> ·
  <a href="tools.html">全部工具 →</a>
</p>
"""
    return page("首页", body, active_slug="index", is_home=True)


# ==========================================================================
# 工具页
# ==========================================================================

def build_tools():
    items = "".join(f"""
    <div class="tool">
      <h4>{html.escape(n)}</h4>
      <div class="rnd">{html.escape(t)} · {html.escape(r)}</div>
      <p>{html.escape(d)}</p>
      <code>python tools/{html.escape(n)}</code>
    </div>""" for n, t, r, d in TOOLS_INFO)

    fw = "".join(f"""
    <div class="tool">
      <h4>{html.escape(os.path.basename(n))}</h4>
      <div class="rnd">{html.escape(t)}</div>
      <p>{html.escape(d)}</p>
      <code>{html.escape(n)}</code>
    </div>""" for n, t, d in FIRMWARE_INFO)

    bugs = "".join(
        f"<tr><td>{i}</td><td>{html.escape(t)}</td><td>{html.escape(d)}</td>"
        f"<td><code>{html.escape(s)}</code></td></tr>"
        for i, (t, d, s) in enumerate(BUGS, 1))

    body = f"""
<h1>工具</h1>
<p>
  仓库内共 <strong>12 个可运行脚本</strong>，每个都带自测。
  它们不是纸面文档 —— 自测实际捕获了 7 个真实缺陷。
</p>

<h2>校核工具</h2>
<div class="tools">{items}</div>

<h2>固件参考实现</h2>
<div class="tools">{fw}</div>

<h2>自测捕获的真实缺陷</h2>
<p>这些是脚本运行时确实报错、或数值对不上而被发现的，全部已修正。</p>
<table>
<thead><tr><th>#</th><th>缺陷</th><th>后果</th><th>发现工具</th></tr></thead>
<tbody>{bugs}</tbody>
</table>

<h2>约定</h2>
<ul>
  <li><strong>可复现</strong>：所有数值都标注了来源，跑对应脚本即可验证</li>
  <li><strong>预估值须实测</strong>：标记 <code>[预估]</code> 的耗时要替换为真机数据</li>
  <li><strong>价格须核价</strong>：生成时的联网检索不可用，价格为经验估计</li>
</ul>

<h2>运行环境</h2>
<pre><code>Python 3.8+
仅需一个第三方库：markdown（只有 build_site.py 用到）
其余脚本全部使用标准库
</code></pre>

<h2>真机工具</h2>
<p>
  <code>firmware/k230/profiler.py</code> 需要在 K230 上运行以实测帧率。
  在 PC 上运行会安全降级并提示部署步骤。
</p>
"""
    return page("工具", body, active_slug="tools")


# ==========================================================================
# 构建
# ==========================================================================

def build(check_only=False):
    if not check_only:
        if os.path.exists(SITE):
            shutil.rmtree(SITE)
        os.makedirs(SITE, exist_ok=True)

    rewriter = DocLinkRewriter()

    # ---- 渲染文档页 ----
    rendered = 0
    missing = []
    for slug, path, title in [(s, p, t) for _, items in NAV for s, p, t in items
                              if s != "index"]:
        src = os.path.join(DOCS, path)
        if not os.path.exists(src):
            missing.append(path)
            continue

        text = io.open(src, encoding="utf-8").read()
        # 去掉第一行的 H1（模板已用 title 显示，避免重复）
        # ⚠️ 只删**第一行**，不能用 `.*?\n` —— 那会连带吞掉标题后的正文
        lines = text.split("\n")
        if lines and lines[0].lstrip().startswith("# "):
            lines = lines[1:]
        text = "\n".join(lines)

        md = markdown.Markdown(extensions=MD_EXTENSIONS, extension_configs=MD_CONFIG)
        body = md.convert(text)
        body = rewriter.rewrite(body)

        # 目录（★ 必须**追加**到正文，不能覆盖 body）
        toc = getattr(md, "toc", "")
        if toc:
            toc = re.sub(r'<div class="toc">|</div>', "", toc)
            body = body + (
                f'<details class="tocbox" style="margin:0 0 26px;'
                f'padding:12px 16px;background:var(--bg2);'
                f'border:1px solid var(--bd);border-radius:8px">'
                f'<summary style="cursor:pointer;color:var(--tx2);'
                f'font-size:13.5px">本页目录</summary>'
                f'<div style="margin-top:9px;font-size:13.5px">{toc}</div>'
                f'</details>')

        if not check_only:
            out = os.path.join(SITE, f"{slug}.html")
            io.open(out, "w", encoding="utf-8").write(
                page(title, body, active_slug=slug))
        rendered += 1

    # ---- 首页与工具页 ----
    if not check_only:
        io.open(os.path.join(SITE, "index.html"), "w",
                encoding="utf-8").write(build_home())
        io.open(os.path.join(SITE, "tools.html"), "w",
                encoding="utf-8").write(build_tools())

        # .nojekyll：阻止 GitHub Pages 用 Jekyll 处理（避免下划线开头的文件被忽略）
        io.open(os.path.join(SITE, ".nojekyll"), "w").write("")

    return rendered, missing


def check_links():
    """检查站内链接完整性。site/ 不存在时返回错误码（不静默通过）"""
    if not os.path.exists(SITE):
        print("\n链接检查：⚠️ site/ 不存在 —— 请先运行构建")
        return 1

    pages = set()
    for f in os.listdir(SITE):
        if f.endswith(".html"):
            pages.add(f)

    if not pages:
        print("\n链接检查：⚠️ 没有 HTML 文件 —— 构建可能失败")
        return 1

    problems = []
    for f in sorted(pages):
        txt = io.open(os.path.join(SITE, f), encoding="utf-8").read()
        for m in re.finditer(r'href="([^"]+)"', txt):
            h = m.group(1)
            if h.startswith(("http://", "https://", "#", "mailto:")):
                continue
            target = h.split("#")[0]
            if target and target not in pages:
                problems.append((f, h))

    print(f"\n链接检查：{len(pages)} 个页面")
    if problems:
        print(f"  发现 {len(problems)} 处问题：")
        for f, h in problems[:20]:
            print(f"    {f} -> {h}")
        return 1
    print("  所有站内链接有效 ✓")
    return 0


def check_content():
    """
    检查每个页面确实含正文。

    ★ 这检查是必要的：开发过程中出现过「body 变量被目录覆盖」
    导致所有文档页只剩目录、正文全空的问题，而链接检查**发现不了**
    （链接全有效，页面却是空壳）。所以必须显式验证正文存在。

    若 site/ 不存在或为空，**返回错误码**（而不是静默通过）——
    否则 CI 中若构建步骤顺序出错，检查会假装成功。
    """
    if not os.path.exists(SITE):
        print("\n内容完整性检查：")
        print("  ⚠️ site/ 不存在 —— 请先运行构建")
        return 1

    html_files = [f for f in os.listdir(SITE) if f.endswith(".html")]
    if not html_files:
        print("\n内容完整性检查：")
        print("  ⚠️ site/ 中没有 HTML 文件 —— 构建可能失败")
        return 1

    print("\n内容完整性检查：")
    bad = []
    for f in sorted(html_files):
        txt = io.open(os.path.join(SITE, f), encoding="utf-8").read()

        i = txt.find("<main>")
        j = txt.find("</main>")
        if i < 0 or j < 0:
            bad.append((f, "缺少 <main> 区"))
            continue
        inner = txt[i + 6:j]

        # 去掉目录块后统计正文
        body_only = re.sub(r'<details class="tocbox".*?</details>', "",
                           inner, flags=re.S)
        # 统计正文里的块级元素
        blocks = (body_only.count("<p>") + body_only.count("<h2")
                  + body_only.count("<h3") + body_only.count("<table>")
                  + body_only.count("<pre>"))
        size = len(body_only.strip())

        if blocks < 3 or size < 500:
            bad.append((f, f"正文过少（块元素 {blocks}，{size} 字符）"))
        else:
            print(f"  {f:<16} {size:>7} 字符  {blocks:>3} 个块元素 ✓")

    if bad:
        print(f"\n  ⚠️ {len(bad)} 个页面内容异常：")
        for f, why in bad:
            print(f"    {f}: {why}")
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serve", action="store_true", help="构建后启动本地预览")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--check", action="store_true", help="只做链接检查")
    args = ap.parse_args()

    print("=" * 78)
    print("GitHub Pages 站点生成器")
    print("=" * 78)

    if args.check:
        rc = check_links()
        rc |= check_content()
        return rc

    rendered, missing = build()

    print(f"\n[1] 渲染文档")
    print(f"    {rendered} 篇文档")
    if missing:
        print(f"    ⚠️ 缺失 {len(missing)} 篇：{missing}")

    print(f"\n[2] 生成页面")
    n_html = len([f for f in os.listdir(SITE) if f.endswith(".html")])
    print(f"    {n_html} 个 HTML 页面")
    print(f"    输出目录：{SITE}")

    rc = check_links()
    rc |= check_content()

    print(f"\n[3] 部署方式")
    print("    方式 A（推荐）：GitHub Actions 自动部署")
    print("      已生成 .github/workflows/pages.yml")
    print("      push 到 main 后自动构建并发布")
    print()
    print("    方式 B：手动部署到 gh-pages 分支")
    print("      git subtree push --prefix site origin gh-pages")
    print()
    print("    方式 C：本地预览")
    print(f"      python tools/build_site.py --serve")

    if args.serve:
        import http.server
        import socketserver
        os.chdir(SITE)
        handler = http.server.SimpleHTTPRequestHandler
        with socketserver.TCPServer(("", args.port), handler) as httpd:
            print(f"\n本地预览：http://localhost:{args.port}")
            print("Ctrl+C 停止")
            try:
                httpd.serve_forever()
            except KeyboardInterrupt:
                print("\n已停止")

    return rc


if __name__ == "__main__":
    sys.exit(main())
