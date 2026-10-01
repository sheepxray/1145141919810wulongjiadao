# 固件构建与发布

> STM32G474VETx 运动控制固件：本地 Keil 构建 → 打包 → 上传 GitHub Release

---

## 1. 为什么是「本地构建」而不是 CI 构建

**结论：CI 构建这条路当前走不通，而且没必要。**

| 障碍 | 说明 |
|---|---|
| **Keil 无法在 Linux runner 运行** | 本工程是 Keil MDK（`.uvprojx`），Keil 是 Windows 商业软件，GitHub Actions 的 `ubuntu-latest` 跑不了 |
| **改用 GCC 有个硬缺口** | 工程里**没有 CMSIS 核心头文件**（`core_cm4.h` / `cmsis_gcc.h` / `cmsis_compiler.h`）——Keil 通过 RTE 包管理器（`Cclass="CMSIS" Cgroup="CORE" Cversion="4.3.0"`）从本地已安装的包取，所以没进仓库。换成 GCC 必须额外补这些文件 |
| **换 GCC 还要动链接脚本** | 需自建 `.ld`（本工程无），并验证 `-mfloat-abi=hard` 与 FreeRTOS 的 `ARM_CM4F` port 匹配 |
| **收益低** | 比赛固件发布频率低，本地构建 + 手动上传完全够用 |

> 本地 Keil **已验证可命令行构建**：
> ```
> UV4.exe -b wulong_fw.uvprojx -j0 -o build_log.txt
> → 退出码 0，0 Error(s) 0 Warning(s)，耗时 9 秒
> ```

---

## 2. 一键构建

```cmd
cd firmware\stm32\wulong_fw
build_release.bat                 :: 版本号自动取 git tag
build_release.bat v1.0.0          :: 或显式指定
```

### 前置条件

| 项 | 要求 |
|---|---|
| Keil MDK | 装在 `C:\Keil_v5`（其他路径设环境变量 `KEIL_PATH`） |
| Python | 不需要 |
| gh CLI | 仅上传 Release 时需要 |

### 产出（仓库根目录 `dist/`）

| 文件 | 用途 |
|---|---|
| `wulong_fw_<ver>.hex` | **烧录用**（STM32CubeProgrammer / ST-Link Utility） |
| `wulong_fw_<ver>.bin` | 原始二进制（自定义烧录器 / OTA） |
| `wulong_fw_<ver>.elf` | 调试符号（GDB / Keil） |
| `wulong_fw_<ver>.map` | 链接映射表（分析 Flash/RAM 占用） |
| `SHA256SUMS.txt` | 校验和 |
| `release_notes.md` | 发布说明模板 |

### 构建实测结果（v0.0.1-test）

```
Program Size: Code=35400  RO-data=832  RW-data=252  ZI-data=22196
hex   102672 bytes
bin    36484 bytes
elf  1357112 bytes
map   396805 bytes
```

> Flash 占用 35.4KB / 512KB，RAM 22.4KB / 96KB —— 余量充足。

---

## 3. 上传 Release

### 首次：登录 gh CLI

```sh
gh auth login
```

选 **GitHub.com** → **HTTPS** → **Login with a web browser**（或粘贴 PAT）。

> ⚠️ 这一步必须**你本人**完成（需要浏览器授权）。登录状态存在系统凭据管理器，
> 后续调用无需再次认证。

### 上传

```sh
gh release create v1.0.0 \
  dist/wulong_fw_v1.0.0.hex \
  dist/wulong_fw_v1.0.0.bin \
  dist/wulong_fw_v1.0.0.elf \
  dist/wulong_fw_v1.0.0.map \
  dist/SHA256SUMS.txt \
  --title "固件 v1.0.0" \
  --notes-file dist/release_notes.md
```

### 或者用网页上传

构建完直接打开 https://github.com/sheepxray/1145141919810wulongjiadao/releases/new
把 `dist/` 里的文件拖进去。

---

## 4. 烧录

```sh
STM32_Programmer_CLI -c port=SWD -w wulong_fw_v1.0.0.hex -v -rst
```

### 校验下载的文件

```sh
sha256sum -c SHA256SUMS.txt
```

---

## 5. 脚本设计要点（踩过的坑）

| 坑 | 处理 |
|---|---|
| **`.bat` 不能含非 ASCII** | cmd.exe 在中文 Windows 用 GBK 解析，UTF-8 的中文注释会让**整个脚本语法崩坏**（实测过）。因此 `build_release.bat` 刻意写成纯 ASCII |
| **cmd 里 `\|` 是管道符** | release notes 的表格行 `\|---\|---\|` 即使在括号块内也会被误解析。改用 PowerShell 生成该文件 |
| **中文路径** | 仓库路径含中文时，`cmd /c "build_release.bat"` 会找不到文件。用 PowerShell 调用正常：`powershell -Command "Set-Location '<路径>'; & '.\build_release.bat'"` |
| **Keil 退出码** | 0=无错无警告 1=有警告 2=错误 3=致命 11=打不开工程 20=许可证。脚本对 0/1 放行，其余报错并打印日志尾部 |
| **陈旧产物** | 构建前先删 `.axf`/`.hex`/`.bin`，避免上次产物被误当成新构建 |

---

## 6. 相关文件

| 文件 | 说明 |
|---|---|
| `build_release.bat` | 一键构建 + 打包（**纯 ASCII**） |
| `STM32G474VETx_FLASH.ld` | GCC 链接脚本（备用，CI 路线用；当前未启用） |
| `Makefile` | GCC 构建（备用，CI 路线用；当前未启用） |
| `MDK-ARM/wulong_fw.uvprojx` | Keil 工程（**当前实际使用**） |

> `Makefile` 与 `.ld` 保留是为了将来若要走 CI 路线时有起点，但**目前不用** ——
> 用它们构建会因缺 CMSIS 核心头文件而失败。
