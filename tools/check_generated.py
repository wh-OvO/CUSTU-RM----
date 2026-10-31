#!/usr/bin/env python3
"""
CubeMX 生成物自检 / 自动修复

背景：STM32CubeMX 6.12.x 生成 CMake 工程时，链接脚本里的内存区域名会被吞掉，
    导致链接报 "syntax error"。CMakeLists.txt 还会把 enable_language() 放在
    project() 之前。两者都是生成器缺陷，每次点 GENERATE CODE 都会复发。

用法：在工程根目录执行
    python tools/check_generated.py          # 只检查
    python tools/check_generated.py --fix    # 检查并修复

退出码：0 = 无问题或已修复；1 = 发现问题且未修复（未加 --fix）
"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
LD_GLOBS = ("*.ld",)


def scan_ld(path: pathlib.Path):
    """返回 (问题列表, 修复后的行列表)"""
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines(keepends=True)
    problems = []
    fixed = list(lines)

    for i, raw in enumerate(lines):
        line = raw.rstrip("\r\n")
        eol = raw[len(line):]

        # 1) _estack = ORIGIN() + LENGTH();
        if re.search(r"_estack\s*=\s*ORIGIN\(\)\s*\+\s*LENGTH\(\)", line):
            problems.append((i + 1, line.strip(), "ORIGIN(RAM) + LENGTH(RAM) 区域名丢失"))
            fixed[i] = line.replace("ORIGIN() + LENGTH()", "ORIGIN(RAM) + LENGTH(RAM)") + eol
            continue

        # 2) } > AT> FLASH
        if re.match(r"^\s*\}\s*>\s*AT>\s*FLASH\s*$", line):
            problems.append((i + 1, line.strip(), "缺少 RAM 区域名"))
            fixed[i] = re.sub(r"^\s*\}\s*>\s*AT>", "  } >RAM AT>", line) + eol
            continue

        # 3) 行尾孤立的 } >
        if re.match(r"^\s*\}\s*>\s*$", line):
            problems.append((i + 1, line.strip(), "缺少区域名（应为 >RAM）"))
            fixed[i] = re.sub(r"^\s*\}\s*>\s*$", "  } >RAM", line) + eol

    return problems, fixed


def scan_cmakelists(path: pathlib.Path):
    """检查 enable_language() 是否早于 project()"""
    if not path.exists():
        return []
    problems = []
    en_line = None
    proj_line = None
    for n, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        if en_line is None and re.match(r"enable_language\s*\(", stripped):
            en_line = n
        if proj_line is None and re.match(r"project\s*\(", stripped):
            proj_line = n
    if en_line and proj_line and en_line < proj_line:
        problems.append((path.name, en_line, "enable_language() 出现在 project() 之前"))
    return problems


def main():
    do_fix = "--fix" in sys.argv
    found = 0
    repaired = 0

    ld_files = [p for g in LD_GLOBS for p in ROOT.glob(g)]
    if not ld_files:
        print("⚠️  没找到 .ld 链接脚本，跳过")

    for ld in ld_files:
        problems, fixed = scan_ld(ld)
        if not problems:
            print(f"✅ {ld.name}: 正常")
            continue
        found += len(problems)
        print(f"\n❌ {ld.name}: 发现 {len(problems)} 处问题")
        for line_no, content, why in problems:
            print(f"   行 {line_no}: {why}")
            print(f"           {content}")
        if do_fix:
            ld.write_text("".join(fixed), encoding="utf-8")
            repaired += len(problems)
            print(f"   → 已修复 {len(problems)} 处")

    cm_problems = scan_cmakelists(ROOT / "CMakeLists.txt")
    if cm_problems:
        for name, line_no, why in cm_problems:
            print(f"\n❌ {name}: 行 {line_no} {why}")
            print("   建议改成：project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM) 并删掉 enable_language 那行")
        found += len(cm_problems)
    else:
        print("✅ CMakeLists.txt: 顺序正常")

    print()
    if found == 0:
        print("结论：生成物干净，可以编译")
        return 0
    if repaired == found:
        print(f"结论：共修复 {repaired} 处，可以编译")
        return 0
    print(f"结论：共发现 {found} 处问题，修复 {repaired} 处；加 --fix 可自动修复链接脚本")
    return 1


if __name__ == "__main__":
    sys.exit(main())
