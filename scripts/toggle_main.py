# -*- coding: utf-8 -*-
# toggle_main.py — 切换测试/demo 文件的 main 注释状态（同一时刻全工程只保留一个激活 main）
# 用法: python scripts/toggle_main.py <文件.cpp> on|off
import re
import sys
import io

def toggle(path, enable):
    with io.open(path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    out, in_main, depth = [], False, 0
    for ln in lines:
        if not in_main:
            m = re.match(r'^(\s*)(//\s*)?int main\(\)', ln)
            if m:
                in_main = True
                depth = 1
                # 保留行尾剩余内容（如 " {"）
                rest = ln[m.end():].rstrip('\r\n')
                out.append(m.group(1) + ('int main()' if enable else '//int main()') + rest + '\n')
                continue
            out.append(ln)
            continue
        # main 函数体内：逐行切换注释（保留行内的真实注释）
        if enable:
            newln = re.sub(r'^(\s*)//', r'\1', ln)
        else:
            newln = re.sub(r'^(\s*)(?=\S)', r'\1//', ln)
        out.append(newln)
        depth += newln.count('{') - newln.count('}')
        if depth <= 0:
            in_main = False
    with io.open(path, 'w', encoding='utf-8', newline='') as f:
        f.writelines(out)

if __name__ == '__main__':
    toggle(sys.argv[1], sys.argv[2] == 'on')
