import re

with open("autopilot.py", "r") as f:
    code = f.read()

# 1. Blind Spot timeouts (80 -> 120)
code = re.sub(r'timeout_counter > 80:(.*?)di blind spot', r'timeout_counter > 120:\1di blind spot', code, flags=re.DOTALL)

# 2. Precision Centering hold timeouts (50 -> 100)
code = code.replace("if timeout_counter > 50: # Stabil di tengah selama 5 detik", "if timeout_counter > 100: # Stabil di tengah selama 10 detik nyata (RTF 30%)")
code = code.replace("if timeout_counter > 50: # Stabil 5 detik", "if timeout_counter > 100: # Stabil 10 detik nyata (RTF 30%)")
# Line 864: `if timeout_counter > 50:` (precision landing timeout)
code = re.sub(r'if timeout_counter > 50:\n(\s+)print\("\[AUTOPILOT\] Waktu habis, mendarat darurat', r'if timeout_counter > 100:\n\1print("[AUTOPILOT] Waktu habis, mendarat darurat', code)

# 3. Line lost timeouts (100 -> 150)
code = code.replace("timeout_counter > 100: # Garis hilang selama ~10 detik nyata (RTF 40% = ~4 detik sim)", "timeout_counter > 150: # Garis hilang (RTF 30% scale)")
code = code.replace("timeout_counter > 100: # RTF 40% x 10s = 4s sim", "timeout_counter > 150: # RTF 30% scale")
code = code.replace("timeout_counter > 100: # 60 derajat maximum", "timeout_counter > 150: # 60 derajat maximum")

# 4. Turn timeouts (80 -> 120)
code = code.replace("timeout_counter > 80: # ~5 detik (Maksimum turning limit)", "timeout_counter > 120: # Maksimum turning limit (RTF 30% scale)")

with open("autopilot.py", "w") as f:
    f.write(code)
