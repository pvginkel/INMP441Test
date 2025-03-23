import os
import sys

env_vars = ["WIFI_PASSWORD"]

content = "#pragma once\n\n"

for var in env_vars:
    value = os.getenv(var, "")
    value = value.replace("\\", "\\\\")
    content += f'#define {var} "{value}"\n'

current_content = ""

if os.path.exists(sys.argv[1]):
    with open(sys.argv[1], "r") as file:
        current_content = file.read()

if current_content != content:
    with open(sys.argv[1], "w") as file:
        file.write(content)
