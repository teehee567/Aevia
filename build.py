import shutil

TOOLS = [
    "cargo",
    "flutter",
    "dart"
]

missing = []

for tool in TOOLS:
    if not shutil.which(tool):
        print(f"MISSING: {tool}")
        missing.append(tool)

if missing:
    raise SystemExit(1)
