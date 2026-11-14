"""Posts a failed test run as GitHub annotations (readable without signing in): each failure, and the end of the output."""
import re
import sys

text = open(sys.argv[1], encoding="utf-8", errors="replace").read().replace("\r", "")


def post(title: str, body: str) -> None:
    print(f"::error title={title}::" + body[-3000:].replace("%", "%25").replace("\n", "%0A"))


for name, body in re.findall(r"_{5,} (\S+) _{5,}\n(.*?)(?=\n_{5,} |\n={5,})", text, re.S)[:8]:
    post(name, body)
# A crash (segfault) ends the run without a failure section: the fault
# handler's stack names the test, above the pytest frames the tail shows.
crash = text.find("Fatal Python error")
if crash >= 0:
    lines = text[crash:].splitlines()
    own = [line for line in lines if "/tests/" in line or "\\tests\\" in line or "weaveengine" in line]
    post("crash", "\n".join(lines[:6] + ["..."] + own[:40]))
post("end of the test output", "\n".join(text.splitlines()[-40:]))
