"""Posts a failed test run as GitHub annotations (readable without signing in): each failure, and the end of the output."""
import re
import sys

text = open(sys.argv[1], encoding="utf-8", errors="replace").read().replace("\r", "")


def post(title: str, body: str) -> None:
    print(f"::error title={title}::" + body[-3000:].replace("%", "%25").replace("\n", "%0A"))


for name, body in re.findall(r"_{5,} (\S+) _{5,}\n(.*?)(?=\n_{5,} |\n={5,})", text, re.S)[:8]:
    post(name, body)
post("end of the test output", "\n".join(text.splitlines()[-40:]))
