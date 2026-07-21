"""Run the project's local quality gates in the same order as CI."""

from __future__ import annotations

import subprocess

COMMANDS = (
    ("ruff", "format", "--check", "."),
    ("ruff", "check", "."),
    ("mypy", "src"),
    ("pytest",),
    ("python", "-m", "build"),
)


def main() -> int:
    for command in COMMANDS:
        completed = subprocess.run(command, check=False)
        if completed.returncode:
            return completed.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
