"""Build the Zig/Pydust extension in self-managed mode."""

from __future__ import annotations

import argparse
import sys

from pydust import buildzig, config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="Zig target triple, for example x86_64-linux-gnu or aarch64-linux-gnu")
    parser.add_argument(
        "--optimize",
        default="ReleaseSafe",
        choices=("Debug", "ReleaseSafe", "ReleaseFast", "ReleaseSmall"),
        help="Zig optimization mode",
    )
    args = parser.parse_args()

    zig_args = ["install", f"-Dpython-exe={sys.executable}", f"-Doptimize={args.optimize}"]
    if args.target is not None:
        zig_args.append(f"-Dtarget={args.target}")

    buildzig.zig_build(zig_args, conf=config.load())


if __name__ == "__main__":
    main()
