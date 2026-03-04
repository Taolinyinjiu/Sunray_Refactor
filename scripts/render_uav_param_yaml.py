#!/usr/bin/env python3
"""
Render UAV parameter YAML from a simple template.

Template tokens:
- {{UAV_NAME}}
- {{UAV_ID}}
- {{UAV_NS}}
"""

import argparse
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Render UAV parameter YAML template.")
    parser.add_argument(
        "--template",
        default="config/sim/sunray-150/sunray-150-basic.yaml.template",
        help="Template YAML path.",
    )
    parser.add_argument(
        "--output",
        default="config/sim/sunray-150/sunray-150-basic.yaml",
        help="Rendered YAML output path.",
    )
    parser.add_argument("--uav-name", default="uav", help="UAV name prefix, e.g. 'uav'.")
    parser.add_argument("--uav-id", type=int, default=1, help="UAV id, e.g. 1.")
    return parser.parse_args()


def main():
    args = parse_args()
    uav_ns = f"{args.uav_name}{args.uav_id}"

    template_path = Path(args.template)
    output_path = Path(args.output)

    content = template_path.read_text(encoding="utf-8")
    rendered = (
        content.replace("{{UAV_NAME}}", args.uav_name)
        .replace("{{UAV_ID}}", str(args.uav_id))
        .replace("{{UAV_NS}}", uav_ns)
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(rendered, encoding="utf-8")
    print(f"Rendered {output_path} with namespace '{uav_ns}'.")


if __name__ == "__main__":
    main()
