#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description="Load Sunray FSM YAML into /<uav_name><uav_id>."
    )
    parser.add_argument(
        "--input-yaml",
        required=True,
        help="Path to source YAML (e.g. control/uav_control/config/sunray_fsm_config.yaml)",
    )
    parser.add_argument(
        "--uav-name",
        default="uav",
        help="UAV name prefix (e.g. 'uav').",
    )
    parser.add_argument(
        "--uav-id",
        default="1",
        help="UAV numeric id (e.g. 1).",
    )
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    input_yaml = os.path.abspath(args.input_yaml)
    if not os.path.exists(input_yaml):
        print("[load_fsm_param] input YAML not found: {}".format(input_yaml), file=sys.stderr)
        return 1

    try:
        uav_id = int(args.uav_id)
    except ValueError:
        print("[load_fsm_param] invalid uav_id: {}".format(args.uav_id), file=sys.stderr)
        return 1

    uav_name = args.uav_name
    uav_ns = "{}{}".format(uav_name, uav_id)
    target_ns = "/{}".format(uav_ns)

    with open(input_yaml, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        data = {}
    if not isinstance(data, dict):
        print("[load_fsm_param] YAML root must be a mapping/dict", file=sys.stderr)
        return 1

    print("[load_fsm_param] input YAML: {}".format(input_yaml))
    print("[load_fsm_param] target namespace: {}".format(target_ns))

    try:
        subprocess.check_call(["rosparam", "load", input_yaml, target_ns])
    except subprocess.CalledProcessError as e:
        print("[load_fsm_param] rosparam command failed: {}".format(e), file=sys.stderr)
        return e.returncode
    print("[load_fsm_param] rosparam load succeeded")

    return 0


if __name__ == "__main__":
    sys.exit(main())
