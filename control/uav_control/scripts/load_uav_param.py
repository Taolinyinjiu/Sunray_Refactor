#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
import tempfile

import yaml


def deep_merge_dict(dst, src):
    for key, value in src.items():
        if key in dst and isinstance(dst[key], dict) and isinstance(value, dict):
            deep_merge_dict(dst[key], value)
        else:
            dst[key] = value
    return dst


class SunrayTemplateLoader(yaml.SafeLoader):
    pass


def construct_mapping_merge_uav_namespace(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        value = loader.construct_object(value_node, deep=deep)

        # The source template may define `uav_namespace` multiple times by design.
        # Merge all blocks into one mapping for conversion.
        if key == "uav_namespace":
            if value is None:
                value = {}
            if not isinstance(value, dict):
                raise ValueError("uav_namespace must be a mapping/dict")
            if key not in mapping:
                mapping[key] = {}
            deep_merge_dict(mapping[key], value)
            continue

        mapping[key] = value

    return mapping


SunrayTemplateLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_mapping_merge_uav_namespace,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build namespaced UAV param YAML and load it into ROS parameter server."
    )
    parser.add_argument(
        "--input-yaml",
        required=True,
        help="Path to source YAML (e.g. config/sim/sunray-150/sunray-150-basic.yaml)",
    )
    parser.add_argument(
        "--uav-name",
        default="__AUTO__",
        help="Override uav_name. Use '__AUTO__' to read from input YAML.",
    )
    parser.add_argument(
        "--uav-id",
        default="__AUTO__",
        help="Override uav_id. Use '__AUTO__' to read from input YAML.",
    )
    parser.add_argument(
        "--output-yaml",
        default="",
        help="Path to generated YAML. If empty, use a temporary file.",
    )
    parser.add_argument(
        "--keep-output-yaml",
        default="true",
        choices=["true", "false"],
        help="Whether to keep generated YAML file after rosparam load.",
    )
    parser.add_argument(
        "--skip-rosparam-load",
        default="false",
        choices=["true", "false"],
        help="Only generate output YAML, do not call 'rosparam load'.",
    )
    args, _ = parser.parse_known_args()
    return args


def to_int_if_possible(value):
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return int(value)
    return int(value)


def main():
    args = parse_args()

    input_yaml = os.path.abspath(args.input_yaml)
    if not os.path.exists(input_yaml):
        print(f"[load_uav_param] Input YAML not found: {input_yaml}", file=sys.stderr)
        return 1

    with open(input_yaml, "r", encoding="utf-8") as f:
        source = yaml.load(f, Loader=SunrayTemplateLoader) or {}

    if not isinstance(source, dict):
        print("[load_uav_param] Input YAML root must be a mapping/dict.", file=sys.stderr)
        return 1

    src_name = source.get("uav_name", "uav")
    src_id = source.get("uav_id", 1)

    uav_name = src_name if args.uav_name == "__AUTO__" else args.uav_name
    uav_id_raw = src_id if args.uav_id == "__AUTO__" else args.uav_id

    try:
        uav_id = to_int_if_possible(uav_id_raw)
    except Exception:
        print(f"[load_uav_param] Invalid uav_id: {uav_id_raw}", file=sys.stderr)
        return 1

    uav_ns = f"{uav_name}{uav_id}"

    namespaced_payload = source.get("uav_namespace", {})
    if namespaced_payload is None:
        namespaced_payload = {}
    if not isinstance(namespaced_payload, dict):
        print("[load_uav_param] 'uav_namespace' must be a mapping/dict.", file=sys.stderr)
        return 1

    global_payload = {
        k: v for k, v in source.items() if k not in {"uav_name", "uav_id", "uav_ns", "uav_namespace"}
    }

    rendered = {
        "uav_name": uav_name,
        "uav_id": uav_id,
        "uav_ns": uav_ns,
        **global_payload,
        uav_ns: namespaced_payload,
    }

    cleanup_after_load = args.keep_output_yaml == "false"
    if args.output_yaml:
        output_yaml = os.path.abspath(args.output_yaml)
        output_dir = os.path.dirname(output_yaml)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        temp_created = False
    else:
        fd, output_yaml = tempfile.mkstemp(prefix="sunray_uav_param_", suffix=".yaml")
        os.close(fd)
        temp_created = True

    with open(output_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(rendered, f, allow_unicode=True, sort_keys=False)

    print(f"[load_uav_param] Rendered YAML: {output_yaml}")
    print(f"[load_uav_param] Namespace: {uav_ns}")

    if args.skip_rosparam_load == "false":
        try:
            subprocess.check_call(["rosparam", "load", output_yaml])
        except subprocess.CalledProcessError as e:
            print(f"[load_uav_param] rosparam load failed: {e}", file=sys.stderr)
            return e.returncode

        print("[load_uav_param] rosparam load succeeded")
    else:
        print("[load_uav_param] Skipped rosparam load")

    if cleanup_after_load or temp_created:
        try:
            os.remove(output_yaml)
            print(f"[load_uav_param] Removed temporary YAML: {output_yaml}")
        except OSError:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
