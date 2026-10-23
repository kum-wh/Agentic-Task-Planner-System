from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

from .agents import MultiAgentPlanner
from .camera import ceiling_for_gsd
from .config import load_config
from .kml_parser import parse_kml
from .qgc_plan import build_baseline_plan, build_qgc_plan, split_plan_by_drone


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a multi-drone QGroundControl Plan from KML")
    parser.add_argument("kml", type=Path, nargs="?")
    parser.add_argument("--config", type=Path, default=Path("config.example.json"))
    parser.add_argument("--output", type=Path, default=Path("mission.plan"))
    parser.add_argument("--baseline-output", type=Path)
    parser.add_argument("--scan-type", choices=["lawnmower", "voronoi"], help="Override scan_type from the config file")
    parser.add_argument("--alt", type=float, help="Override scan_altitude_m — the one knob every imaging number derives from")
    parser.add_argument("--drone-plans-dir", type=Path, help="Folder to write one .plan file per drone into (default: <output>_drones)")
    parser.add_argument("--ceiling", type=float, metavar="GSD_CM", help="Print the highest altitude that still resolves this GSD (cm/px), then exit")
    args = parser.parse_args()

    if args.ceiling is not None:
        print(f"Max altitude for {args.ceiling:.1f} cm/px GSD: {ceiling_for_gsd(args.ceiling):.1f} m")
        return
    if args.kml is None:
        parser.error("a KML file is required (or use --ceiling)")

    mission = parse_kml(args.kml)
    config = load_config(args.config)
    if args.scan_type:
        config = replace(config, scan_type=args.scan_type)
    if args.alt is not None:
        config = replace(config, scan_altitude_m=args.alt)
    config.validate()

    profile = config.profile
    print(
        f"[camera] alt={profile.altitude_m:.0f} m -> spacing {profile.line_spacing_m:.1f} m, "
        f"GSD {profile.gsd_cm_px:.2f} cm/px, cruise {config.cruise_speed_m_s:.1f} m/s"
    )
    layers = [config.for_drone(index) for index in range(config.number_of_drones)]
    print("[layers] " + " · ".join(
        f"drone-{index} {layer.scan_altitude_m:.0f} m: {layer.gsd_cm_px:.2f} cm/px, "
        f"{layer.lane_spacing_m:.1f} m lanes, {layer.cruise_speed_m_s:.1f} m/s"
        for index, layer in enumerate(layers, start=1)
    ))

    baseline = build_baseline_plan(mission)
    baseline_path = args.baseline_output or args.output.with_name(args.output.stem + ".baseline.plan")
    baseline_path.write_text(json.dumps(baseline, indent=2) + "\n", encoding="utf-8")

    result = MultiAgentPlanner().run(mission, config)
    if not result.guardrail_passed:
        raise RuntimeError("Guardrail rejected the generated plan: " + "; ".join(result.warnings))
    plan = build_qgc_plan(result)

    args.output.write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")

    drone_plans_dir = args.drone_plans_dir or args.output.with_name(args.output.stem + "_drones")
    drone_plans_dir.mkdir(parents=True, exist_ok=True)
    for drone_id, drone_plan in sorted(split_plan_by_drone(plan).items()):
        drone_path = drone_plans_dir / f"drone-{drone_id}.plan"
        drone_path.write_text(json.dumps(drone_plan, indent=2) + "\n", encoding="utf-8")

    print(f"wrote baseline: {baseline_path}")
    print(f"wrote plan: {args.output}")
    print(f"wrote per-drone plans: {drone_plans_dir}")
    for warning in result.warnings:
        print(f"warning: {warning}")
    print("load sar_turn_params.params in QGC (Vehicle Setup > Parameters > Tools) for banked corners "
          "(yaw and bank together)")


if __name__ == "__main__":
    main()
