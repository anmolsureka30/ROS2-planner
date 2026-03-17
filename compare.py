"""
Ablation Study and Comparison Framework for Hybrid A* Planner
Runs systematic parameter sweeps and produces comparison tables.
"""

import os
import sys
import numpy as np
import yaml
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from src.map_handler import MapHandler
from src.hybrid_astar.motion_model import MotionModel, VehicleFootprint
from src.hybrid_astar.heuristic import HeuristicCalculator
from src.hybrid_astar.hybrid_astar import HybridAStar
from src.state import State
from src.hybrid_astar.visualizer import Visualizer
from benchmark import build_planner, load_config


def run_single(config, map_path, start, goal, **overrides):
    """Run a single planning query with overrides, return metrics."""
    try:
        map_handler, planner, visualizer, _ = build_planner(
            config, map_path=map_path, **overrides
        )
        result = planner.plan(start, goal)
        if result:
            path, info = result
            return info
        else:
            return {'success': False, 'search_time': 0, 'path_length': 0,
                    'nodes_expanded': 0, 'min_clearance': 0, 'curvature_sum': 0}
    except Exception as e:
        print(f"  ERROR: {e}")
        return {'success': False, 'search_time': 0, 'path_length': 0,
                'nodes_expanded': 0, 'min_clearance': 0, 'curvature_sum': 0}


def ablation_heuristic(config, output_dir):
    """Compare heuristic types across scenarios."""
    print("\n" + "=" * 70)
    print("ABLATION: Heuristic Type Comparison")
    print("=" * 70)

    heuristic_types = ["euclidean", "dubins", "2d_astar", "max"]
    map_path = "maps/map_open.png"
    start = State(10, 50, np.radians(0))
    goal = State(90, 50, np.radians(0))

    results = []
    for h_type in heuristic_types:
        print(f"\n  Testing heuristic: {h_type}")
        info = run_single(config, map_path, start, goal, heuristic_type=h_type)
        info['heuristic'] = h_type
        results.append(info)

    _print_comparison(results, 'heuristic',
                      ['success', 'search_time', 'path_length',
                       'nodes_expanded', 'min_clearance'],
                      "Heuristic Comparison")

    _save_comparison_csv(results, 'heuristic',
                         os.path.join(output_dir, "ablation_heuristic.csv"))
    return results


def ablation_steering_angles(config, output_dir):
    """Compare number of steering angles."""
    print("\n" + "=" * 70)
    print("ABLATION: Steering Angle Count")
    print("=" * 70)

    angles = [3, 5, 7]
    map_path = "maps/map_open.png"
    start = State(15, 15, np.radians(45))
    goal = State(85, 85, np.radians(45))

    results = []
    for n in angles:
        print(f"\n  Testing {n} steering angles")
        info = run_single(config, map_path, start, goal, num_steering=n)
        info['num_steering'] = n
        results.append(info)

    _print_comparison(results, 'num_steering',
                      ['success', 'search_time', 'path_length',
                       'nodes_expanded', 'curvature_sum'],
                      "Steering Angle Count Comparison")

    _save_comparison_csv(results, 'num_steering',
                         os.path.join(output_dir, "ablation_steering.csv"))
    return results


def ablation_resolution(config, output_dir):
    """Compare grid resolution settings."""
    print("\n" + "=" * 70)
    print("ABLATION: Resolution Sensitivity")
    print("=" * 70)

    configs = [
        {"name": "coarse", "xy": 2.0, "theta": 15.0},
        {"name": "medium", "xy": 1.0, "theta": 5.0},
        {"name": "fine", "xy": 0.5, "theta": 3.0},
    ]
    map_path = "maps/map_open.png"
    start = State(10, 50, np.radians(0))
    goal = State(90, 50, np.radians(0))

    results = []
    for cfg in configs:
        print(f"\n  Testing resolution: {cfg['name']} (xy={cfg['xy']}, theta={cfg['theta']})")
        info = run_single(config, map_path, start, goal,
                          resolution_xy=cfg['xy'],
                          resolution_theta=cfg['theta'])
        info['resolution'] = cfg['name']
        info['xy_res'] = cfg['xy']
        info['theta_res'] = cfg['theta']
        results.append(info)

    _print_comparison(results, 'resolution',
                      ['success', 'search_time', 'path_length',
                       'nodes_expanded', 'curvature_sum'],
                      "Resolution Sensitivity")

    _save_comparison_csv(results, 'resolution',
                         os.path.join(output_dir, "ablation_resolution.csv"))
    return results


def _print_comparison(results, key_field, metric_fields, title):
    """Print a formatted comparison table."""
    try:
        from tabulate import tabulate
        headers = [key_field] + metric_fields
        rows = []
        for r in results:
            row = [r.get(key_field, '')]
            for field in metric_fields:
                val = r.get(field, 0)
                if isinstance(val, bool):
                    row.append("Yes" if val else "No")
                elif isinstance(val, float):
                    row.append(f"{val:.4f}")
                else:
                    row.append(str(val))
            rows.append(row)
        print(f"\n{title}")
        print(tabulate(rows, headers=headers, tablefmt='grid'))
    except ImportError:
        print(f"\n{title}:")
        for r in results:
            print(f"  {r.get(key_field, '')}: "
                  f"success={r.get('success', False)}, "
                  f"time={r.get('search_time', 0):.3f}s, "
                  f"length={r.get('path_length', 0):.1f}m, "
                  f"nodes={r.get('nodes_expanded', 0)}")


def _save_comparison_csv(results, key_field, csv_path):
    """Save comparison results to CSV."""
    if not results:
        return

    fieldnames = list(results[0].keys())
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"  Saved: {csv_path}")


def run_all_ablations(config=None, output_dir="results/ablation"):
    """Run all ablation studies."""
    os.makedirs(output_dir, exist_ok=True)

    if config is None:
        config = load_config()

    print("\n" + "=" * 70)
    print("RUNNING ALL ABLATION STUDIES")
    print("=" * 70)

    all_results = {}

    all_results['heuristic'] = ablation_heuristic(config, output_dir)
    all_results['steering'] = ablation_steering_angles(config, output_dir)
    all_results['resolution'] = ablation_resolution(config, output_dir)

    print("\n" + "=" * 70)
    print("ALL ABLATION STUDIES COMPLETE")
    print("=" * 70)

    return all_results


if __name__ == "__main__":
    config = load_config()
    run_all_ablations(config)
