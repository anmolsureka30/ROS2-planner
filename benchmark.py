"""
Comprehensive Benchmark Runner for Hybrid A* Planner
Runs scenarios across maps, collects metrics, outputs CSV + formatted tables.
"""

import os
import sys
import time
import csv
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

from src.map_handler import MapHandler
from src.hybrid_astar.motion_model import MotionModel, VehicleFootprint
from src.hybrid_astar.heuristic import HeuristicCalculator
from src.hybrid_astar.hybrid_astar import HybridAStar
from src.state import State
from src.hybrid_astar.visualizer import Visualizer


def load_config(config_path="config/planner_config.yaml"):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def build_planner(config, map_path=None, heuristic_type=None,
                  num_steering=None, resolution_xy=None,
                  resolution_theta=None):
    """Build planner with optional parameter overrides."""
    cfg = dict(config)
    cfg['map'] = dict(config['map'])
    cfg['motion'] = dict(config['motion'])
    cfg['search'] = dict(config['search'])
    cfg['cost'] = dict(config.get('cost', {}))
    cfg['heuristic'] = dict(config['heuristic'])

    if map_path:
        cfg['map']['path'] = map_path
    if num_steering:
        cfg['motion']['num_steering_angles'] = num_steering
    if resolution_xy:
        cfg['search']['xy_resolution'] = resolution_xy
    if resolution_theta:
        cfg['search']['theta_resolution'] = resolution_theta
    if heuristic_type:
        cfg['heuristic']['type'] = heuristic_type
    map_handler = MapHandler(
        map_path=cfg['map']['path'],
        resolution=cfg['map']['resolution'],
        obstacle_threshold=cfg['map']['obstacle_threshold']
    )

    motion_model = MotionModel(
        wheel_base=cfg['vehicle']['wheel_base'],
        max_steering_angle=cfg['motion']['max_steering_angle'],
        step_size=cfg['motion']['step_size'],
        num_steering_angles=cfg['motion']['num_steering_angles'],
        allow_reverse=cfg['motion']['allow_reverse']
    )

    footprint = VehicleFootprint(
        length=cfg['vehicle']['length'],
        width=cfg['vehicle']['width'],
        rear_axle_to_back=cfg['vehicle']['rear_axle_to_back']
    )

    cost_cfg = cfg.get('cost', {})

    heuristic = HeuristicCalculator(
        map_handler=map_handler,
        heuristic_type=cfg['heuristic']['type'],
        turning_radius=cfg['heuristic']['turning_radius'],
        cost_alpha=cfg['heuristic'].get('cost_alpha', 1.0)
    )

    planner = HybridAStar(
        map_handler=map_handler,
        motion_model=motion_model,
        vehicle_footprint=footprint,
        heuristic_calculator=heuristic,
        xy_resolution=cfg['search']['xy_resolution'],
        theta_resolution=cfg['search']['theta_resolution'],
        steering_penalty=cost_cfg.get('steering_penalty', 1.5),
        reversing_penalty=cost_cfg.get('reversing_penalty', 2.0),
        steering_change_penalty=cost_cfg.get('steering_change_penalty', 1.5),
        direction_switch_penalty=cost_cfg.get('direction_switch_penalty', 10.0),
        shot_distance=cfg['search']['shot_distance'],
        max_iterations=cfg['search'].get('max_iterations', 50000)
    )

    visualizer = Visualizer(map_handler, footprint)

    return map_handler, planner, visualizer, footprint


def get_scenarios():
    """Define all benchmark scenarios."""
    return {
        "map_open.png": [
            {
                "name": "S1_straight_run",
                "start": State(10, 50, np.radians(0)),
                "goal": State(90, 50, np.radians(0)),
                "description": "Straight run across open field"
            },
            {
                "name": "S2_diagonal_avoidance",
                "start": State(15, 15, np.radians(45)),
                "goal": State(85, 85, np.radians(45)),
                "description": "Diagonal with obstacle avoidance"
            },
            {
                "name": "S3_u_turn",
                "start": State(50, 50, np.radians(0)),
                "goal": State(50, 50, np.radians(180)),
                "description": "U-turn maneuver"
            },
        ],
        "map_maze_gen.png": [
            {
                "name": "S4_maze_traverse",
                "start": State(15, 15, np.radians(0)),
                "goal": State(85, 85, np.radians(0)),
                "description": "Full maze traversal"
            },
            {
                "name": "S5_corridor_nav",
                "start": State(10, 50, np.radians(0)),
                "goal": State(90, 50, np.radians(0)),
                "description": "Navigate through corridors"
            },
        ],
        "map_parking.png": [
            {
                "name": "S6_parking_approach",
                "start": State(15, 50, np.radians(0)),
                "goal": State(80, 20, np.radians(90)),
                "description": "Navigate to parking spot"
            },
            {
                "name": "S7_three_point_turn",
                "start": State(50, 65, np.radians(0)),
                "goal": State(50, 65, np.radians(180)),
                "description": "Three-point turn in parking lot"
            },
        ],
        "map_warehouse.png": [
            {
                "name": "S8_warehouse_traverse",
                "start": State(15, 50, np.radians(0)),
                "goal": State(85, 50, np.radians(0)),
                "description": "Navigate warehouse aisles"
            },
        ],
    }


def run_scenario(planner, start, goal, num_runs=3):
    """Run a single scenario multiple times and return averaged metrics."""
    results = []
    for run in range(num_runs):
        result = planner.plan(start, goal)
        if result is not None:
            path, info = result
            results.append(info)
        else:
            results.append({
                'success': False,
                'nodes_expanded': 0,
                'nodes_visited': 0,
                'search_time': 0,
                'path_length': 0,
                'analytical': False,
            })

    # Average metrics
    if not results:
        return None

    avg = {}
    success_count = sum(1 for r in results if r.get('success', False))
    avg['success_rate'] = success_count / len(results)
    avg['success'] = success_count > 0

    numeric_keys = ['nodes_expanded', 'nodes_visited', 'search_time',
                    'path_length']
    successful = [r for r in results if r.get('success', False)]

    if successful:
        for key in numeric_keys:
            values = [r.get(key, 0) for r in successful]
            avg[key] = np.mean(values)
            avg[f'{key}_std'] = np.std(values) if len(values) > 1 else 0
        avg['analytical'] = any(r.get('analytical', False) for r in successful)
    else:
        for key in numeric_keys:
            avg[key] = 0
            avg[f'{key}_std'] = 0
        avg['analytical'] = False

    return avg


def run_full_benchmark(config, output_dir="results/benchmark"):
    """Run all scenarios and produce results."""
    os.makedirs(output_dir, exist_ok=True)

    scenarios = get_scenarios()
    all_results = []

    for map_file, scenario_list in scenarios.items():
        map_path = os.path.join("maps", map_file)
        if not os.path.exists(map_path):
            print(f"WARNING: Map {map_path} not found, skipping...")
            continue

        print(f"\n{'='*70}")
        print(f"MAP: {map_file}")
        print(f"{'='*70}")

        try:
            map_handler, planner, visualizer, footprint = build_planner(
                config, map_path=map_path
            )
        except Exception as e:
            print(f"  ERROR building planner for {map_file}: {e}")
            continue

        for scenario in scenario_list:
            print(f"\n  Scenario: {scenario['name']} - {scenario['description']}")

            result = run_scenario(planner,
                                  scenario['start'], scenario['goal'],
                                  num_runs=3)

            if result:
                result['scenario'] = scenario['name']
                result['map'] = map_file
                result['description'] = scenario['description']
                all_results.append(result)

                # Save visualization for successful runs
                if result['success']:
                    plan_result = planner.plan(scenario['start'], scenario['goal'])
                    if plan_result:
                        path, info = plan_result
                        save_path = os.path.join(output_dir,
                                                 f"{scenario['name']}.png")
                        visualizer.plot_results(
                            start=scenario['start'],
                            goal=scenario['goal'],
                            path=path,
                            info=info,
                            save_path=save_path,
                            show_vehicle=True
                        )
                        plt.close('all')

    # Save results to CSV
    if all_results:
        csv_path = os.path.join(output_dir, "benchmark_results.csv")
        save_results_csv(all_results, csv_path)
        print_results_table(all_results)

    return all_results


def save_results_csv(results, csv_path):
    """Save benchmark results to CSV."""
    fieldnames = [
        'scenario', 'map', 'description', 'success_rate',
        'search_time', 'search_time_std',
        'path_length', 'path_length_std',
        'nodes_expanded', 'nodes_expanded_std',
        'nodes_visited', 'nodes_visited_std',
        'min_clearance', 'min_clearance_std',
        'curvature_sum', 'curvature_sum_std',
        'analytical'
    ]

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            writer.writerow(r)

    print(f"\nResults saved to: {csv_path}")


def print_results_table(results):
    """Print formatted results table."""
    try:
        from tabulate import tabulate
        headers = ['Scenario', 'Success', 'Time (s)', 'Path (m)',
                   'Nodes', 'Clearance (m)', 'Curvature', 'RS Shot']
        rows = []
        for r in results:
            rows.append([
                r['scenario'],
                f"{r['success_rate']*100:.0f}%",
                f"{r['search_time']:.3f}",
                f"{r['path_length']:.1f}",
                f"{int(r['nodes_expanded'])}",
                f"{r['min_clearance']:.2f}",
                f"{r['curvature_sum']:.2f}",
                "Yes" if r.get('analytical') else "No"
            ])
        print(f"\n{'='*90}")
        print("BENCHMARK RESULTS SUMMARY")
        print(f"{'='*90}")
        print(tabulate(rows, headers=headers, tablefmt='grid'))
    except ImportError:
        print("\nResults (install 'tabulate' for formatted table):")
        for r in results:
            print(f"  {r['scenario']}: success={r['success_rate']*100:.0f}%, "
                  f"time={r['search_time']:.3f}s, "
                  f"length={r['path_length']:.1f}m, "
                  f"nodes={int(r['nodes_expanded'])}")


if __name__ == "__main__":
    config = load_config()
    results = run_full_benchmark(config)
