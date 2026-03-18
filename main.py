"""
Main execution script for Hybrid A* and RRT* Family Planners
"""

import argparse
import yaml
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Tuple, List

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.map_handler import MapHandler
from src.hybrid_astar.motion_model import MotionModel, VehicleFootprint
from src.hybrid_astar.heuristic import HeuristicCalculator
from src.hybrid_astar.hybrid_astar import HybridAStar
from src.state import State
from src.hybrid_astar.visualizer import Visualizer
from src.rrt_star.rrt_star import RRTStarPlanner
from src.rrt_star.informed_rrt_star import InformedRRTStarPlanner
from src.rrt_star.bi_rrt_star import BIRRTStarPlanner
from scripts.rrt_visualizer import RRTVisualizer


def load_config(config_path: str = "config/planner_config.yaml") -> Dict:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def setup_map_handler(config: Dict) -> MapHandler:
    """Initialize MapHandler from config."""
    return MapHandler(
        map_path=config['map']['path'],
        resolution=config['map']['resolution'],
        obstacle_threshold=config['map']['obstacle_threshold']
    )


def setup_hybrid_astar(config: Dict, map_handler: MapHandler) -> Tuple[HybridAStar, Visualizer]:
    """Initialize Hybrid A* planner and visualizer."""
    motion_model = MotionModel(
        wheel_base=config['vehicle']['wheel_base'],
        max_steering_angle=config['motion']['max_steering_angle'],
        step_size=config['motion']['step_size'],
        num_steering_angles=config['motion']['num_steering_angles'],
        allow_reverse=config['motion']['allow_reverse']
    )

    footprint = VehicleFootprint(
        length=config['vehicle']['length'],
        width=config['vehicle']['width'],
        rear_axle_to_back=config['vehicle']['rear_axle_to_back']
    )

    heuristic = HeuristicCalculator(
        map_handler=map_handler,
        heuristic_type=config['heuristic']['type'],
        turning_radius=config['heuristic']['turning_radius'],
        cost_alpha=config['heuristic'].get('cost_alpha', 1.0)
    )

    cost_cfg = config.get('cost', {})
    planner = HybridAStar(
        map_handler=map_handler,
        motion_model=motion_model,
        vehicle_footprint=footprint,
        heuristic_calculator=heuristic,
        xy_resolution=config['search']['xy_resolution'],
        theta_resolution=config['search']['theta_resolution'],
        steering_penalty=cost_cfg.get('steering_penalty', 1.5),
        reversing_penalty=cost_cfg.get('reversing_penalty', 2.0),
        steering_change_penalty=cost_cfg.get('steering_change_penalty', 1.5),
        direction_switch_penalty=cost_cfg.get('direction_switch_penalty', 10.0),
        shot_distance=config['search']['shot_distance'],
        max_iterations=config['search'].get('max_iterations', 50000)
    )

    visualizer = Visualizer(map_handler, footprint)
    return planner, visualizer


ALGORITHM_NAMES = {
    '1': 'hybrid_astar',
    '2': 'rrt_star',
    '3': 'informed_rrt_star',
    '4': 'bi_rrt_star',
    '5': 'compare_all',
}


def select_algorithm() -> str:
    """Prompt user to select a planning algorithm."""
    print("\n" + "=" * 60)
    print("SELECT ALGORITHM")
    print("=" * 60)
    print("1. Hybrid A* (graph-based, non-holonomic)")
    print("2. RRT* (sampling-based, asymptotically optimal)")
    print("3. Informed RRT* (ellipsoidal focused sampling)")
    print("4. BI-RRT* (bidirectional + pruning + informed)")
    print("5. Compare ALL RRT* variants")

    while True:
        choice = input("\nEnter choice [1-5] (default 1): ").strip()
        if not choice:
            return 'hybrid_astar'
        if choice in ALGORITHM_NAMES:
            return ALGORITHM_NAMES[choice]
        print("Invalid selection. Please try again.")


def select_heuristic() -> str:
    """Prompt user to select a heuristic type (for Hybrid A*)."""
    print("\n" + "=" * 60)
    print("SELECT HEURISTIC")
    print("=" * 60)
    print("1. Euclidean (Fastest, ignores obstacles)")
    print("2. Dubins/Reeds-Shepp (Non-holonomic, ignores obstacles)")
    print("3. 2D A* (Cost-Aware holonomic, considers obstacles)")
    print("4. Max (RECOMMENDED: max(RS, 2D A*) - tightest bound)")

    while True:
        choice = input("\nEnter choice [1-4] (default 4): ").strip()
        if not choice:
            return "max"
        if choice == "1":
            return "euclidean"
        elif choice == "2":
            return "dubins"
        elif choice == "3":
            return "2d_astar"
        elif choice == "4":
            return "max"
        else:
            print("Invalid selection. Please try again.")


def create_rrt_planner(algorithm: str, map_handler: MapHandler, config: Dict):
    """Factory function for RRT* family planners."""
    planners = {
        'rrt_star': RRTStarPlanner,
        'informed_rrt_star': InformedRRTStarPlanner,
        'bi_rrt_star': BIRRTStarPlanner,
    }
    cls = planners.get(algorithm)
    if cls is None:
        raise ValueError(f"Unknown algorithm: {algorithm}")
    return cls(map_handler, config)


def get_state_from_clicks(ax, name: str, color: str) -> State:
    """Get state from user clicks on plot."""
    print(f"\nClick to set {name} POSITION...")
    ax.set_title(f"Click {name} POSITION", fontsize=14, color=color, fontweight='bold')
    plt.draw()

    pts = plt.ginput(1, timeout=0)
    if not pts:
        raise KeyboardInterrupt("Selection cancelled")
    x, y = pts[0]

    ax.plot(x, y, 'o', color=color, markersize=8)
    plt.draw()

    print(f"Click to set {name} HEADING (click a point to look towards)...")
    ax.set_title(f"Click {name} HEADING", fontsize=14, color=color, fontweight='bold')
    plt.draw()

    pts = plt.ginput(1, timeout=0)
    if not pts:
        raise KeyboardInterrupt("Selection cancelled")
    hx, hy = pts[0]

    theta = np.arctan2(hy - y, hx - x)

    arrow_len = 5.0
    ax.arrow(x, y, arrow_len * np.cos(theta), arrow_len * np.sin(theta),
             head_width=1.5, color=color, zorder=10)
    plt.draw()

    return State(x, y, theta)


def run_and_visualize(algorithm: str, config: Dict, map_handler: MapHandler,
                       start: State, goal: State) -> Tuple[List[State], Dict]:
    """Run a planner and return results."""
    if algorithm == 'hybrid_astar':
        planner, _ = setup_hybrid_astar(config, map_handler)
        result = planner.plan(start, goal)
        if result is None:
            return [], {'success': False, 'algorithm': 'Hybrid A*'}
        path, info = result
        info['algorithm'] = 'Hybrid A*'
        return path, info
    else:
        planner = create_rrt_planner(algorithm, map_handler, config)
        result = planner.plan(start, goal)
        if result is None:
            return [], {'success': False, 'algorithm': algorithm}
        return result


def visualize_result(algorithm: str, map_handler: MapHandler, config: Dict,
                      start: State, goal: State,
                      path: List[State], info: Dict,
                      planner=None) -> None:
    """Visualize planning result."""
    save_path = None
    if config['visualization']['save_results']:
        save_path = os.path.join(
            config['visualization']['results_dir'],
            f"{algorithm}.png"
        )

    if algorithm == 'hybrid_astar':
        footprint = VehicleFootprint(
            length=config['vehicle']['length'],
            width=config['vehicle']['width'],
            rear_axle_to_back=config['vehicle']['rear_axle_to_back']
        )
        vis = Visualizer(map_handler, footprint)
        vis.plot_results(start=start, goal=goal, path=path,
                         info=info, save_path=save_path, show_vehicle=True)
    else:
        rrt_vis = RRTVisualizer(map_handler)
        tree = None
        if planner is not None and hasattr(planner, 'tree') and planner.tree is not None:
            tree = planner.tree
        ellipse_params = info.get('ellipse_params')
        rrt_vis.plot_results(start=start, goal=goal, path=path, info=info,
                              tree=tree, ellipse_params=ellipse_params,
                              save_path=save_path)

    if save_path:
        print(f"\nResult saved to {save_path}")


def run_batch_scenarios(config: Dict):
    """Run pre-defined batch scenarios."""
    map_handler = setup_map_handler(config)

    scenarios = [
        {
            "name": "scenario_1_simple",
            "start": State(10, 10, np.radians(0)),
            "goal": State(80, 80, np.radians(90))
        },
        {
            "name": "scenario_2_heading",
            "start": State(10, 10, np.radians(0)),
            "goal": State(50, 50, np.radians(180))
        },
        {
            "name": "scenario_3_maze",
            "start": State(30, 70, np.radians(0)),
            "goal": State(70, 30, np.radians(90))
        }
    ]

    algorithm = config.get('algorithm', {}).get('type', 'hybrid_astar')

    print("\n" + "=" * 70)
    print(f"RUNNING BATCH SCENARIOS ({algorithm})")
    print("=" * 70)

    for scenario in scenarios:
        print(f"\nScenario: {scenario['name']}")
        print("=" * 60)

        path, info = run_and_visualize(
            algorithm, config, map_handler,
            scenario['start'], scenario['goal'])

        if info.get('success'):
            print("Path found!")
            visualize_result(algorithm, map_handler, config,
                              scenario['start'], scenario['goal'],
                              path, info)
            print("Close the plot window to continue.")
            plt.ioff()
            plt.show(block=True)
        else:
            print("Planning failed!")


def interactive_mode():
    """Interactive mode with algorithm selection and graphical start/goal."""
    config = load_config()
    algorithm = select_algorithm()

    if algorithm == 'hybrid_astar':
        config['heuristic']['type'] = select_heuristic()
        print(f"Selected: Hybrid A* with {config['heuristic']['type']} heuristic")
    elif algorithm == 'compare_all':
        print("Selected: Compare ALL RRT* variants")
    else:
        print(f"Selected: {algorithm}")

    map_handler = setup_map_handler(config)

    while True:
        try:
            print("\n" + "-" * 70)
            print("Click on the map to select start and goal states.")
            print("-" * 70)

            fig, ax = plt.subplots(figsize=(12, 12))
            map_handler.plot(ax)
            ax.set_title("Path Planner - Interactive Selection", fontsize=16)
            plt.tight_layout()

            try:
                plt.show(block=False)
                plt.pause(0.1)
            except Exception:
                pass

            start = get_state_from_clicks(ax, "START", "green")
            goal = get_state_from_clicks(ax, "GOAL", "red")

            print(f"\nStart: {start}")
            print(f"Goal:  {goal}")
            plt.close(fig)

            if algorithm == 'compare_all':
                # Run all RRT* variants and compare
                rrt_algorithms = ['rrt_star', 'informed_rrt_star',
                                   'bi_rrt_star']
                results = {}
                for alg in rrt_algorithms:
                    print(f"\nRunning {alg}...")
                    path, info = run_and_visualize(
                        alg, config, map_handler, start, goal)
                    results[info.get('algorithm', alg)] = (path, info)

                # Comparison visualization
                rrt_vis = RRTVisualizer(map_handler)
                save_path = None
                if config['visualization']['save_results']:
                    save_path = os.path.join(
                        config['visualization']['results_dir'],
                        "comparison_rrt_star.png"
                    )
                rrt_vis.plot_comparison(results, start, goal,
                                         save_path=save_path)

                # Cost convergence plot
                fig2, ax2 = plt.subplots(figsize=(10, 6))
                colors = ['blue', 'red', 'green', 'purple']
                for idx, (name, (_, info)) in enumerate(results.items()):
                    ch = info.get('cost_history', [])
                    if ch:
                        rrt_vis.plot_cost_convergence(
                            ax2, ch, label=name,
                            color=colors[idx % len(colors)])
                ax2.set_title('Cost Convergence Comparison')

                if config['visualization']['save_results']:
                    conv_path = os.path.join(
                        config['visualization']['results_dir'],
                        "convergence_rrt_star.png"
                    )
                    plt.savefig(conv_path, dpi=150, bbox_inches='tight')

                plt.ioff()
                plt.show(block=True)

            else:
                # Single algorithm
                print(f"\nRunning {algorithm}...")

                if algorithm == 'hybrid_astar':
                    path, info = run_and_visualize(
                        algorithm, config, map_handler, start, goal)
                    planner_obj = None
                else:
                    planner_obj = create_rrt_planner(algorithm, map_handler, config)
                    result = planner_obj.plan(start, goal)
                    if result:
                        path, info = result
                    else:
                        path, info = [], {'success': False, 'algorithm': algorithm}

                if info.get('success'):
                    visualize_result(algorithm, map_handler, config,
                                      start, goal, path, info,
                                      planner=planner_obj)
                    print("Close the plot window to continue.")
                    plt.ioff()
                    plt.show(block=True)
                else:
                    print("Planning failed. Try different start/goal positions.")

            cont = input("\nRun another simulation? (y/n): ").lower()
            if cont != 'y':
                break

        except KeyboardInterrupt:
            print("\nExiting interactive mode...")
            break
        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()
            break


def main():
    parser = argparse.ArgumentParser(
        description="Hybrid A* & RRT* Family Path Planner")
    parser.add_argument("--batch", "-b", action="store_true",
                        help="Run pre-defined batch scenarios")
    parser.add_argument("--algorithm", "-a", type=str, default=None,
                        help="Algorithm: hybrid_astar, rrt_star, "
                             "informed_rrt_star, bi_rrt_star, bit_star")

    args = parser.parse_args()

    os.makedirs("results", exist_ok=True)

    if args.batch:
        config = load_config()
        if args.algorithm:
            config['algorithm'] = {'type': args.algorithm}
        run_batch_scenarios(config)
    else:
        print("\n" + "=" * 70)
        print("PATH PLANNER - INTERACTIVE MODE")
        print("=" * 70)
        interactive_mode()


if __name__ == "__main__":
    main()
