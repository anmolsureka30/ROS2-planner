"""Multi-algorithm comparison benchmarker for RRT* family."""

import time
import numpy as np
from typing import Dict, List, Tuple, Optional

from src.state import State
from src.rrt_star.rrt_star import RRTStarPlanner
from src.rrt_star.informed_rrt_star import InformedRRTStarPlanner
from src.rrt_star.bi_rrt_star import BIRRTStarPlanner


PLANNER_CLASSES = {
    'rrt_star': RRTStarPlanner,
    'informed_rrt_star': InformedRRTStarPlanner,
    'bi_rrt_star': BIRRTStarPlanner,
}


class RRTBenchmarker:
    """Run and compare multiple RRT* family algorithms.

    Args:
        map_handler: MapHandler instance
        config: full config dict
    """

    def __init__(self, map_handler, config: Dict) -> None:
        self._map = map_handler
        self._config = config

    def run_comparison(self, start: State, goal: State,
                       algorithms: Optional[List[str]] = None,
                       num_runs: int = 1) -> Dict[str, Dict]:
        """Run algorithms and collect results.

        Args:
            start: start state
            goal: goal state
            algorithms: list of algorithm names (default: all)
            num_runs: number of runs per algorithm for averaging

        Returns:
            Dict mapping algorithm name -> aggregated info dict
        """
        if algorithms is None:
            algorithms = list(PLANNER_CLASSES.keys())

        results = {}
        for alg_name in algorithms:
            cls = PLANNER_CLASSES.get(alg_name)
            if cls is None:
                print(f"  Unknown algorithm: {alg_name}")
                continue

            times, lengths, sizes = [], [], []
            best_result = None

            for run in range(num_runs):
                planner = cls(self._map, self._config)
                result = planner.plan(start, goal)
                if result is not None:
                    path, info = result
                    times.append(info['search_time'])
                    lengths.append(info['path_length'])
                    sizes.append(info['tree_size'])
                    if best_result is None or info['path_length'] < best_result[1]['path_length']:
                        best_result = (path, info)

            if best_result:
                _, info = best_result
                info['mean_time'] = np.mean(times) if times else 0
                info['std_time'] = np.std(times) if times else 0
                info['mean_length'] = np.mean(lengths) if lengths else 0
                info['success_rate'] = len(times) / num_runs
                results[alg_name] = best_result
            else:
                results[alg_name] = ([], {'success': False, 'algorithm': alg_name})

        return results

    def print_table(self, results: Dict[str, Tuple]) -> None:
        """Print formatted comparison table."""
        print(f"\n{'='*80}")
        print("RRT* Family Comparison")
        print(f"{'='*80}")
        print(f"{'Algorithm':<25} {'Success':>8} {'Time (s)':>10} "
              f"{'Length (m)':>11} {'Tree Size':>10}")
        print(f"{'-'*25} {'-'*8} {'-'*10} {'-'*11} {'-'*10}")

        for name, (path, info) in results.items():
            success = 'Yes' if info.get('success') else 'No'
            t = info.get('search_time', 0)
            length = info.get('path_length', 0)
            tree = info.get('tree_size', 0)
            print(f"{name:<25} {success:>8} {t:>10.3f} "
                  f"{length:>11.2f} {tree:>10}")
        print(f"{'='*80}")
