import sys
from typing import Dict, Tuple, FrozenSet, List, Any
import ex1

# ==============================================================================
# Simulation of WateringProblem class (as developed by you)
# ==============================================================================


# ==============================================================================
# Input data and test execution
# ==============================================================================

INIT_STATE_PDF = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (2, 1)},
    "Taps":   {(1, 1): 6},
    "Plants": {(2, 0): 0, (0, 2): 0},
    "Robots": {10: (1, 1, 1, 2), 11: (1, 2, 0, 2)},
}

def run_tests():
    
    print("--- 🔬 WateringProblem Implementation Tests (Problem 1) 🔬 ---")
    
    # 1. Testing init and conversion to Hashable
    problem = ex1.WateringProblem(INIT_STATE_PDF)
    initial_state = problem.initial

    print("test1: init (Hashable)")
    print(f"   initial state: {initial_state}\n\n")
    
    
    # 2. Testing goal_test (initial state)
    print("Test 2: goal_test (initial state)")
    is_goal = problem.goal_test(initial_state)
    print(f"   Is initial state a goal? {is_goal}\n\n")

    
    # 4. Testing successor (initial action count)
    successors = problem.successor(initial_state)
    print(f"test 2: successor (count actions)\n")
    print(f"   sum of actions counted: {len(successors)}\n")
    print(f"   actions: {[action for action, _ in successors]}\n\n")


if __name__ == "__main__":
    run_tests()