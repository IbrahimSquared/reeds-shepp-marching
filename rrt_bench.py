#!/usr/bin/env python3
import os
import math
import time
import sys
import csv

import numpy as np
import pandas as pd
from PIL import Image

import matplotlib.pyplot as plt  # only needed if you want to visualize (can be removed)

# OMPL imports
try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    print("Error: OMPL Python bindings could not be imported. Check your installation.")
    sys.exit(1)

# ------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------
PROJECT_ROOT = "/home/ibrahim/projects/reeds-shepp-marching"

IMAGE_PATH = os.path.join(PROJECT_ROOT, "images", "waffle_inflated.jpeg")
RS_RESULTS_CSV = os.path.join(PROJECT_ROOT, "bench_RS_vs_RRT", "RS_benchmark_results.csv")
RRT_RESULTS_CSV = os.path.join(PROJECT_ROOT, "bench_RS_vs_RRT", "RRTstar_benchmark_results.csv")

MIN_TURNING_RADIUS = 10.0      # must match RS / config
ROBOT_WIDTH = 2.0              # in pixels for collision checker
ROBOT_LENGTH = 2.0             # in pixels for collision checker

INITIAL_TIME = 0.01            # seconds for first solution attempt
TOTAL_BUDGET = 0.10            # total planning time budget in seconds

POS_TOL = 1.0                  # pixels: goal position tolerance
YAW_TOL_RAD = math.radians(20) # radians: yaw tolerance

INTERPOLATION_POINTS = 200     # states to interpolate along solution path

RANDOM_SEED = 12345            # for reproducibility

# ------------------------------------------------------------------------
# Global map variables
# ------------------------------------------------------------------------
OBSTACLE_MAP = None  # boolean array [H, W]
WIDTH = 0
HEIGHT = 0
COLLISION_POINTS = None  # footprint corners relative to center


# ------------------------------------------------------------------------
# Map loading and obstacle handling
# ------------------------------------------------------------------------
def load_and_prepare_map(image_path):
    """
    Load occupancy image and create boolean obstacle map:
    True for obstacle, False for free.
    """
    global OBSTACLE_MAP, WIDTH, HEIGHT, COLLISION_POINTS

    img = Image.open(image_path).convert("L")
    map_array = np.array(img)

    # Define obstacles: dark pixels are obstacles
    OBSTACLE_MAP = (map_array < 128).astype(bool)
    HEIGHT, WIDTH = OBSTACLE_MAP.shape
    print(f"[MAP] Loaded occupancy map: {WIDTH} x {HEIGHT} pixels")

    # Rectangular robot footprint (relative to center)
    COLLISION_POINTS = np.array([
        [-ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0],
        [ ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0],
        [ ROBOT_LENGTH / 2.0,  ROBOT_WIDTH / 2.0],
        [-ROBOT_LENGTH / 2.0,  ROBOT_WIDTH / 2.0],
    ])


# ------------------------------------------------------------------------
# OMPL State Validity Checker
# ------------------------------------------------------------------------
class CarStateValidityChecker(ob.StateValidityChecker):
    """
    Uses a rectangular footprint on the occupancy grid.
    """
    def __init__(self, si, collision_points):
        super(CarStateValidityChecker, self).__init__(si)
        self.collision_points = collision_points

    def isValid(self, state):
        x = state.getX()
        y = state.getY()
        theta = state.getYaw()

        # Check bounds for center
        if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
            return False

        # Rotate footprint and check each corner
        c = math.cos(theta)
        s = math.sin(theta)
        R = np.array([[c, -s],
                      [s,  c]])

        pts = (self.collision_points @ R.T) + np.array([x, y])

        for px, py in pts:
            ix = int(round(px))
            iy = int(round(py))
            if 0 <= ix < WIDTH and 0 <= iy < HEIGHT:
                if OBSTACLE_MAP[iy, ix]:
                    return False
            else:
                # Corner out of bounds
                return False

        return True


# ------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------
def make_rrtstar_setup():
    """
    Create OMPL Reeds-Shepp space + SimpleSetup + RRT* planner.
    """
    space = ob.ReedsSheppStateSpace(MIN_TURNING_RADIUS)

    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, 0.0)
    bounds.setHigh(0, WIDTH - 1.0)
    bounds.setLow(1, 0.0)
    bounds.setHigh(1, HEIGHT - 1.0)
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)

    checker = CarStateValidityChecker(ss.getSpaceInformation(), COLLISION_POINTS)
    ss.setStateValidityChecker(checker)

    # Optionally increase checking resolution a bit for safety
    ss.getSpaceInformation().setStateValidityCheckingResolution(0.01)

    planner = og.RRTstar(ss.getSpaceInformation())
    ss.setPlanner(planner)

    # Optimize path length
    objective = ob.PathLengthOptimizationObjective(ss.getSpaceInformation())
    ss.setOptimizationObjective(objective)

    return ss, space


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a


def goal_reached(path, goal_state):
    """
    Check if path endpoint is within position & yaw tolerance of goal.
    goal_state = (x, y, theta)
    """
    if path.getStateCount() == 0:
        return False

    last = path.getState(path.getStateCount() - 1)
    dx = last.getX() - goal_state[0]
    dy = last.getY() - goal_state[1]
    dist = math.hypot(dx, dy)
    dyaw = wrap_angle(last.getYaw() - goal_state[2])

    return (dist <= POS_TOL) and (abs(dyaw) <= YAW_TOL_RAD)


def compute_collisions(path):
    """
    Count how many sampled states of the path fall on occupied cells.
    """
    collisions = 0
    count = path.getStateCount()
    for i in range(count):
        st = path.getState(i)
        x = st.getX()
        y = st.getY()
        ix = int(round(x))
        iy = int(round(y))
        if 0 <= ix < WIDTH and 0 <= iy < HEIGHT:
            if OBSTACLE_MAP[iy, ix]:
                collisions += 1
        else:
            # Out-of-bounds sample counts as collision
            collisions += 1
    return collisions


def path_cost_and_collisions(path, goal_state):
    """
    Return (cost, collisions) where cost is path length if goal reached, else inf.
    """
    if path is None or path.getStateCount() == 0:
        return math.inf, math.nan

    # OMPL path length in Reeds–Shepp metric
    cost = path.length()

    # Check if goal is actually reached
    if not goal_reached(path, goal_state):
        return math.inf, math.nan

    collisions = compute_collisions(path)
    return cost, collisions


def plan_rrtstar_for_pair(start_px, goal_px):
    """
    Run RRT* (Reeds–Shepp) for one start/goal pair with two phases:
    - INITIAL_TIME: initial solution
    - TOTAL_BUDGET: total time (initial + refinement)
    Returns dict with timing / cost / collisions.
    """
    ss, space = make_rrtstar_setup()

    # Set start and goal states
    start = ob.State(space)
    start[0] = start_px[0]
    start[1] = start_px[1]
    start[2] = start_px[2]

    goal = ob.State(space)
    goal[0] = goal_px[0]
    goal[1] = goal_px[1]
    goal[2] = goal_px[2]

    ss.setStartAndGoalStates(start, goal)
    ss.setup()

    # ---------------- Initial solve ----------------
    init_solution_time = math.nan
    init_cost = math.inf

    t0 = time.time()
    solved_initial = ss.solve(INITIAL_TIME)
    t_initial_wall = time.time() - t0

    if solved_initial:
        init_solution_time = ss.getLastPlanComputationTime()
        path_initial = ss.getSolutionPath()
        if INTERPOLATION_POINTS is not None and INTERPOLATION_POINTS > 0:
            path_initial.interpolate(INTERPOLATION_POINTS)
        init_cost, _ = path_cost_and_collisions(path_initial, goal_px)

    # ---------------- Refinement until total budget ----------------
    remaining = max(TOTAL_BUDGET - INITIAL_TIME, 0.0)
    final_solution_time = math.nan
    final_cost = math.inf
    collisions = math.nan

    if remaining > 0.0:
        t1 = time.time()
        solved_final = ss.solve(remaining)
        t_refine_wall = time.time() - t1
        total_wall_time = t_initial_wall + t_refine_wall
    else:
        solved_final = solved_initial
        total_wall_time = t_initial_wall

    if solved_final:
        final_solution_time = init_solution_time + ss.getLastPlanComputationTime() \
                              if not math.isnan(init_solution_time) else ss.getLastPlanComputationTime()

        path_final = ss.getSolutionPath()
        if INTERPOLATION_POINTS is not None and INTERPOLATION_POINTS > 0:
            path_final.interpolate(INTERPOLATION_POINTS)

        final_cost, collisions = path_cost_and_collisions(path_final, goal_px)
    else:
        # No solution at all within TOTAL_BUDGET
        total_wall_time = t_initial_wall  # best we have

    return {
        "initial_solution_time_s": init_solution_time,
        "initial_cost": init_cost,
        "final_solution_time_s": final_solution_time,
        "final_cost": final_cost,
        "total_wall_time_s": total_wall_time,
        "collisions": collisions
    }


# ------------------------------------------------------------------------
# Main: batch over RS_benchmark_results.csv
# ------------------------------------------------------------------------
def main():
    # Seed OMPL RNG for reproducibility (if available)
    try:
        ob.RNG().setSeed(RANDOM_SEED)
        print(f"[RNG] Set OMPL RNG seed to {RANDOM_SEED}")
    except Exception as e:
        print(f"[RNG] Could not set OMPL seed: {e}")

    # Load map once
    load_and_prepare_map(IMAGE_PATH)

    # Load RS benchmark CSV
    if not os.path.exists(RS_RESULTS_CSV):
        print(f"RS results CSV not found: {RS_RESULTS_CSV}")
        sys.exit(1)

    rs_df = pd.read_csv(RS_RESULTS_CSV)

    # Prepare output
    rows_out = []
    print(f"[BENCH] Running RRT* on {len(rs_df)} start/goal pairs...")

    for idx, row in rs_df.iterrows():
        xs = float(row["xs"])
        ys = float(row["ys"])
        thetas_deg = float(row["thetas_deg"])
        xg = float(row["xg"])
        yg = float(row["yg"])
        thetag_deg = float(row["thetag_deg"])

        theta_s = math.radians(thetas_deg)
        theta_g = math.radians(thetag_deg)

        start_state = (xs, ys, theta_s)
        goal_state = (xg, yg, theta_g)

        print(f"\n=== Test {idx+1}/{len(rs_df)} ===")
        print(f"Start: ({xs:.1f}, {ys:.1f}, {thetas_deg:.1f} deg)")
        print(f"Goal : ({xg:.1f}, {yg:.1f}, {thetag_deg:.1f} deg)")

        metrics = plan_rrtstar_for_pair(start_state, goal_state)

        out_row = {
            "xs": xs,
            "ys": ys,
            "thetas_deg": thetas_deg,
            "xg": xg,
            "yg": yg,
            "thetag_deg": thetag_deg,
            "initial_solution_time_s": metrics["initial_solution_time_s"],
            "initial_cost": metrics["initial_cost"],
            "final_solution_time_s": metrics["final_solution_time_s"],
            "final_cost": metrics["final_cost"],
            "total_wall_time_s": metrics["total_wall_time_s"],
            "collisions": metrics["collisions"]
        }

        rows_out.append(out_row)

    out_df = pd.DataFrame(rows_out)
    out_df.to_csv(RRT_RESULTS_CSV, index=False)
    print(f"\n[BENCH] Saved RRT* benchmark results to:\n  {RRT_RESULTS_CSV}")


if __name__ == "__main__":
    main()
