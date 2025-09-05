#!/usr/bin/env python3
"""
UTM Normal Operations Dataset Generator
---------------------------------------
Generates a JSONL dataset of (state -> decision) pairs for two roles:
  - "Patrol"  (patrolling UAV coordinator)
  - "Delivery" (delivery UAV coordinator)

This is a headless clone of the boids + priority rules used in your Pygame sim,
with a minimal 2D vector implementation (no external dependencies).
Each simulation tick logs *two* records (one per role).

Output format per line (JSONL):
{
  "role": "Patrol|Delivery",
  "state": {
    "t": <int>,
    "ports": [{"id": 0, "x": ..., "y": ...}, ...],
    "queues": [q0, q1, q2, q3, q4, q5],
    "conflicts": {"recent_window": W},
    "hotspots": [{"port_id": k, "recent_conflicts": c}, ...],
    "uavs_sample": [{"id": 27, "type": "priority|normal", "x": ..., "y": ...}, ...],
    "min_pairwise_sep": <float>,
    "weather": "nominal"
  },
  "decision": {
    "actions": [
      // Patrol normal-ops examples:
      {"action":"grant_corridor","lane_id":"port-1->port-3","ttl_s":90,"conditions":["speed<=vmax"]},
      {"action":"declare_no_go_volume","volume_id":"bubble-port-3","ttl_s":60},

      // Delivery normal-ops examples:
      {"action":"reroute_flows","from_port":"port-3","to_port":"port-1","fraction":0.2},
      {"action":"set_param","target":"Delivery","name":"sep_margin","value":5.0,"ttl_s":120}
    ],
    "rationale": "short sentence for alignment"
  }
}

Usage:
  python generate_utm_normal_ops.py --steps 1500 --window 60 --out utm_normal_ops.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import random
import itertools
from collections import deque, Counter
from typing import List, Tuple


# ----------------------
# Minimal 2D vector (no external deps)
# ----------------------
class V2:
    __slots__ = ("x", "y")
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = float(x)
        self.y = float(y)
    def __add__(self, other: "V2") -> "V2": return V2(self.x + other.x, self.y + other.y)
    def __sub__(self, other: "V2") -> "V2": return V2(self.x - other.x, self.y - other.y)
    def __mul__(self, k: float) -> "V2": return V2(self.x * k, self.y * k)
    __rmul__ = __mul__
    def length(self) -> float: return math.hypot(self.x, self.y)
    def distance_to(self, other: "V2") -> float: return math.hypot(self.x - other.x, self.y - other.y)
    def normalize(self) -> "V2":
        L = self.length()
        if L == 0.0: return V2(0.0, 0.0)
        return V2(self.x / L, self.y / L)
    def angle_to(self, other: "V2") -> float:
        # Signed angle from self to other (degrees)
        dot = self.x * other.x + self.y * other.y
        cross = self.x * other.y - self.y * other.x
        return math.degrees(math.atan2(cross, dot))
    def scale_to_length(self, L: float) -> None:
        n = self.normalize()
        self.x = n.x * L
        self.y = n.y * L


# ----------------------
# Defaults (tuneable via CLI)
# ----------------------
DEF_WIDTH, DEF_HEIGHT = 1200, 800
DEF_NUM_WHITE = 25
DEF_NUM_RED   = 5
DEF_MAX_SPEED = 4.0
DEF_NEIGHBOR_RADIUS = 90.0
DEF_AVOID_RADIUS    = 60.0
DEF_CONFLICT_DIST   = 5.0


def generate_polygon_points(n_sides: int, radius: float, center: Tuple[float, float]) -> List[V2]:
    angle_step = 2 * math.pi / n_sides
    cx, cy = center
    return [V2(cx + radius * math.cos(i * angle_step), cy + radius * math.sin(i * angle_step))
            for i in range(n_sides)]


# ----------------------
# UAV agent
# ----------------------
class UAV:
    _id_iter = itertools.count(1)

    def __init__(self, is_priority: bool, width: int, height: int, max_speed: float):
        self.is_priority = is_priority
        self.type = "priority" if is_priority else "normal"
        self.position = V2(
            random.uniform(width * 0.3, width * 0.7),
            random.uniform(height * 0.3, height * 0.7)
        )
        ang = random.uniform(0, 2 * math.pi)
        self.velocity = V2(math.cos(ang) * max_speed, math.sin(ang) * max_speed)
        self.task_count = 0
        self.unique_id = next(UAV._id_iter)
        self.hover = False
        self.max_speed = max_speed
        self.width = width
        self.height = height
        self.patrol_points: List[V2] = []
        self.patrol_index = 0
        self.target: V2 | None = None

    def set_patrol(self):
        self.patrol_points = [
            V2(self.width * 0.3, self.height * 0.3),
            V2(self.width * 0.7, self.height * 0.3),
            V2(self.width * 0.7, self.height * 0.7),
            V2(self.width * 0.3, self.height * 0.7),
        ]
        self.patrol_index = 0
        self.target = self.patrol_points[self.patrol_index]

    def set_target(self, ports: List[V2]):
        self.target = random.choice(ports)

    def update(self, uavs: List["UAV"], ports: List[V2], neighbor_radius: float, avoid_radius: float):
        if self.hover:
            return
        assert self.target is not None, "Target must be set before update."
        to_target = self.target - self.position

        if self.is_priority:
            # Patrol square
            if to_target.length() < 10:
                self.patrol_index = (self.patrol_index + 1) % len(self.patrol_points)
                self.target = self.patrol_points[self.patrol_index]
                to_target = self.target - self.position

            # Mild repulsion among priority UAVs
            repulsion = V2(0, 0)
            for other in uavs:
                if other is self or not other.is_priority:
                    continue
                dist = self.position.distance_to(other.position)
                if dist < 40:
                    repulsion = repulsion + (self.position - other.position)

            if repulsion.length() > 0:
                repulsion = repulsion.normalize() * 0.5
                self.velocity = to_target.normalize() * self.max_speed + repulsion
            else:
                self.velocity = to_target.normalize() * self.max_speed

        else:
            # Delivery boid logic
            if to_target.length() < 10:
                self.task_count += 1
                self.set_target(ports)
                to_target = self.target - self.position

            alignment = V2(0, 0)
            cohesion  = V2(0, 0)
            separation= V2(0, 0)
            total = 0.0

            for other in uavs:
                if other is self:
                    continue
                dist = self.position.distance_to(other.position)
                if dist < neighbor_radius:
                    weight = 3.0 if other.type == "priority" else 1.0
                    alignment = alignment + (other.velocity * weight)
                    cohesion  = cohesion  + (other.position * weight)
                    if dist < avoid_radius:
                        separation = separation + ((self.position - other.position) * weight)
                    total += weight

            if total > 0:
                alignment = V2(alignment.x / total, alignment.y / total)
                if alignment.length() > 0:
                    alignment = alignment.normalize() * self.max_speed
                    alignment = alignment - self.velocity

                cohesion = V2(cohesion.x / total, cohesion.y / total)
                cohesion = cohesion - self.position
                if cohesion.length() > 0:
                    cohesion = cohesion.normalize() * self.max_speed
                    cohesion = cohesion - self.velocity

                separation = V2(separation.x / total, separation.y / total)
                if separation.length() > 0:
                    separation = separation.normalize() * self.max_speed
                    separation = separation - self.velocity

            goal_vec = to_target.normalize() * self.max_speed
            steer = goal_vec - self.velocity
            # Gains aligned with your original sim (alignment, cohesion not used in sum there)
            self.velocity = self.velocity + (steer * 0.04) + (separation * 0.4)

            if self.velocity.length() > self.max_speed:
                self.velocity.scale_to_length(self.max_speed)

        self.position = self.position + self.velocity
        self.edges()

    def edges(self):
        if self.position.x > self.width:  self.position.x = 0
        elif self.position.x < 0:         self.position.x = self.width
        if self.position.y > self.height: self.position.y = 0
        elif self.position.y < 0:         self.position.y = self.height


# ----------------------
# Helpers
# ----------------------
def nearest_port_index(pt: V2, ports: List[V2]) -> int:
    dists = [pt.distance_to(p) for p in ports]
    return min(range(len(ports)), key=lambda i: dists[i])


def summarize_state(step: int, uavs: List[UAV], whites: List[UAV], reds: List[UAV],
                    recent_conflicts: list[tuple[int,int]], ports: List[V2]) -> dict:
    # Port queues: white UAVs that still haven't reached target (>=10 px away)
    queues = [0] * len(ports)
    for w in whites:
        idx = ports.index(w.target)  # target is one of ports objects
        if w.position.distance_to(w.target) >= 10:
            queues[idx] += 1

    # Map conflict midpoints to nearest port
    port_conf = Counter()
    for (i, j) in recent_conflicts:
        mid = V2((uavs[i].position.x + uavs[j].position.x) / 2.0,
                 (uavs[i].position.y + uavs[j].position.y) / 2.0)
        port_conf[nearest_port_index(mid, ports)] += 1
    hotspots = [{"port_id": int(k), "recent_conflicts": int(v)} for k, v in port_conf.most_common(3)]

    # Min pairwise separation
    mind = float("inf")
    for i in range(len(uavs)):
        for j in range(i + 1, len(uavs)):
            d = uavs[i].position.distance_to(uavs[j].position)
            if d < mind:
                mind = d

    # Sample up to 12 UAVs for compact logging
    sample = random.sample(uavs, min(12, len(uavs)))
    uavs_sample = [{"id": u.unique_id, "type": u.type, "x": round(u.position.x, 2), "y": round(u.position.y, 2)}
                   for u in sample]

    state = {
        "t": step,
        "ports": [{"id": i, "x": round(p.x, 2), "y": round(p.y, 2)} for i, p in enumerate(ports)],
        "queues": queues,
        "conflicts": {"recent_window": len(recent_conflicts)},
        "hotspots": hotspots,
        "uavs_sample": uavs_sample,
        "min_pairwise_sep": round(mind, 2),
        "weather": "nominal"
    }
    return state


def normal_policy_patrol(state: dict) -> dict:
    actions = []
    queues = state["queues"]
    if queues:
        max_q = max(queues); min_q = min(queues)
        max_i = queues.index(max_q); min_i = queues.index(min_q)
        if max_q >= 6 and max_i != min_i:
            actions.append({
                "action": "grant_corridor",
                "lane_id": f"port-{min_i}->port-{max_i}",
                "ttl_s": 90,
                "conditions": ["speed<=vmax"]
            })
    if state["conflicts"]["recent_window"] >= 3 and state["hotspots"]:
        h = state["hotspots"][0]["port_id"]
        actions.append({
            "action": "declare_no_go_volume",
            "volume_id": f"bubble-port-{h}",
            "ttl_s": 60
        })
    return {"actions": actions, "rationale": "Maintain coverage and ease congestion under nominal ops."}


def normal_policy_delivery(state: dict) -> dict:
    actions = []
    queues = state["queues"]
    if queues:
        max_q = max(queues); min_q = min(queues)
        max_i = queues.index(max_q); min_i = queues.index(min_q)
        if max_q - min_q >= 4 and max_i != min_i:
            actions.append({
                "action": "reroute_flows",
                "from_port": f"port-{max_i}",
                "to_port": f"port-{min_i}",
                "fraction": 0.2   # move 20% of new assignments
            })
    actions.append({
        "action": "set_param",
        "target": "Delivery",
        "name": "sep_margin",
        "value": 5.0,
        "ttl_s": 120
    })
    return {"actions": actions, "rationale": "Balance queues and preserve nominal separation."}


def run_and_log(steps: int, window: int, out_path: str, width: int, height: int, n_white: int, n_red: int,
                max_speed: float, neighbor_radius: float, avoid_radius: float, conflict_distance: float,
                seed: int | None = None) -> str:
    if seed is not None:
        random.seed(seed)

    ports = generate_polygon_points(n_sides=6, radius=min(width, height) * 0.35, center=(width // 2, height // 2))

    whites = [UAV(is_priority=False, width=width, height=height, max_speed=max_speed) for _ in range(n_white)]
    reds   = [UAV(is_priority=True,  width=width, height=height, max_speed=max_speed) for _ in range(n_red)]
    for r in reds:
        r.set_patrol()
    for w in whites:
        w.set_target(ports)

    uavs = reds + whites
    recent_conflicts = deque(maxlen=window)

    with open(out_path, "w") as f:
        for step in range(1, steps + 1):
            # Reset hover flags
            for u in uavs:
                u.hover = False

            # Conflict detection & priority/yield rules
            seen_pairs = set()
            for i in range(len(uavs)):
                for j in range(i + 1, len(uavs)):
                    d = uavs[i].position.distance_to(uavs[j].position)
                    if d < conflict_distance and (i, j) not in seen_pairs:
                        u1, u2 = uavs[i], uavs[j]
                        if (not u1.is_priority) and (not u2.is_priority):
                            # White-White: right-hand yield rule
                            rel = V2(u2.position.x - u1.position.x, u2.position.y - u1.position.y)
                            angle = rel.angle_to(u1.velocity)
                            if angle < 90:
                                u1.hover = True
                            else:
                                u2.hover = True
                        else:
                            # Red-Red or Red-White => white yields
                            if u1.is_priority and (not u2.is_priority):
                                u2.hover = True
                            elif u2.is_priority and (not u1.is_priority):
                                u1.hover = True
                            # Red-Red: no hover here; repulsion handled in update
                        seen_pairs.add((i, j))
                        recent_conflicts.append((i, j))

            # Dynamics update
            for u in uavs:
                u.update(uavs, ports, neighbor_radius, avoid_radius)

            # Build state summary
            state = summarize_state(step, uavs, whites, reds, list(recent_conflicts), ports)

            # Normal-ops "teacher" decisions
            patrol_dec   = normal_policy_patrol(state)
            delivery_dec = normal_policy_delivery(state)

            # Log (Patrol + Delivery)
            f.write(json.dumps({"role": "Patrol",   "state": state, "decision": patrol_dec}) + "\n")
            f.write(json.dumps({"role": "Delivery", "state": state, "decision": delivery_dec}) + "\n")

    return out_path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate UTM normal-operations (state->decision) JSONL dataset.")
    p.add_argument("--steps", type=int, default=5000, help="Simulation steps (each step logs 2 records).")
    p.add_argument("--window", type=int, default=60, help="Recent conflict window size.")
    p.add_argument("--out", type=str, default="utm_normal_ops.jsonl", help="Output JSONL path.")
    p.add_argument("--width", type=int, default=DEF_WIDTH)
    p.add_argument("--height", type=int, default=DEF_HEIGHT)
    p.add_argument("--n_white", type=int, default=DEF_NUM_WHITE)
    p.add_argument("--n_red", type=int, default=DEF_NUM_RED)
    p.add_argument("--max_speed", type=float, default=DEF_MAX_SPEED)
    p.add_argument("--neighbor_radius", type=float, default=DEF_NEIGHBOR_RADIUS)
    p.add_argument("--avoid_radius", type=float, default=DEF_AVOID_RADIUS)
    p.add_argument("--conflict_distance", type=float, default=DEF_CONFLICT_DIST)
    p.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility.")
    return p.parse_args()


def main():
    args = parse_args()
    path = run_and_log(
        steps=args.steps,
        window=args.window,
        out_path=args.out,
        width=args.width,
        height=args.height,
        n_white=args.n_white,
        n_red=args.n_red,
        max_speed=args.max_speed,
        neighbor_radius=args.neighbor_radius,
        avoid_radius=args.avoid_radius,
        conflict_distance=args.conflict_distance,
        seed=args.seed,
    )
    print(f"Wrote dataset to: {path}")


if __name__ == "__main__":
    main()
