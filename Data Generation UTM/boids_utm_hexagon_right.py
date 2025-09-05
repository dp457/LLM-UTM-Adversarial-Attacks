# -*- coding: utf-8 -*-
"""
Created on Fri May  2 15:08:23 2025

@author: Deepak.Panda
"""

import pygame
import random
import math

# Constants
WIDTH, HEIGHT = 1000, 700
NUM_WHITE_UAVS = 25
NUM_RED_UAVS = 5
MAX_SPEED = 4
NEIGHBOR_RADIUS = 90
AVOID_RADIUS = 60
CONFLICT_DISTANCE = 5

# Pygame setup
pygame.init()

screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
WIDTH, HEIGHT = screen.get_size()

pygame.display.set_caption("UTM UAV Simulation: Conflicts, Tasks, and Labeled Delivery Ports")
clock = pygame.time.Clock()
font = pygame.font.SysFont('Arial', 14)

def generate_polygon_points(n_sides, radius, center):
    angle_step = 2 * math.pi / n_sides
    return [
        pygame.math.Vector2(
            center[0] + radius * math.cos(i * angle_step),
            center[1] + radius * math.sin(i * angle_step)
        )
        for i in range(n_sides)
    ]

DELIVERY_POINTS = generate_polygon_points(
    n_sides=6,
    radius=min(WIDTH, HEIGHT)*0.35,
    center=(WIDTH // 2, HEIGHT // 2)
)

class UAV:
    def __init__(self, is_priority=False):
        self.is_priority = is_priority
        self.type = "priority" if is_priority else "normal"
        self.position = pygame.math.Vector2(
            random.uniform(WIDTH * 0.3, WIDTH * 0.7),
            random.uniform(HEIGHT * 0.3, HEIGHT * 0.7)
        )
        angle = random.uniform(0, 2 * math.pi)
        self.velocity = pygame.math.Vector2(math.cos(angle), math.sin(angle)) * MAX_SPEED
        self.task_count = 0
        self.unique_id = id(self) % 1000
        self.hover = False

        if self.is_priority:
            self.patrol_points = [
                pygame.math.Vector2(WIDTH * 0.3, HEIGHT * 0.3),
                pygame.math.Vector2(WIDTH * 0.7, HEIGHT * 0.3),
                pygame.math.Vector2(WIDTH * 0.7, HEIGHT * 0.7),
                pygame.math.Vector2(WIDTH * 0.3, HEIGHT * 0.7)
            ]
            self.patrol_index = 0
            self.target = self.patrol_points[self.patrol_index]
        else:
            self.set_target()

    def set_target(self):
        self.target = random.choice(DELIVERY_POINTS)

    def update(self, uavs):
        if self.hover:
            return

        to_target = self.target - self.position

        if self.is_priority:
            if to_target.length() < 10:
                self.patrol_index = (self.patrol_index + 1) % len(self.patrol_points)
                self.target = self.patrol_points[self.patrol_index]
                to_target = self.target - self.position

            repulsion = pygame.math.Vector2(0, 0)
            for other in uavs:
                if other != self and other.is_priority:
                    distance = self.position.distance_to(other.position)
                    if distance < 40:
                        repulsion += (self.position - other.position)

            if repulsion.length() > 0:
                repulsion = repulsion.normalize() * 0.5
                self.velocity = to_target.normalize() * MAX_SPEED + repulsion
            else:
                self.velocity = to_target.normalize() * MAX_SPEED

        else:
            if to_target.length() < 10:
                self.task_count += 1
                self.set_target()
                to_target = self.target - self.position

            alignment = pygame.math.Vector2(0, 0)
            cohesion = pygame.math.Vector2(0, 0)
            separation = pygame.math.Vector2(0, 0)
            total = 0

            for other in uavs:
                if other == self:
                    continue
                distance = self.position.distance_to(other.position)
                if distance < NEIGHBOR_RADIUS:
                    weight = 3.0 if other.type == "priority" else 1.0
                    alignment += other.velocity * weight
                    cohesion += other.position * weight
                    if distance < AVOID_RADIUS:
                        separation += (self.position - other.position) * weight
                    total += weight

            if total > 0:
                alignment /= total
                if alignment.length() > 0:
                    alignment = alignment.normalize() * MAX_SPEED
                    alignment -= self.velocity

                cohesion /= total
                cohesion = cohesion - self.position
                if cohesion.length() > 0:
                    cohesion = cohesion.normalize() * MAX_SPEED
                    cohesion -= self.velocity

                separation /= total
                if separation.length() > 0:
                    separation = separation.normalize() * MAX_SPEED
                    separation -= self.velocity

            goal_vector = to_target.normalize() * MAX_SPEED
            steer = goal_vector - self.velocity
            self.velocity += steer * 0.04 + alignment * 0.0 + cohesion * 0.0 + separation * 0.4

            if self.velocity.length() > MAX_SPEED:
                self.velocity.scale_to_length(MAX_SPEED)

        self.position += self.velocity
        self.edges()

    def edges(self):
        if self.position.x > WIDTH: self.position.x = 0
        elif self.position.x < 0: self.position.x = WIDTH
        if self.position.y > HEIGHT: self.position.y = 0
        elif self.position.y < 0: self.position.y = HEIGHT

    def draw(self, screen):
        color = (255, 0, 0) if self.is_priority else (255, 255, 255)
        pygame.draw.circle(screen, color, self.position, 5)
        pygame.draw.circle(screen, (0, 255, 0), self.target, 4, 1)

        if self.is_priority:
            label = f"P{self.patrol_index + 1}"
            text_surface = font.render(label, True, (0, 255, 0))
            screen.blit(text_surface, (self.target.x + 5, self.target.y + 5))

# Initialize UAVs
white_uavs = [UAV(is_priority=False) for _ in range(NUM_WHITE_UAVS)]
red_uavs = [UAV(is_priority=True) for _ in range(NUM_RED_UAVS)]
uavs = red_uavs + white_uavs

# Main loop
running = True
total_steps = 0
conflict_rr = 0
conflict_ww = 0
conflict_rw = 0

while running:
    screen.fill((0, 0, 0))
    total_steps += 1

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    for uav in uavs:
        uav.hover = False

    all_conflicts = set()
    for i in range(len(uavs)):
        for j in range(i + 1, len(uavs)):
            d = uavs[i].position.distance_to(uavs[j].position)
            if d < CONFLICT_DISTANCE:
                pair = tuple(sorted([uavs[i].unique_id, uavs[j].unique_id]))
                if pair not in all_conflicts:
                    uav1, uav2 = uavs[i], uavs[j]
                    if uav1.is_priority and uav2.is_priority:
                        conflict_rr += 1
                    elif not uav1.is_priority and not uav2.is_priority:
                        conflict_ww += 1
                        angle = (uav2.position - uav1.position).angle_to(uav1.velocity)
                        if angle < 90:
                            uav1.hover = True  # uav2 is to the right, so uav1 yields
                        else:
                            uav2.hover = True
                    else:
                        conflict_rw += 1
                        if uav1.is_priority:
                            uav2.hover = True
                        else:
                            uav1.hover = True
                    all_conflicts.add(pair)

    for uav in uavs:
        uav.update(uavs)
        uav.draw(screen)

    for idx, pt in enumerate(DELIVERY_POINTS):
        pygame.draw.circle(screen, (0, 128, 0), pt, 6)
        label_offset_x = 15 * math.cos((2 * math.pi / 6) * idx)
        label_offset_y = 15 * math.sin((2 * math.pi / 6) * idx)
        label = f"Port-{idx+1}"
        label_surface = font.render(label, True, (0, 255, 0))
        screen.blit(label_surface, (pt.x + label_offset_x, pt.y + label_offset_y))

    pygame.draw.rect(screen, (50, 50, 50), pygame.Rect(WIDTH * 0.3, HEIGHT * 0.3, WIDTH * 0.4, HEIGHT * 0.4), width=1)

    total_tasks = sum(uav.task_count for uav in white_uavs)
    avg_tasks = total_tasks / NUM_WHITE_UAVS

    info_text = [
        f"Time Steps: {total_steps}",
        f"Completed Tasks: {total_tasks}",
        f"Avg Tasks/UAV: {avg_tasks:.2f}",
        f"Conflicts (Red-Red): {conflict_rr}",
        f"Conflicts (White-White): {conflict_ww}",
        f"Conflicts (Red-White): {conflict_rw}",
        f"Total Conflicts: {conflict_rr + conflict_ww + conflict_rw}"
    ]

    for i, line in enumerate(info_text):
        text_surface = font.render(line, True, (255, 255, 0))
        screen.blit(text_surface, (10, 10 + i * 20))

    pygame.display.flip()
    clock.tick(60)

    if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
        running = False

pygame.quit()