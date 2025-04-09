# visuals/sprites.py
import pygame
import math
import numpy as np
from config import CAR_WIDTH, CAR_LENGTH, ROAD_COLOR, CAR_COLOR, LANE_WIDTH, OBSTACLE_COLOR

# Cache surfaces globally (you could optimize further if needed)


def draw_car(screen, car, lane):
    # Calculate direction and position
    angle = math.atan2(lane.unit_direction[1], lane.unit_direction[0])
    pos = lane.start_pos + lane.unit_direction * car.offset

    # Normalize speed to grayscale
    speed = car.driver_model.desired_speed + car.max_speed
    intensity = 255 - int(255 * (speed-(lane.max_speed - 30)) / 60)
    color = (255, intensity, intensity)

    # Draw car
    CAR_SURFACE = pygame.Surface((CAR_LENGTH, CAR_WIDTH), pygame.SRCALPHA)
    CAR_SURFACE.fill(color)
    rotated = pygame.transform.rotate(CAR_SURFACE, -math.degrees(angle))
    rect = rotated.get_rect(center=pos)
    screen.blit(rotated, rect)

def draw_obstacle(screen, obstacle, lane):
    obstacle_surface = pygame.Surface((obstacle.length, CAR_WIDTH), pygame.SRCALPHA)
    obstacle_surface.fill(OBSTACLE_COLOR)
    angle = math.atan2(lane.unit_direction[1], lane.unit_direction[0])
    pos = lane.start_pos + lane.unit_direction * obstacle.offset
    rotated = pygame.transform.rotate(obstacle_surface, -math.degrees(angle))
    rect = rotated.get_rect(center=pos)
    screen.blit(rotated, rect)



def draw_road(screen, road):
    # Compute road surface
    road_length = int(np.linalg.norm(road.end_pos - road.start_pos))
    road_height = int(road.num_lanes * LANE_WIDTH)
    angle = math.atan2(road.unit_direction[1], road.unit_direction[0])
    angle_deg = -math.degrees(angle)

    # Create road surface
    road_surface = pygame.Surface((road_length, road_height), pygame.SRCALPHA)
    road_surface.fill(ROAD_COLOR)

    # Draw dashed white lines between lanes
    for i in range(1, road.num_lanes):
        y = i * LANE_WIDTH
        dash_len = 10
        gap = 10
        for x in range(0, road_length, dash_len + gap):
            pygame.draw.line(road_surface, (255, 255, 255), (x, y), (x + dash_len, y), width=2)

    # Draw outer border (black outline)
    pygame.draw.rect(road_surface, (0, 0, 0), road_surface.get_rect(), width=3)

    # Rotate the whole road surface
    rotated = pygame.transform.rotate(road_surface, angle_deg)
    center_pos = road.start_pos + (road.end_pos - road.start_pos) / 2
    rect = rotated.get_rect(center=center_pos)

    screen.blit(rotated, rect)

