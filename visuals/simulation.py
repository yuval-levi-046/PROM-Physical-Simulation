# visuals/simulation.py
import pygame
from visuals.sprites import draw_road, draw_car
from config import SCREEN_WIDTH, SCREEN_HEIGHT, BACKGROUND_COLOR, FPS

def run_visual_simulation(system, controller=None, logger=None):
    """
    Runs a visual simulation loop using pygame.

    Args:
        system: The traffic simulation System object.
        controller: Optional TrafficController to manage spawning/transfers.
        logger: Optional DataLogger to collect metrics during simulation.
    """
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Traffic Simulation Viewer")
    clock = pygame.time.Clock()

    running = True
    while running and system.time < system.final_time:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Logic update
        if controller:
            controller.control(system.dt)
        system.update(logger=logger)

        # Drawing
        screen.fill(BACKGROUND_COLOR)
        for road in system.roads:
            draw_road(screen, road)
            for lane in road.lanes:
                for car in lane.cars:
                    draw_car(screen, car, lane)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()