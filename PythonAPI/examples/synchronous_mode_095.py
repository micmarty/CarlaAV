#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
from pathlib import PosixPath
from collections import namedtuple
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
import logging
import random
import argparse
import math
try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))

    return (norm_target, d_angle)

def draw_image(surface, image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

def can_stop_gathering(counter, limit) -> bool:
    if counter['green'] >= limit and counter['red'] >= limit and counter['none'] >= limit:
        return True
    return False
    #return all([val >= limit for val in counter.values()])


def get_gt_class(vehicle, traffic_lights, debug) -> str:
    min_angle = 180.0
    best_magnitude = 0.0
    best_tl = None

    veh_transform = vehicle.get_transform()
    for traffic_light in traffic_lights:
        tl_location = traffic_light.get_location()
        magnitude, angle = compute_magnitude_angle(tl_location, veh_transform.location, veh_transform.rotation.yaw)
        is_above_veh = True # tl_location.z >= veh_transform.location.z
        if is_above_veh and magnitude < 40.0 and angle < min(25.0, min_angle):
            best_magnitude = magnitude
            best_tl = traffic_light
            min_angle = angle
    
    if best_tl:
        print(best_magnitude, min_angle)
        debug.draw_point(best_tl.get_location() + carla.Location(z=2.0), 0.5, carla.Color(255, 0, 0), 1, False)


        if best_tl.state == carla.libcarla.TrafficLightState.Red:
            return 'red'
        elif best_tl.state == carla.libcarla.TrafficLightState.Green:
            return 'green'
        elif best_tl.state == carla.libcarla.TrafficLightState.Yellow:
            return 'yellow'
    else:
        return 'none'

# TODO Camera pos
# TODO Lane explorer
# TODO Custom path
def main():
    argparser = argparse.ArgumentParser(description='CARLA Traffic Lights dataset collector')
    argparser.add_argument('-r', '--record', action='store_true', help='Save images to disk')
    argparser.add_argument('-x', '--explore-lanes', action='store_true', help='Explore lanes')
    argparser.add_argument('-o', '--output-dir', required=True, type=PosixPath, metavar='DIR', help='Directory to put dataset into')
    argparser.add_argument('-c', '--images-per-class', required=True, metavar='N', type=int, help='Stop recording after collecting N images per class')
    argparser.add_argument('-d', '--teleport-distance', required=True, metavar='N', type=float, help='Distance in meters between waypoints to teleport vehicle')

    args = argparser.parse_args()
    

    # Prepare folder structure
    basepath = args.output_dir.expanduser()
    basepath.mkdir(parents=True, exist_ok=True)
    classes = ['green', 'red', 'yellow', 'none']
    counter = {
        'green': 0,
        'red': 0,
        'yellow': 0,
        'none': 0
    }
    for classname in classes:
        (basepath / classname).mkdir(parents=True, exist_ok=True)
    # ImgCounter = namedtuple('ImgCounter', field_names=classes)
    # counter = ImgCounter(green=0, red=0, yellow=0, none=0)

    can_stop_gathering(counter, args.images_per_class)


    actor_list = []
    pygame.init()

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    world = client.get_world()

    print('enabling synchronous mode.')
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        current_w = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.lincoln*')),
            start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)

        # Camera
        third_person_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        dashboard_transform = carla.Transform(carla.Location(x=1.4, z=1.6))

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera = world.spawn_actor(camera_bp, dashboard_transform, attach_to=vehicle)
        actor_list.append(camera)

        # Actors
        actors = world.get_actors()
        traffic_lights = actors.filter("*traffic_light*")

        # Make sync queue for sensor data.
        image_queue = queue.Queue()
        camera.listen(image_queue.put)

        frame = None

        display = pygame.display.set_mode(
            (800, 600),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        font = get_font()

        clock = pygame.time.Clock()

        while True:
            if should_quit():
                return

            clock.tick()
            world.tick()
            ts = world.wait_for_tick()

            if frame is not None:
                if ts.frame_count != frame + 1:
                    logging.warning('frame skip!')

            frame = ts.frame_count

            while True:
                image = image_queue.get()
                if image.frame_number == ts.frame_count:
                    break
                logging.warning(
                    'wrong image time-stampstamp: frame=%d, image.frame=%d',
                    ts.frame_count,
                    image.frame_number)

            potential_w = list(current_w.next(args.teleport_distance))

            if args.explore_lanes:
                # check for available right driving lanes
                if current_w.lane_change & carla.LaneChange.Right:
                    right_w = current_w.get_right_lane()
                    if right_w and right_w.lane_type == carla.libcarla.LaneType.Driving:
                        potential_w += list(right_w.next(args.teleport_distance))

                # check for available left driving lanes
                if current_w.lane_change & carla.LaneChange.Left:
                    left_w = current_w.get_left_lane()
                    if left_w and left_w.lane_type == carla.libcarla.LaneType.Driving:
                        potential_w += list(left_w.next(args.teleport_distance))

            # choose a random waypoint to be the next
            next_w = random.choice(potential_w)
            potential_w.remove(next_w)
            vehicle.set_transform(next_w.transform)
            current_w = next_w

            draw_image(display, image)

            # Save image
            grount_truth_class = get_gt_class#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
from pathlib import PosixPath
from collections import namedtuple
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
import logging
import random
import argparse
import math
try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))

    return (norm_target, d_angle)

def draw_image(surface, image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

def can_stop_gathering(counter, limit) -> bool:
    if counter['green'] >= limit and counter['red'] >= limit and counter['none'] >= limit:
        return True
    return False
    #return all([val >= limit for val in counter.values()])


def get_gt_class(vehicle, traffic_lights, debug) -> str:
    min_angle = 180.0
    best_magnitude = 0.0
    best_tl = None

    veh_transform = vehicle.get_transform()
    for traffic_light in traffic_lights:
        tl_location = traffic_light.get_location()
        magnitude, angle = compute_magnitude_angle(tl_location, veh_transform.location, veh_transform.rotation.yaw)
        is_above_veh = True # tl_location.z >= veh_transform.location.z
        if is_above_veh and magnitude < 40.0 and angle < min(25.0, min_angle):
            best_magnitude = magnitude
            best_tl = traffic_light
            min_angle = angle
    
    if best_tl:
        print(best_magnitude, min_angle)
        debug.draw_point(best_tl.get_location() + carla.Location(z=2.0), 0.5, carla.Color(255, 0, 0), 1, False)


        if best_tl.state == carla.libcarla.TrafficLightState.Red:
            return 'red'
        elif best_tl.state == carla.libcarla.TrafficLightState.Green:
            return 'green'
        elif best_tl.state == carla.libcarla.TrafficLightState.Yellow:
            return 'yellow'
    else:
        return 'none'

# TODO Camera pos
# TODO Lane explorer
# TODO Custom path
def main():
    argparser = argparse.ArgumentParser(description='CARLA Traffic Lights dataset collector')
    argparser.add_argument('-r', '--record', action='store_true', help='Save images to disk')
    argparser.add_argument('-x', '--explore-lanes', action='store_true', help='Explore lanes')
    argparser.add_argument('-o', '--output-dir', required=True, type=PosixPath, metavar='DIR', help='Directory to put dataset into')
    argparser.add_argument('-c', '--images-per-class', required=True, metavar='N', type=int, help='Stop recording after collecting N images per class')
    argparser.add_argument('-d', '--teleport-distance', required=True, metavar='N', type=float, help='Distance in meters between waypoints to teleport vehicle')

    args = argparser.parse_args()
    

    # Prepare folder structure
    basepath = args.output_dir.expanduser()
    basepath.mkdir(parents=True, exist_ok=True)
    classes = ['green', 'red', 'yellow', 'none']
    counter = {
        'green': 0,
        'red': 0,
        'yellow': 0,
        'none': 0
    }
    for classname in classes:
        (basepath / classname).mkdir(parents=True, exist_ok=True)
    # ImgCounter = namedtuple('ImgCounter', field_names=classes)
    # counter = ImgCounter(green=0, red=0, yellow=0, none=0)

    can_stop_gathering(counter, args.images_per_class)


    actor_list = []
    pygame.init()

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    world = client.get_world()

    print('enabling synchronous mode.')
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        current_w = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.lincoln*')),
            start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)

        # Camera
        third_person_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        dashboard_transform = carla.Transform(carla.Location(x=1.4, z=1.6))

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera = world.spawn_actor(camera_bp, dashboard_transform, attach_to=vehicle)
        actor_list.append(camera)

        # Actors
        actors = world.get_actors()
        traffic_lights = actors.filter("*traffic_light*")

        # Make sync queue for sensor data.
        image_queue = queue.Queue()
        camera.listen(image_queue.put)

        frame = None

        display = pygame.display.set_mode(
            (800, 600),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        font = get_font()

        clock = pygame.time.Clock()

        while True:
            if should_quit():
                return

            clock.tick()
            world.tick()
            ts = world.wait_for_tick()

            if frame is not None:
                if ts.frame_count != frame + 1:
                    logging.warning('frame skip!')

            frame = ts.frame_count

            while True:
                image = image_queue.get()
                if image.frame_number == ts.frame_count:
                    break
                logging.warning(
                    'wrong image time-stampstamp: frame=%d, image.frame=%d',
                    ts.frame_count,
                    image.frame_number)

            potential_w = list(current_w.next(args.teleport_distance))

            if args.explore_lanes:
                # check for available right driving lanes
                if current_w.lane_change & carla.LaneChange.Right:
                    right_w = current_w.get_right_lane()
                    if right_w and right_w.lane_type == carla.libcarla.LaneType.Driving:
                        potential_w += list(right_w.next(args.teleport_distance))

                # check for available left driving lanes
                if current_w.lane_change & carla.LaneChange.Left:
                    left_w = current_w.get_left_lane()
                    if left_w and left_w.lane_type == carla.libcarla.LaneType.Driving:
                        potential_w += list(left_w.next(args.teleport_distance))

            # choose a random waypoint to be the next
            next_w = random.choice(potential_w)
            potential_w.remove(next_w)
            vehicle.set_transform(next_w.transform)
            current_w = next_w

            draw_image(display, image)

            # Save image
            grount_truth_class = get_gt_class(vehicle, traffic_lights, world.debug)
            if args.record:
                print('[Recording] Class: {} [{}/{}]'.format(grount_truth_class, counter[grount_truth_class], args.images_per_class))
                counter[grount_truth_class] += 1
                filepath = '{base}/{name}/{id}.png'.format(base=basepath, name=grount_truth_class, id=image.frame_number)
                image.save_to_disk(filepath, color_converter=cc.Raw)

                if can_stop_gathering(counter, args.images_per_class):
                    break

            text_surface = font.render('% 5d FPS' % clock.get_fps(), True, (255, 255, 255))
            display.blit(text_surface, (8, 10))

            pygame.display.flip()

    finally:
        print('\ndisabling synchronous mode.')
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    main()
(vehicle, traffic_lights, world.debug)
            if args.record:
                print('[Recording] Class: {}'.format(grount_truth_class))
                counter[grount_truth_class] += 1
                filepath = '{base}/{name}/{id}.png'.format(base=basepath, name=grount_truth_class, id=image.frame_number)
                image.save_to_disk(filepath, color_converter=cc.Raw)

                if can_stop_gathering(counter, args.images_per_class):
                    break

            text_surface = font.render('% 5d FPS' % clock.get_fps(), True, (255, 255, 255))
            display.blit(text_surface, (8, 10))

            pygame.display.flip()

    finally:
        print('\ndisabling synchronous mode.')
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    main()
