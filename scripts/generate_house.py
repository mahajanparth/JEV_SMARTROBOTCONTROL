#!/usr/bin/env python3
"""Generate matching Gazebo collision geometry and an occupancy map. No SLAM needed."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'src' / 'jev_localization_recovery'
RESOLUTION = 0.05
WIDTH, HEIGHT = 12.0, 10.0
# name, x_min, y_min, x_max, y_max, height, RGB
WALL = '0.82 0.84 0.88'
WOOD = '0.55 0.31 0.15'
BOXES = [
    ('south', 0, 0, 12, .15, 1.2, WALL), ('north', 0, 9.85, 12, 10, 1.2, WALL),
    ('west', 0, 0, .15, 10, 1.2, WALL), ('east', 11.85, 0, 12, 10, 1.2, WALL),
    ('living_hall_left', 0, 3.9, 1.8, 4.05, 1.2, WALL),
    ('living_hall_right', 3.0, 3.9, 8.4, 4.05, 1.2, WALL),
    ('kitchen_hall_right', 9.6, 3.9, 12, 4.05, 1.2, WALL),
    ('kitchen_divider', 5.9, 0, 6.05, 3.9, 1.2, WALL),
    ('bedroom_hall_left', 0, 5.95, 2.4, 6.1, 1.2, WALL),
    ('bedroom_hall_right', 3.6, 5.95, 8.4, 6.1, 1.2, WALL),
    ('office_hall_right', 9.6, 5.95, 12, 6.1, 1.2, WALL),
    ('office_divider', 6.9, 6.1, 7.05, 10, 1.2, WALL),
    ('sofa', .55, .8, 1.3, 2.8, .65, '0.22 0.40 0.62'),
    ('coffee_table', 2.2, 1.0, 3.3, 1.6, .4, WOOD),
    ('living_cabinet', 4.8, .4, 5.5, 1.2, .8, WOOD),
    ('kitchen_island', 8, 1, 9, 2.0, .8, '0.7 0.7 0.72'),
    ('kitchen_counter', 11, .4, 11.7, 3.4, .85, '0.8 0.8 0.8'),
    ('bed', .8, 7.6, 2.8, 9.1, .5, '0.28 0.62 0.48'),
    ('wardrobe', 5.5, 7.8, 6.6, 9.5, 1.0, WOOD),
    ('office_desk', 8, 8.7, 10, 9.4, .7, WOOD),
    ('bookcase', 10.9, 6.8, 11.7, 8, 1.0, WOOD),
]


def box(name, x0, y0, x1, y1, height, color):
    size = '{} {} {}'.format(x1 - x0, y1 - y0, height)
    return f'''<model name="{name}"><static>true</static>
      <pose>{(x0+x1)/2} {(y0+y1)/2} {height/2} 0 0 0</pose><link name="body">
      <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{size}</size></box></geometry>
      <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link></model>'''


def main():
    maps, worlds = ROOT / 'maps', ROOT / 'worlds'
    maps.mkdir(exist_ok=True)
    worlds.mkdir(exist_ok=True)
    width, height = round(WIDTH / RESOLUTION), round(HEIGHT / RESOLUTION)
    pixels = bytearray()
    for row in range(height):
        y = (height - row - .5) * RESOLUTION
        for col in range(width):
            x = (col + .5) * RESOLUTION
            occupied = any(x0 <= x <= x1 and y0 <= y <= y1 for _, x0, y0, x1, y1, *_ in BOXES)
            pixels.append(0 if occupied else 254)
    (maps / 'jev_house.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    (maps / 'jev_house.yaml').write_text('image: jev_house.pgm\nmode: trinary\nresolution: 0.05\n'
                                       'origin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    geometry = '\n'.join(box(*item) for item in BOXES)
    world = f'''<?xml version="1.0"?>
<sdf version="1.6"><world name="jev_house">
  <gravity>0 0 -9.81</gravity>
  <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
  <scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>
  <light name="sun" type="directional"><pose>0 0 12 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction><cast_shadows>false</cast_shadows></light>
  <gui fullscreen="0"><camera name="user_camera"><pose>6 -3 14 0 1.08 1.57</pose>
    <view_controller>orbit</view_controller></camera></gui>
  <model name="floor"><static>true</static><pose>6 5 -0.05 0 0 0</pose><link name="floor">
    <collision name="floor"><geometry><box><size>14 12 0.1</size></box></geometry></collision>
    <visual name="floor"><geometry><box><size>14 12 0.1</size></box></geometry>
      <material><ambient>0.75 0.70 0.60 1</ambient><diffuse>0.75 0.70 0.60 1</diffuse></material></visual>
  </link></model>
  {geometry}
</world></sdf>'''
    (worlds / 'jev_house.world').write_text(world)
    print('Generated 12 x 10 m house and matching 0.05 m occupancy map.')


if __name__ == '__main__':
    main()
