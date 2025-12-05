#!/usr/bin/env python3
"""
Simple script to spawn multiple robots in Gazebo
Usage: 
  ./spawn_robots.py                    # Spawns 3 robots with default positions
  ./spawn_robots.py --count 5          # Spawns 5 robots
  ./spawn_robots.py --positions "0,0 2,0 0,2"  # Custom positions
"""

import subprocess
import time
import argparse
import os

# Path to your URDF file
URDF_PATH = os.path.expanduser('~/multi_robot_ws/src/simple_robot_with_control.urdf')

def spawn_robot(robot_name, x, y, z=0.2, namespace=None):
    """Spawn a single robot in Gazebo"""
    if namespace is None:
        namespace = robot_name
    
    cmd = [
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', robot_name,
        '-file', URDF_PATH,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z),
        '-robot_namespace', namespace
    ]
    
    print(f"Spawning {robot_name} at position ({x}, {y}, {z})...")
    subprocess.run(cmd)
    time.sleep(2)  # Wait 2 seconds between spawns

def main():
    parser = argparse.ArgumentParser(description='Spawn multiple robots in Gazebo')
    parser.add_argument('--count', type=int, default=3, help='Number of robots to spawn')
    parser.add_argument('--spacing', type=float, default=2.0, help='Distance between robots (meters)')
    parser.add_argument('--positions', type=str, help='Custom positions as "x1,y1 x2,y2 x3,y3"')
    parser.add_argument('--pattern', choices=['line', 'grid', 'circle'], default='line', 
                       help='Spawn pattern: line, grid, or circle')
    
    args = parser.parse_args()
    
    positions = []
    
    if args.positions:
        # Parse custom positions
        for pos in args.positions.split():
            x, y = map(float, pos.split(','))
            positions.append((x, y))
    elif args.pattern == 'line':
        # Spawn in a line along x-axis
        for i in range(args.count):
            positions.append((i * args.spacing, 0))
    elif args.pattern == 'grid':
        # Spawn in a grid pattern
        grid_size = int(args.count ** 0.5) + 1
        for i in range(args.count):
            x = (i % grid_size) * args.spacing
            y = (i // grid_size) * args.spacing
            positions.append((x, y))
    elif args.pattern == 'circle':
        # Spawn in a circle
        import math
        radius = args.spacing
        for i in range(args.count):
            angle = (2 * math.pi * i) / args.count
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            positions.append((x, y))
    
    print(f"\n{'='*50}")
    print(f"Spawning {len(positions)} robots in '{args.pattern}' pattern")
    print(f"{'='*50}\n")
    
    for i, (x, y) in enumerate(positions, start=1):
        robot_name = f"robot{i}"
        spawn_robot(robot_name, x, y)
    
    print(f"\n{'='*50}")
    print(f"✓ Successfully spawned {len(positions)} robots!")
    print(f"{'='*50}\n")

if __name__ == '__main__':
    main()