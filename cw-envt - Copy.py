import math
import random
import time

import pybullet as p
import pybullet_data

import creature  # your creature module


# =========================
# ENVIRONMENT BUILDERS
# =========================

def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    """
    Procedurally generate a 'mountain' made of small boxes using a Gaussian height profile.
    This is *optional* if you're loading a URDF mountain instead, but it's here if you want it.
    """
    def gaussian(x, y, sigma=arena_size / 4):
        # Height of the mountain at (x, y) using a 2D Gaussian
        return mountain_height * math.exp(-((x ** 2 + y ** 2) / (2 * sigma ** 2)))

    for _ in range(num_rocks):
        x = random.uniform(-arena_size / 2, arena_size / 2)
        y = random.uniform(-arena_size / 2, arena_size / 2)
        z = gaussian(x, y)  # height from Gaussian

        # Smaller rocks near the peak
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([
            random.uniform(0, math.pi),
            random.uniform(0, math.pi),
            random.uniform(0, math.pi),
        ])

        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size],
            rgbaColor=[0.5, 0.5, 0.5, 1],
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=rock_shape,
            baseVisualShapeIndex=rock_visual,
            basePosition=[x, y, z],
            baseOrientation=orientation,
        )


def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    """
    Simple scattered rocks on the floor, if you want extra obstacles.
    """
    for _ in range(num_rocks):
        x = random.uniform(-arena_size / 2, arena_size / 2)
        y = random.uniform(-arena_size / 2, arena_size / 2)
        z = 0.5
        size = random.uniform(0.1, max_size)

        orientation = p.getQuaternionFromEuler([
            random.uniform(0, math.pi),
            random.uniform(0, math.pi),
            random.uniform(0, math.pi),
        ])

        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size],
            rgbaColor=[0.5, 0.5, 0.5, 1],
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=rock_shape,
            baseVisualShapeIndex=rock_visual,
            basePosition=[x, y, z],
            baseOrientation=orientation,
        )


def make_arena(arena_size=10, wall_height=1):
    """
    Build a simple square arena with a yellow floor and grey walls.
    """
    wall_thickness = 0.5

    # Floor
    floor_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, arena_size / 2, wall_thickness],
    )
    floor_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, arena_size / 2, wall_thickness],
        rgbaColor=[1, 1, 0, 1],  # yellow floor
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=floor_collision_shape,
        baseVisualShapeIndex=floor_visual_shape,
        basePosition=[0, 0, -wall_thickness],
    )

    # Two opposite walls (front/back)
    wall_collision_shape_fb = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2],
    )
    wall_visual_shape_fb = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2],
        rgbaColor=[0.7, 0.7, 0.7, 1],  # grey walls
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape_fb,
        baseVisualShapeIndex=wall_visual_shape_fb,
        basePosition=[0, arena_size / 2, wall_height / 2],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape_fb,
        baseVisualShapeIndex=wall_visual_shape_fb,
        basePosition=[0, -arena_size / 2, wall_height / 2],
    )

    # Two opposite walls (left/right)
    wall_collision_shape_lr = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2],
    )
    wall_visual_shape_lr = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2],
        rgbaColor=[0.7, 0.7, 0.7, 1],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape_lr,
        baseVisualShapeIndex=wall_visual_shape_lr,
        basePosition=[arena_size / 2, 0, wall_height / 2],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape_lr,
        baseVisualShapeIndex=wall_visual_shape_lr,
        basePosition=[-arena_size / 2, 0, wall_height / 2],
    )


def load_mountain():
    """
    Load the mountain URDF generated by prepare_shapes.py.
    Assumes there is a 'shapes/' folder with a 'gaussian_pyramid.urdf' inside.
    """
    mountain_position = (0, 0, -1)  # tweak if needed
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))

    # Add local 'shapes' folder to search path so loadURDF can find the mountain
    p.setAdditionalSearchPath("shapes/")

    # You can swap this to other URDFs you generate
    mountain_id = p.loadURDF(
        "gaussian_pyramid.urdf",
        mountain_position,
        mountain_orientation,
        useFixedBase=1,
    )

    return mountain_id


def spawn_random_creature():
    """
    Generate a random creature using your Creature class and drop it into the world.
    Returns the PyBullet body id of the robot.
    """
    # gene_count can be adjusted depending on how complex you want it
    cr = creature.Creature(gene_count=3)

    # Save the creature as a temporary URDF
    urdf_path = "test.urdf"
    with open(urdf_path, "w") as f:
        f.write(cr.to_xml())

    # Spawn it above the mountain/arena so it can fall down
    start_pos = (0, 0, 10)
    start_orn = p.getQuaternionFromEuler((0, 0, 0))
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn)

    return robot_id


# =========================
# MAIN DEMO / ENTRY POINT
# =========================

def main():
    # Connect in GUI mode so you can actually see the environment.
    p.connect(p.GUI)

    # Allow PyBullet to find its default URDFs (plane, etc.)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Gravity
    p.setGravity(0, 0, -10)

    # Build arena and mountain
    arena_size = 20
    make_arena(arena_size=arena_size)
    # make_rocks(arena_size=arena_size)  # optional, if you want extra obstacles
    load_mountain()

    # Create one random creature
    robot_id = spawn_random_creature()

    # Use real-time simulation so PyBullet updates automatically with wall-clock time
    p.setRealTimeSimulation(1)

    print("Simulation running. Close the window or press Ctrl+C in the terminal to exit.")

    try:
        # Keep the script alive so the GUI window doesn't close immediately
        while True:
            # If you want to read state or apply control, you can do it here.
            # For now, we just sleep a bit to avoid busy-waiting.
            time.sleep(1.0 / 120.0)
    except KeyboardInterrupt:
        print("Exiting simulation...")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
