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
    Returns BOTH the PyBullet body id of the robot AND the Creature object.
    """
    cr = creature.Creature(gene_count=3)

    # Save the creature as a temporary URDF
    urdf_path = "test.urdf"
    with open(urdf_path, "w") as f:
        f.write(cr.to_xml())

    # Spawn it *on the slope* of the mountain
    start_pos = (0, -4, 3)   # y closer to 0, z a bit higher
    start_orn = p.getQuaternionFromEuler((0, 0, 0))
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn)

    return robot_id, cr



# =========================
# MAIN DEMO / ENTRY POINT
# =========================

def main():
    # Connect in GUI mode so you can actually see the environment.
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Build arena and mountain
    arena_size = 20
    make_arena(arena_size=arena_size)
    load_mountain()

    # Create one random creature – get BOTH id and Creature object
    robot_id, cr = spawn_random_creature()

    # Camera
    p.resetDebugVisualizerCamera(
        cameraDistance=18,
        cameraYaw=45,
        cameraPitch=-35,
        cameraTargetPosition=[0, 0, 2],
    )

    # We will step simulation manually so we can drive motors
    p.setRealTimeSimulation(0)

    print("Simulation running. Close the window or press Ctrl+C in the terminal to exit.")

    try:
        step = 0
        while True:
            p.stepSimulation()
            step += 1

            # Every N steps, update the joint motors from the creature's motors
            if step % 24 == 0:
                update_motors_for_ga(robot_id, cr)

            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Exiting simulation...")
    finally:
        p.disconnect()


# =========================
# HELPERS FOR GA (reuse this from ga_mountain.py)
# =========================

def setup_world_for_ga(arena_size=20):
    """
    Reset the physics world and rebuild the arena + mountain.
    Used by the GA (in DIRECT mode), so there is NO GUI here.
    Assumes p.connect(...) has already been called.
    """
    p.resetSimulation()
    p.setPhysicsEngineParameter(enableFileCaching=0)

    # standard gravity and search path
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # our arena + mountain
    make_arena(arena_size=arena_size)
    load_mountain()


def update_motors_for_ga(cid, cr):
    """
    Copy of the logic from simulation.update_motors, but using the
    default physics client. This lets the GA drive the joints.
    """
    motors = cr.get_motors()
    for jid in range(p.getNumJoints(cid)):
        m = motors[jid]
        p.setJointMotorControl2(
            cid,
            jid,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=m.get_output(),
            force=15,
        )


def run_creature_on_mountain(dna, iterations=2400, start_pos=(0, -4, 3)):
    """
    Core evaluation function for the GA.

    dna: list of numpy arrays (same structure as Creature.dna)
    returns: fitness = height gained during the simulation
    """
    # build the world
    setup_world_for_ga(arena_size=20)

    # build creature with this dna
    gene_count = len(dna)
    cr = creature.Creature(gene_count=gene_count)
    cr.update_dna(dna)

    # write URDF and load into PyBullet
    xml_file = "ga_temp.urdf"
    with open(xml_file, "w") as f:
        f.write(cr.to_xml())

    start_orn = p.getQuaternionFromEuler((0, 0, 0))
    cid = p.loadURDF(xml_file, start_pos, start_orn)

    # start height = z of spawn position
    start_z = start_pos[2]
    max_height = start_z

    for step in range(iterations):
        p.stepSimulation()

        # drive motors every few frames (same as lecturer’s Simulation)
        if step % 24 == 0:
            update_motors_for_ga(cid, cr)

        pos, _ = p.getBasePositionAndOrientation(cid)
        cr.update_position(pos)

        z = pos[2]
        if z > max_height:
            max_height = z

    # fitness = how much higher it got than its starting position
    fitness = max_height - start_z
    return fitness



if __name__ == "__main__":
    main()