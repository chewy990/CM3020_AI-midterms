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

    p.setAdditionalSearchPath("shapes/")

    mountain_id = p.loadURDF(
        "gaussian_pyramid.urdf",
        mountain_position,
        mountain_orientation,
        useFixedBase=1,
    )

    # Make the mountain non-bouncy and reasonably grippy
    p.changeDynamics(mountain_id, -1, restitution=0.0, lateralFriction=2.5)

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

    # Spawn creature on mountain
    start_pos = (0, 0, 6)   # y closer to 0, z a bit higher
    start_orn = p.getQuaternionFromEuler((0, 0, 0))
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn)

    # Make creature less bouncy and a bit grippy
    p.changeDynamics(robot_id, -1, restitution=0.0, lateralFriction=2.0)

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

            update_motors_for_ga(robot_id, cr)
            clamp_base_velocity(robot_id, max_lin=3.0, max_ang=3.0)

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
    motors = cr.get_motors()
    for jid in range(p.getNumJoints(cid)):
        m = motors[jid]
        vel = m.get_output() * 4.0
        p.setJointMotorControl2(
            cid,
            jid,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=vel,
            force=40,
        )

def update_motors_for_view(cid, cr, scale=3.0):
    """
    Stronger motors for VISUALISATION ONLY.
    This does NOT affect GA fitness, only the GUI demo.
    """
    motors = cr.get_motors()
    for jid in range(p.getNumJoints(cid)):
        m = motors[jid]
        vel = m.get_output() * scale   # exaggerate motion
        p.setJointMotorControl2(
            cid,
            jid,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=vel,
            force=40,                  # stronger force for demo
        )


def clamp_base_velocity(cid, max_lin=3.0, max_ang=3.0):
    """
    Limit the robot's linear and angular velocity so it cannot 'rocket' away.
    """
    lin_vel, ang_vel = p.getBaseVelocity(cid)

    # Clamp linear velocity
    lx, ly, lz = lin_vel
    lin_speed = math.sqrt(lx*lx + ly*ly + lz*lz)
    if lin_speed > max_lin and lin_speed > 1e-6:
        scale = max_lin / lin_speed
        lx *= scale
        ly *= scale
        lz *= scale
    else:
        scale = 1.0  # not really used, but fine

    # Clamp angular velocity
    ax, ay, az = ang_vel
    ang_speed = math.sqrt(ax*ax + ay*ay + az*az)
    if ang_speed > max_ang and ang_speed > 1e-6:
        ascale = max_ang / ang_speed
        ax *= ascale
        ay *= ascale
        az *= ascale

    p.resetBaseVelocity(cid, [lx, ly, lz], [ax, ay, az])



def run_creature_on_mountain(dna, iterations=2400, start_pos=(3, 0, 3)):
    """
    Core evaluation function for the GA.

    dna: list of numpy arrays (same structure as Creature.dna)
    returns: fitness = how well it climbs and stays on the mountain
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

    # Creature dynamics: no bounce, some friction
    p.changeDynamics(cid, -1, restitution=0.0, lateralFriction=2.0)


    # ---------------------------
    # FITNESS MEASUREMENT
    # ---------------------------

    start_z = start_pos[2]

    # starting radial distance from centre (we spawn off-centre now)
    start_r = math.sqrt(start_pos[0]**2 + start_pos[1]**2)
    best_r = start_r

    # Track *average* climbing while near the mountain centre
    total_xy_dist = 0.0
    steps = 0
    time_on_mountain = 0
    sum_height_on_mountain = 0.0

    # radius that roughly covers your mountain footprint
    MOUNTAIN_RADIUS = 4.0
    MAX_CLIMB_REWARD = 4.0  # only reward up to +4m above start

    for step in range(iterations):
        p.stepSimulation()
        
        update_motors_for_ga(cid, cr)

        clamp_base_velocity(cid, max_lin=3.0, max_ang=3.0)

        pos, _ = p.getBasePositionAndOrientation(cid)
        cr.update_position(pos)

        x, y, z = pos
        r = (x ** 2 + y ** 2) ** 0.5  # distance from centre (0,0)

        total_xy_dist += r
        steps += 1

        # track closest we ever got to the centre (towards the top)
        if r < best_r:
            best_r = r

        # Are we roughly on the mountain (not at the far walls / below floor)?
        if r < MOUNTAIN_RADIUS and z > start_z - 0.5:
            time_on_mountain += 1

            # Clamp height so crazy rocket jumps don't dominate
            z_clamped = min(z, start_z + MAX_CLIMB_REWARD)
            sum_height_on_mountain += z_clamped

    # ---- morphology penalty (size) ----
    aabb_min, aabb_max = p.getAABB(cid)
    extent_x = aabb_max[0] - aabb_min[0]
    extent_y = aabb_max[1] - aabb_min[1]
    extent_z = aabb_max[2] - aabb_min[2]
    size_penalty = 0.01 * (extent_x + extent_y)

    # ---- aggregate fitness terms ----
    avg_xy_dist = total_xy_dist / max(1, steps)

    if time_on_mountain > 0:
        avg_height_on_mountain = sum_height_on_mountain / time_on_mountain
    else:
        avg_height_on_mountain = start_z

    # climb: average height while ON the mountain
    climb_height = avg_height_on_mountain - start_z

    # time spent actually on the mountain (normalised)
    time_fraction = time_on_mountain / max(1, steps)

    radial_improvement = max(0.0, start_r - best_r)


    fitness = (
        2.0 * radial_improvement   # MAIN goal: get closer to the peak
        + 1.0 * climb_height       # main goal: be higher on average
        + 0.8 * time_fraction  # strong bonus: stay on the slope
        + 0.2 * radial_improvement  # reward moving towards the top
        - 0.02 * avg_xy_dist   # penalty: hang out far from centre
        - size_penalty         # penalty: very wide shapes
    )

    # keep bad individuals from dominating selection numerically
    fitness = max(fitness, -3.0)
    return fitness





def watch_creature_on_mountain(dna, iterations=2400, start_pos=(3, 0, 3)):
    """
    Same as run_creature_on_mountain, but:
    - sets a nice camera
    - slows the simulation down so you can actually SEE the creature move
    - does not return fitness (it's just for visualisation)
    Assumes p.connect(...) has already been called (GUI or DIRECT).
    """
    # Build the world (arena + mountain)
    setup_world_for_ga(arena_size=20)

    # Build creature with this dna
    gene_count = len(dna)
    cr = creature.Creature(gene_count=gene_count)
    cr.update_dna(dna)

    # Write URDF and load into PyBullet
    xml_file = "ga_view.urdf"
    with open(xml_file, "w") as f:
        f.write(cr.to_xml())

    start_orn = p.getQuaternionFromEuler((0, 0, 0))
    cid = p.loadURDF(xml_file, start_pos, start_orn)
    
    # Creature dynamics for viewing
    p.changeDynamics(cid, -1, restitution=0.0, lateralFriction=1.2)

    # Nice camera pointing at the centre of the arena
    p.resetDebugVisualizerCamera(
        cameraDistance=18,
        cameraYaw=45,
        cameraPitch=-35,
        cameraTargetPosition=[0, 0, 2],
    )

    for step in range(iterations):
        p.stepSimulation()

        update_motors_for_view(cid, cr, scale=3.0)
        
        clamp_base_velocity(cid, max_lin=3.0, max_ang=3.0)
        time.sleep(1.0 / 240.0)



if __name__ == "__main__":
    main()