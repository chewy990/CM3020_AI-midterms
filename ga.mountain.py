import numpy as np
import pybullet as p
import time

POP_SIZE = 10
GENOME_LENGTH = 10
NUM_GENERATIONS = 5
SIM_STEPS = 600

def setup_world():
    """
    TODO: copy the world-setup code from cw-envt.py into here.
    This should:
      - load the plane/sandbox
      - load the mountain
      - set gravity, etc.
    """
    # Example only – you will replace this with actual code from cw-envt.py
    # p.loadURDF("plane.urdf")
    # p.setGravity(0, 0, -9.8)
    pass

def create_creature_from_genome(genome):
    """
    TODO: copy the code that creates a creature in cw-envt.py into here.
    For now, you can ignore the genome and just create the same kind of creature.
    Later, you can use genome values to change its motors/body.
    """
    # Example placeholder:
    # creature_id = p.loadURDF("some_robot.urdf")
    creature_id = None
    return creature_id

def run_simulation(creature_id):
    """
    Simulate the creature and return a fitness value (how high it climbs).
    """
    max_height = -1e9
    for step in range(SIM_STEPS):
        # TODO: send motor commands if you have them
        p.stepSimulation()

        if creature_id is not None:
            pos, _ = p.getBasePositionAndOrientation(creature_id)
            z = pos[2]
            if z > max_height:
                max_height = z

    return max_height

def evaluate_genome(genome, gui=False):
    """
    Run one full evaluation of a genome in PyBullet and return fitness.
    """
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setGravity(0, 0, -9.8)
    setup_world()
    creature_id = create_creature_from_genome(genome)
    fitness = run_simulation(creature_id)
    p.disconnect()
    return fitness

def init_population(pop_size, genome_length):
    return np.random.uniform(-1, 1, size=(pop_size, genome_length))

def genetic_algorithm():
    population = init_population(POP_SIZE, GENOME_LENGTH)

    for gen in range(NUM_GENERATIONS):
        print(f"Generation {gen+1}/{NUM_GENERATIONS}")
        fitnesses = []
        for i, genome in enumerate(population):
            fit = evaluate_genome(genome, gui=False)
            fitnesses.append(fit)
            print(f"  Individual {i+1}: fitness = {fit:.3f}")
        fitnesses = np.array(fitnesses)
        print(f"  Best fitness this gen: {fitnesses.max():.3f}")

        # TODO later: selection, crossover, mutation, etc.
        # For now, you can just keep the same population to check things work.

    return population

if __name__ == "__main__":
    final_pop = genetic_algorithm()
    print("Done.")
