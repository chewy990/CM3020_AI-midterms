import numpy as np
import pybullet as p

import creature
import cw_envt_copy as env   # environment + fitness functions

# =========================
# GA CONFIGURATION
# =========================

# Choose which experiment to run:
# "full"        -> evolve morphology + motors
# "motors_only" -> fix morphology, evolve motors only
MODE = "full"

POP_SIZE = 10          # number of individuals per generation
NUM_GENERATIONS = 5  # how many generations to evolve for
ITERATIONS = 1600      # simulation steps per fitness evaluation

MUTATION_RATE = 0.15   # probability of mutating each gene element
MUTATION_STD = 0.15    # standard deviation of Gaussian mutation noise

LOG_FILE = "test.csv"

# =========================
# DNA UTILITIES
# =========================

# Use a probe creature to discover the DNA structure (shapes of the gene arrays)
_probe = creature.Creature(gene_count=3)
_TEMPLATE_DNA = _probe.dna  # list of numpy arrays matching Creature.dna


def random_dna():
    """
    Create a new random DNA with the same shapes as the template.
    Values are initialised in [0, 1), matching the original genome design.
    """
    new_dna = []
    for gene in _TEMPLATE_DNA:
        shape = gene.shape
        new_gene = np.random.rand(*shape)
        new_dna.append(new_gene)
    return new_dna


def clone_dna(dna):
    """
    Deep-copy a DNA (list of numpy arrays).
    """
    return [g.copy() for g in dna]


def mutate_dna(dna, rate=MUTATION_RATE, std=MUTATION_STD):
    """
    Mutate the full DNA (morphology + motors).

    Adds Gaussian noise to a subset of elements in each gene, then clamps to
    [0, 0.999] to avoid genome crashes (e.g. invalid parent indices at 1.0).
    """
    new = []
    for gene in dna:
        g = gene.copy()
        mask = np.random.rand(*g.shape) < rate
        g[mask] += np.random.normal(0, std, size=mask.sum())
        # IMPORTANT: upper bound is < 1.0 to keep parent indices valid
        g = np.clip(g, 0.0, 0.999)
        new.append(g)
    return new


def mutate_motors_only(dna, rate=MUTATION_RATE, std=MUTATION_STD):
    """
    Mutate ONLY the motor-related genes.

    Assumes the last gene in the DNA list encodes motor parameters.
    Morphology-related genes are kept fixed to isolate control evolution.
    """
    new = []
    num_genes = len(dna)
    motor_gene_index = num_genes - 1  # assume last gene = motors

    for idx, gene in enumerate(dna):
        g = gene.copy()
        if idx == motor_gene_index:
            # apply Gaussian noise to a subset of motor parameters
            mask = np.random.rand(*g.shape) < rate
            g[mask] += np.random.normal(0, std, size=mask.sum())
            # keep values inside [0, 1) to avoid genome crashes
            g = np.clip(g, 0.0, 0.999)
        # else: body/structure genes stay exactly the same
        new.append(g)

    return new


# =========================
# FITNESS EVALUATION WRAPPER
# =========================

def evaluate_dna(dna, gui=False):
    """
    Evaluate one DNA by running the creature on the mountain and returning fitness.

    If gui=False, use PyBullet DIRECT mode (fast, no window).
    If gui=True, use GUI mode (for debugging / visual inspection).
    """
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    fitness = env.run_creature_on_mountain(dna, iterations=ITERATIONS)
    p.disconnect()
    return fitness


# =========================
# GA LOOPS
# =========================

def genetic_algorithm_full():
    """
    Baseline GA: evolve the full DNA (morphology + motors).
    Uses elitism and mutation-only reproduction.
    """
    population = [random_dna() for _ in range(POP_SIZE)]
    history = []  # list of (generation, best_fitness, mean_fitness)

    for gen in range(NUM_GENERATIONS):
        print(f"\n[Full] Generation {gen+1}/{NUM_GENERATIONS}")

        fitnesses = np.zeros(POP_SIZE)
        for i, dna in enumerate(population):
            fit = evaluate_dna(dna, gui=False)
            fitnesses[i] = fit
            print(f"  Individual {i+1}: fitness = {fit:.3f}")

        # record best and mean for logging / plotting
        best_idx = np.argmax(fitnesses)
        best_fit = fitnesses[best_idx]
        mean_fit = np.mean(fitnesses)
        print(f"  Best fitness: {best_fit:.3f}")
        print(f"  Mean fitness: {mean_fit:.3f}")

        history.append((gen+1, best_fit, mean_fit))

        # ---- elitism + mutated copies (full DNA mutates) ----
        new_population = []
        elite_dna = clone_dna(population[best_idx])  # best individual survives
        new_population.append(elite_dna)

        # fill the rest of the population with mutated copies of the elite
        while len(new_population) < POP_SIZE:
            child = mutate_dna(elite_dna)
            new_population.append(child)

        population = new_population

    # Save fitness history for plotting in Jupyter
    np.savetxt(
        LOG_FILE,
        np.array(history),
        delimiter=",",
        header="generation,best_fitness,mean_fitness",
        comments=""
    )

    return population, elite_dna


def genetic_algorithm_motors_only():
    """
    Encoding experiment: fix the body, evolve ONLY the motor parameters.

    Same GA structure, but mutation is restricted to the motor gene.
    """
    population = [random_dna() for _ in range(POP_SIZE)]
    history = []  # list of (generation, best_fitness, mean_fitness)

    for gen in range(NUM_GENERATIONS):
        print(f"\n[Motors-only] Generation {gen+1}/{NUM_GENERATIONS}")

        fitnesses = np.zeros(POP_SIZE)
        for i, dna in enumerate(population):
            fit = evaluate_dna(dna, gui=False)
            fitnesses[i] = fit
            print(f"  Individual {i+1}: fitness = {fit:.3f}")

        # record best and mean for logging / plotting
        best_idx = np.argmax(fitnesses)
        best_fit = fitnesses[best_idx]
        mean_fit = np.mean(fitnesses)
        print(f"  Best fitness: {best_fit:.3f}")
        print(f"  Mean fitness: {mean_fit:.3f}")

        history.append((gen+1, best_fit, mean_fit))

        # ---- elitism + mutated copies (motors only mutate) ----
        new_population = []
        elite_dna = clone_dna(population[best_idx])  # best individual survives
        new_population.append(elite_dna)

        # fill the rest of the population with mutated copies of the elite
        while len(new_population) < POP_SIZE:
            child = mutate_motors_only(elite_dna)
            new_population.append(child)

        population = new_population

    # Save fitness history for plotting in Jupyter
    np.savetxt(
        "ga_motors_only_log.csv",
        np.array(history),
        delimiter=",",
        header="generation,best_fitness,mean_fitness",
        comments=""
    )

    return population, elite_dna


# =========================
# MAIN ENTRY POINT
# =========================

if __name__ == "__main__":
    # MODE selects which encoding experiment to run
    # "full"        -> baseline (body + motors evolve)
    # "motors_only" -> fixed body, only motors evolve
    if MODE == "full":
        final_pop, best_dna = genetic_algorithm_full()
        np.save("best_dna_full.npy", best_dna, allow_pickle=True)
        print("Saved best DNA to best_dna_full.npy")
    elif MODE == "motors_only":
        final_pop, best_dna = genetic_algorithm_motors_only()
        np.save("best_dna_motors_only.npy", best_dna, allow_pickle=True)
        print("Saved best DNA to best_dna_motors_only.npy")
    else:
        raise ValueError("Unknown MODE, use 'full' or 'motors_only'.")

    # After evolution, replay the best creature in GUI for visual inspection
    print("\nRunning best creature in GUI...")
    p.connect(p.GUI)

    try:
        # Slower, visual-friendly playback if implemented
        env.watch_creature_on_mountain(best_dna, iterations=ITERATIONS)
    except AttributeError:
        # Fallback to a normal evaluation if no watch function exists
        env.run_creature_on_mountain(best_dna, iterations=ITERATIONS)

    input("Press Enter to quit...")
    p.disconnect()


# ============= END OF MY CODE ====================