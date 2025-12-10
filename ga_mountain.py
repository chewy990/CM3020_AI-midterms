import numpy as np
import pybullet as p

import creature
import cw_envt_copy as env   # <- rename if your file name is different

MODE = "full"
POP_SIZE = 30
NUM_GENERATIONS = 30
ITERATIONS = 1600
MUTATION_RATE = 0.15
MUTATION_STD = 0.15


# -------------------------
# DNA utilities
# -------------------------

# Use a probe creature to discover the DNA structure
_probe = creature.Creature(gene_count=3)
_TEMPLATE_DNA = _probe.dna  # list of numpy arrays


def random_dna():
    """
    Create a new random DNA with the same shapes as the template.
    Values are in [0,1], which matches the original genome design.
    """
    new_dna = []
    for gene in _TEMPLATE_DNA:
        shape = gene.shape
        new_gene = np.random.rand(*shape)
        new_dna.append(new_gene)
    return new_dna


def clone_dna(dna):
    return [g.copy() for g in dna]


def mutate_dna(dna, rate=MUTATION_RATE, std=MUTATION_STD):
    """
    Add Gaussian noise to some elements of each gene, then clamp to [0, 0.999].
    This avoids genome.py crashes when 'joint-parent' etc. becomes exactly 1.0.
    """
    new = []
    for gene in dna:
        g = gene.copy()
        mask = np.random.rand(*g.shape) < rate
        g[mask] += np.random.normal(0, std, size=mask.sum())
        # IMPORTANT: upper bound is < 1.0, to keep parent indices valid
        g = np.clip(g, 0.0, 0.999)
        new.append(g)
    return new

def mutate_motors_only(dna, rate=MUTATION_RATE, std=MUTATION_STD):
    """
    Mutate ONLY the motor-related genes.
    Assumes the last gene in the DNA list corresponds mainly to motor parameters.
    If your Creature encoding is different, you can adjust which indices are mutated.
    """
    new = []
    num_genes = len(dna)
    motor_gene_index = num_genes - 1  # assume last gene = motors

    for idx, gene in enumerate(dna):
        g = gene.copy()
        if idx == motor_gene_index:
            # apply Gaussian noise to some elements
            mask = np.random.rand(*g.shape) < rate
            g[mask] += np.random.normal(0, std, size=mask.sum())
            # keep values inside [0, 1) to avoid genome crashes
            g = np.clip(g, 0.0, 0.999)
        # else: body/structure genes stay exactly the same
        new.append(g)

    return new



# -------------------------
# Evaluation
# -------------------------

def evaluate_dna(dna, gui=False):
    """
    Connect to PyBullet, run the creature on the mountain, return fitness.
    """
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    fitness = env.run_creature_on_mountain(dna, iterations=ITERATIONS)
    p.disconnect()
    return fitness


# -------------------------
# Simple GA loop
# -------------------------

def genetic_algorithm_full():
    """
    Baseline GA: evolve full DNA (body + motors).
    """
    population = [random_dna() for _ in range(POP_SIZE)]
    history = []  # (generation, best, mean)

    for gen in range(NUM_GENERATIONS):
        print(f"\n[Full] Generation {gen+1}/{NUM_GENERATIONS}")

        fitnesses = np.zeros(POP_SIZE)
        for i, dna in enumerate(population):
            fit = evaluate_dna(dna, gui=False)
            fitnesses[i] = fit
            print(f"  Individual {i+1}: fitness = {fit:.3f}")

        best_idx = np.argmax(fitnesses)
        best_fit = fitnesses[best_idx]
        mean_fit = np.mean(fitnesses)
        print(f"  Best fitness: {best_fit:.3f}")
        print(f"  Mean fitness: {mean_fit:.3f}")

        history.append((gen+1, best_fit, mean_fit))

        # ---- elitism + mutated copies (full DNA mutates) ----
        new_population = []
        elite_dna = clone_dna(population[best_idx])
        new_population.append(elite_dna)

        while len(new_population) < POP_SIZE:
            child = mutate_dna(elite_dna)
            new_population.append(child)

        population = new_population

    # Save log to CSV
    np.savetxt(
        "ga_full_log.csv",
        np.array(history),
        delimiter=",",
        header="generation,best_fitness,mean_fitness",
        comments=""
    )

    return population, elite_dna


def genetic_algorithm_motors_only():
    """
    Encoding experiment: fix the body, evolve ONLY the motor parameters.
    """
    population = [random_dna() for _ in range(POP_SIZE)]
    history = []  # (generation, best, mean)

    for gen in range(NUM_GENERATIONS):
        print(f"\n[Motors-only] Generation {gen+1}/{NUM_GENERATIONS}")

        fitnesses = np.zeros(POP_SIZE)
        for i, dna in enumerate(population):
            fit = evaluate_dna(dna, gui=False)
            fitnesses[i] = fit
            print(f"  Individual {i+1}: fitness = {fit:.3f}")

        best_idx = np.argmax(fitnesses)
        best_fit = fitnesses[best_idx]
        mean_fit = np.mean(fitnesses)
        print(f"  Best fitness: {best_fit:.3f}")
        print(f"  Mean fitness: {mean_fit:.3f}")

        history.append((gen+1, best_fit, mean_fit))

        # ---- elitism + mutated copies (ONLY motors mutate) ----
        new_population = []
        elite_dna = clone_dna(population[best_idx])
        new_population.append(elite_dna)

        while len(new_population) < POP_SIZE:
            child = mutate_motors_only(elite_dna)
            new_population.append(child)

        population = new_population

    # Save log to CSV
    np.savetxt(
        "ga_motors_only_log.csv",
        np.array(history),
        delimiter=",",
        header="generation,best_fitness,mean_fitness",
        comments=""
    )

    return population, elite_dna


if __name__ == "__main__":
    # Choose which experiment to run:
    # "full"        -> baseline (body + motors evolve)
    # "motors_only" -> fixed body, only motors evolve
    MODE = "full"  # change this and re-run to switch

    if MODE == "full":
        final_pop, best_dna = genetic_algorithm_full()
    elif MODE == "motors_only":
        final_pop, best_dna = genetic_algorithm_motors_only()
    else:
        raise ValueError("Unknown MODE, use 'full' or 'motors_only'.")

    print("\nRunning best creature in GUI...")
    p.connect(p.GUI)

    # If you implemented watch_creature_on_mountain, use that for slow, visible playback:
    try:
        env.watch_creature_on_mountain(best_dna, iterations=ITERATIONS)
    except AttributeError:
        # fallback: fast evaluation, may finish quickly
        env.run_creature_on_mountain(best_dna, iterations=ITERATIONS)

    input("Press Enter to quit...")
    p.disconnect()

