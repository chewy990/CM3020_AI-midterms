import numpy as np
import pybullet as p

import creature
import cw_envt_copy as env   # <- rename if your file name is different


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

def genetic_algorithm():
    population = [random_dna() for _ in range(POP_SIZE)]

    # For logging
    history = []  # list of (generation, best, mean)

    for gen in range(NUM_GENERATIONS):
        print(f"\nGeneration {gen+1}/{NUM_GENERATIONS}")

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

        # ---- simple evolution: elitism + mutated copies ----
        new_population = []

        elite_dna = clone_dna(population[best_idx])
        new_population.append(elite_dna)

        while len(new_population) < POP_SIZE:
            child = mutate_dna(elite_dna)
            new_population.append(child)

        population = new_population

    # Save log to CSV for later plotting
    np.savetxt(
        "ga_mountain_log.csv",
        np.array(history),
        delimiter=",",
        header="generation,best_fitness,mean_fitness",
        comments=""
    )

    return population, elite_dna



if __name__ == "__main__":
    final_pop, best_dna = genetic_algorithm()

    # Optional: run the best creature in GUI so you can watch it
    print("\nRunning best creature in GUI...")
    p.connect(p.GUI)
    env.watch_creature_on_mountain(best_dna, iterations=ITERATIONS)

    input("Press Enter to quit...")
    p.disconnect()
