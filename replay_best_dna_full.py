import numpy as np
import pybullet as p

import cw_envt_copy as env  # your environment file

# 1. Load saved DNA (adjust filename if needed)
dna = np.load("best_dna_full.npy", allow_pickle=True)

# If dna loads as a numpy array of objects and you prefer a plain list:
# dna = dna.tolist()

# 2. Start PyBullet GUI
p.connect(p.GUI)

# 3. Watch the creature on the mountain
env.watch_creature_on_mountain(dna, iterations=1600)

# 4. Keep the window open until you press Enter in the terminal
input("Press Enter to quit...")
p.disconnect()
