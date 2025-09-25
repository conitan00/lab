import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import glob

# plt.rcParams['font.family'] = ['Hiragino Sans', 'Arial']

output_dirs = glob.glob("../output/*/")

coverage_array = pd.read_csv("../input/icover.csv").values

for output_dir in output_dirs:
    collision_file = os.path.join(output_dir, "collision.csv")

    if os.path.exists(collision_file):
        collision_data = pd.read_csv(collision_file)

        plt.figure(figsize=(12, 10))

        colored_map = np.zeros((*coverage_array.shape, 3))
        colored_map[coverage_array == -1] = [1.0, 1.0, 1.0]
        colored_map[coverage_array == 0] = [0.2, 0.6, 1.0]
        colored_map[coverage_array == 7] = [0.3, 0.7, 0.3]
        plt.imshow(colored_map, origin='upper', alpha=0.7)

        x = collision_data['x'].values
        y = 65000 - collision_data['y'].values
        map_h, map_w = coverage_array.shape

        x_scaled = x / 36000 * map_w
        y_scaled = y / 66000 * map_h
        y_flipped = map_h - y_scaled

        plt.scatter(x_scaled, y_flipped, c='red', s=50, alpha=0.8,
                   marker='o', edgecolors='darkred', linewidth=1)

        plt.title(f'Collision Points - {os.path.basename(output_dir.rstrip("/"))}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True, alpha=0.3)
        plt.gca().set_aspect('equal')
        plt.tight_layout()

        output_file = os.path.join(output_dir, "collision_visualization.png")
        plt.savefig(output_file, dpi=300)
        plt.close()

        print(f"Generated: {output_file}")

print("Collision visualization completed for all output directories.") 