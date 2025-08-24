import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.rcParams['font.family'] = ['Hiragino Sans', 'Arial']

collision_data = pd.read_csv("data/collision/30agents_collision.csv")
coverage_array = pd.read_csv("input/icover.csv").values

plt.figure(figsize=(12, 10))

colored_map = np.zeros((*coverage_array.shape, 3))
colored_map[coverage_array == -1] = [1.0, 1.0, 1.0]  # 白
colored_map[coverage_array == 0] = [0.2, 0.6, 1.0]   # 青色
colored_map[coverage_array == 7] = [0.3, 0.7, 0.3]   # 緑色
plt.imshow(colored_map, origin='upper', alpha=0.7)

x = collision_data['x'].values
y = 65000 - collision_data['y'].values
map_h, map_w = coverage_array.shape

x_scaled = x / 36000 * map_w
y_scaled = y / 66000 * map_h
y_flipped = map_h - y_scaled

plt.scatter(x_scaled, y_flipped, c='red', s=50, alpha=0.8, 
           marker='o', edgecolors='darkred', linewidth=1)

plt.title('Collision Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True, alpha=0.3)
plt.gca().set_aspect('equal')
plt.tight_layout()
plt.savefig('collision_map.png', dpi=300)
plt.show() 