import laspy
import open3d as o3d
import numpy as np
import scipy.linalg as la
from scipy import ndimage
import matplotlib.pyplot as plt

# ==========================================
# 2. CHARGEMENT ET SOL (RANSAC)
# ==========================================

las = laspy.read("railway_000033.laz")
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.vstack((las.x, las.y, las.z)).T)
pcd = pcd.voxel_down_sample(voxel_size=0.2) 

# 2. NETTOYAGE DU BRUIT (L'équivalent du SOR de CloudCompare)
# nb_neighbors : nombre de voisins à regarder pour chaque point (ex: 20)
# std_ratio : plus il est petit, plus le filtre est agressif (ex: 2.0)
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=3.0)
pcd = pcd.select_by_index(ind)

# RANSAC serré pour ne pas empiéter sur les objets
plane_model, inliers = pcd.segment_plane(distance_threshold=0.5, ransac_n=3, num_iterations=1000)
ground_cloud = pcd.select_by_index(inliers)
object_cloud = pcd.select_by_index(inliers, invert=True)
ground_cloud.paint_uniform_color([1, 0, 0]) # Rouge

# Filtrage vertical strict par rapport au plan
a, b, c, d = plane_model
obj_pts = np.asarray(object_cloud.points)
dist = (a * obj_pts[:, 0] + b * obj_pts[:, 1] + c * obj_pts[:, 2] + d)
filtered_objects = object_cloud.select_by_index(np.where(dist > 0.2)[0])

# ==========================================
# 3. CLUSTERING ET ANALYSE DE FORME
# ==========================================


# 1. Appliquer DBSCAN
# eps=0.5 (50cm de rayon), min_points=50
labels = np.array(filtered_objects.cluster_dbscan(eps=1.2, min_points=20, print_progress=True))

# 2. Compter le nombre d'objets trouvés
max_label = labels.max()
print(f"Le nuage contient {max_label + 1} objets distincts")

# 3. Colorer chaque objet avec une couleur aléatoire
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0  # Les points considérés comme "bruit" seront en noir
filtered_objects.colors = o3d.utility.Vector3dVector(colors[:, :3])

# 4. Affichage final
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()
visualizer.add_geometry(ground_cloud)
visualizer.add_geometry(filtered_objects)
render_option = visualizer.get_render_option()
render_option.background_color = np.asarray([0, 0, 0]) # Fond NOIR
visualizer.run()
visualizer.destroy_window()