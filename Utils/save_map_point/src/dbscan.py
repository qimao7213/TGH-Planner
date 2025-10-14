import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull
import os

def read_points_from_txt(file_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split())
            points.append([x, y])
    return np.array(points)

def save_clusters_to_txt(file_path, cluster_centers):
    with open(file_path, 'w') as file:
        for center in cluster_centers:
            file.write(f"{center[0]} {center[1]}\n")

def save_convex_hull_vertices(file_path, convex_hull_vertices):
    with open(file_path, 'w') as file:
        for vertices in convex_hull_vertices:
            for vertex in vertices:
                file.write(f"{vertex[0]} {vertex[1]}\n")
            # file.write("\n")  # 添加空行分隔不同的凸壳

def plot_convex_hulls(points, labels, unique_labels, convex_hull_vertices):
    for label in unique_labels:
        if label == -1:  # 跳过噪声点
            continue
        cluster_points = points[labels == label]
        hull = ConvexHull(cluster_points)
        for simplex in hull.simplices:
            plt.plot(cluster_points[simplex, 0], cluster_points[simplex, 1], 'k-')
            # 标注聚类编号
            center = np.mean(cluster_points, axis=0)
            plt.text(center[0], center[1], f'{label}', fontsize=12, color='red', weight='bold')

        convex_hull_vertices.append(cluster_points[hull.vertices])

def main():
    input_file = "../map_point/forest/map_point.txt"
    output_file = "../map_pointt/forest/map_point_cluster.txt"
    convex_hull_file = "../map_pointt/forest/convex_hull_vertices.txt"

    points = read_points_from_txt(input_file)

    dbscan = DBSCAN(eps=0.15, min_samples=5)
    labels = dbscan.fit_predict(points)

    unique_labels = set(labels)
    cluster_centers = []
    convex_hull_vertices = []
    for label in unique_labels:
        if label == -1:  # 跳过噪声点
            continue
        cluster_points = points[labels == label]
        center = np.mean(cluster_points, axis=0)
        cluster_centers.append(center)

        # 计算凸度
        hull = ConvexHull(cluster_points)
        hull_area = hull.volume  # 对于2D凸包，volume 就是面积
        # 点集面积可以用 Delaunay 三角剖分求近似（或用 α-shape 更精确）
        # 但简化处理：我们用点集的“包围面积”来估算
        cluster_area = len(cluster_points) * 1e-4  # 假设点密度均匀，每点占据一个小面积
        convexity = min(cluster_area / hull_area * 100, 1.0)

        print(f"Cluster {label}: Convexity ≈ {convexity:.4f} | Points: {len(cluster_points)}, Hull area: {hull_area:.6f}")

    plt.figure(figsize=(8, 6))
    plt.scatter(points[:, 0], points[:, 1], c=labels, cmap='viridis', marker='o', s=10, label='Original Points')
    if cluster_centers:
        cluster_centers = np.array(cluster_centers)
        plt.scatter(cluster_centers[:, 0], cluster_centers[:, 1], c='red', marker='x', s=100, label='Cluster Centers')

    plot_convex_hulls(points, labels, unique_labels, convex_hull_vertices)

    plt.legend()
    plt.title('DBSCAN Clustering with Convex Hulls')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()

    save_clusters_to_txt(output_file, cluster_centers)
    print(f"Cluster centers have been saved to {output_file}")

    save_convex_hull_vertices(convex_hull_file, convex_hull_vertices)
    print(f"Convex hull vertices have been saved to {convex_hull_file}")

if __name__ == '__main__':
    main()