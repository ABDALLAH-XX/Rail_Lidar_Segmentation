# 🚂 LiDAR-Based Railway Infrastructure Segmentation (Non-AI Approach)

![Current Segmentation Result](results/railway_000033_PCL.png)

## 🎯 Project Overview
The objective of this project is to **segment and isolate critical railway infrastructure components** (tracks, catenaries, poles) from raw 3D LiDAR point clouds. 

This project specifically explores a **Rule-Based / Heuristic approach**, deliberately avoiding Deep Learning (AI). This ensures a lightweight, explainable, and computationally efficient solution suitable for edge computing or real-time monitoring systems.

---

## 📊 Dataset Source
The data used in this project is sourced from the French Open Data platform:
* **Dataset Name:** [Nuage de points 3D des infrastructures ferroviaires](https://www.data.gouv.fr/datasets/nuage-de-points-3d-des-infrastructures-ferroviaires)
* **Source:** Data.gouv.fr / SNCF Réseau
* **Format:** The raw dataset provides download links to compressed **.laz** tiles.

---

## 📈 Project Evolution: From Prototyping to Performance
The development of this pipeline followed a two-step engineering approach:

1.  **Phase 1: Prototyping (Python & Open3D):** Initial R&D was conducted using **Open3D** to validate geometric heuristics.
2.  **Phase 2: Production-Ready (C++ & PCL):** To achieve industrial performance and handle large-scale railway tiles, the core engine was migrated to **C++ 17** using the **Point Cloud Library (PCL)**.

---

## 🏗️ Technical Achievements [Work in Progress 🚧]

### 1. Slope-Aware Ground Extraction
The track bed (ballast/rails) is isolated using a **RANSAC Plane Model**. Unlike static filters, this model calculates height **relative to the plane**, allowing the algorithm to follow the natural incline/slope of the railway line.

### 2. Infrastructure Classification (Poles & Catenaries)
The algorithm uses a multi-step deterministic logic to differentiate between diverse structural types:
* **Standard Pole Detection (e.g., Pole 235): [Validated]** Successful isolation of vertical supports using height-to-width ratios and ground-anchoring verification ($z_{min} < 0.8m$ relative to the slope).
* **Complex Catenary Portals (e.g., Structures 112 & 172): [In Progress]** Refinement of the segmentation for heavy support structures. Current focus is on robustly separating the catenary arms (consoles) from the main poles.
* **Deterministic Filtering:** Architecture where objects are classified by geometric properties (**Linearity via PCA**, verticality, and relative height) rather than opaque neural network weights.

### 3. Real-Time Slope Analysis
The system automatically segments the ground into "Low" and "High" zones to calculate:
* **Z-Mean Values:** Average absolute altitude of the ballast at both ends of the segment.
* **Slope Percentage:** Estimated gradient of the track, essential for normalizing height measurements across the entire line.

---

## ⚙️ Methodology & Empirical Tuning
A core challenge of this project was the **Empirical Optimization** of the algorithm. All parameters were determined through iterative testing to find the optimal balance between noise reduction and feature preservation.

### 🛠️ The Processing Pipeline

#### 1. Statistical Outlier Removal (SOR)
Eliminates sensor noise ("laser dust") to prevent isolated noise points from acting as "bridges" between distinct objects.
* **Empirical Choice:** `nb_neighbors=50`, `std_ratio=1.0`.

#### 2. Ground Plane Extraction (RANSAC)
Isolation of the track bed.
* **Empirical Threshold:** `distance_threshold=0.3m`.

#### 3. Spatial Clustering (Euclidean)
Groups remaining points into individual entities (Poles, Catenaries, Vegetation).
* **Empirical Tuning:** `ClusterTolerance=0.2m`, `MinSize=40`.

#### 4. Relative Height Analysis
Every point's height $H$ is calculated using the point-to-plane distance formula:
$$d = \frac{|ax + by + cz + d|}{\sqrt{a^2 + b^2 + c^2}}$$
This ensures that Pole **235** is detected with the same logic as Structures **112/172**, regardless of absolute altitude.

---

## 🛠️ Tech Stack
* **Language:** C++ 17 (Core) / Python (Prototyping)
* **Libraries:** [PCL (Point Cloud Library)](https://pointclouds.org/), [Open3D](http://www.open3d.org/)
* **Data Handling:** CloudCompare (for .laz to .pcd conversion & Global Shift)
* **Build System:** CMake

## 🌍 Impact & Use Cases
* **Railway Maintenance:** Automated clearance checks and vegetation risk management.
* **Digital Twins:** Rapid generation of classified 3D models for BIM integration.
* **Explainability:** 100% transparent classification logic for safety-critical infrastructure.