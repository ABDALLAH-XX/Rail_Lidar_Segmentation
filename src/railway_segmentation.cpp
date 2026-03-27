#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd_centered>" << std::endl;
        return -1;
    }

    // --- 1. CHARGEMENT ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);
    std::cout << "Points charges: " << cloud->size() << std::endl;

    // --- 2. VOXEL DOWN SAMPLE ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*cloud_down);

    // --- 3. SOR ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_down);
    sor.setMeanK(40);
    sor.setStddevMulThresh(3.0);
    sor.filter(*cloud_sor);

    // --- 4. RANSAC Sol ---
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_ground(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);
    seg.setInputCloud(cloud_sor);
    seg.segment(*inliers_ground, *coefficients);

    float a = coefficients->values[0], b = coefficients->values[1],
          c = coefficients->values[2], d = coefficients->values[3];

    // Extraction sol
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_sor);
    extract.setIndices(inliers_ground);
    extract.setNegative(false);
    extract.filter(*ground_cloud);

    // Extraction objets
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*object_cloud);

    // --- 5. FILTRAGE VERTICAL ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : object_cloud->points) {
        float dist = std::abs(a * pt.x + b * pt.y + c * pt.z + d);
        if (dist > 0.2)
            filtered_objects->points.push_back(pt);
    }
    filtered_objects->width = filtered_objects->points.size();
    filtered_objects->height = 1;

    // --- 6. CLUSTERING ---
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_objects);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(500000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_objects);
    ec.extract(cluster_indices);

    std::cout << "Nombre de clusters : " << cluster_indices.size() << std::endl;

    // --- 7. DIAGNOSTIC CLUSTERS CIBLES ---
    // Clusters identifiés visuellement comme poteaux potentiels
    std::vector<int> clusters_cibles = {16, 22, 442, 469, 527};
    std::cout << "\n=== DIAGNOSTIC CLUSTERS CIBLES ===" << std::endl;

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        // Vérifier si ce cluster est dans la liste cible
        bool est_cible = false;
        for (int cible : clusters_cibles)
            if ((int)i == cible) { est_cible = true; break; }
        if (!est_cible) continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster_indices[i].indices)
            cluster->points.push_back((*filtered_objects)[idx]);

        float mean_dist = 0;
        for (const auto& pt : cluster->points)
            mean_dist += std::abs(a*pt.x + b*pt.y + c*pt.z + d);
        mean_dist /= cluster->size();

        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;
        float length_xy = std::sqrt(dx*dx + dy*dy);

        std::cout << "C" << i
                  << " | pts=" << cluster->size()
                  << " | h=" << mean_dist
                  << " | dz=" << dz
                  << " | xy=" << length_xy << std::endl;
    }

    // --- 8. AFFICHAGE NUAGE COMPLET + POTEAUX ET CABLES ---
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("SNCF - Nuage Complets + Poteaux et Cables"));
    viewer->setBackgroundColor(0, 0, 0);

    // Sol en rouge
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        red_handler(ground_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(ground_cloud, red_handler, "ground");

    // Tous les clusters en gris avec labels
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster_indices[i].indices)
            cluster->points.push_back((*filtered_objects)[idx]);

        std::string name = "all_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            grey(cluster, 80, 80, 80);
        viewer->addPointCloud<pcl::PointXYZ>(cluster, grey, name);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);

        // Label au-dessus du cluster
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        pcl::PointXYZ center;
        center.x = (min_pt.x + max_pt.x) / 2;
        center.y = (min_pt.y + max_pt.y) / 2;
        center.z = max_pt.z + 0.5;
        viewer->addText3D(std::to_string(i), center, 0.3, 1.0, 1.0, 0.0,
                          "label_" + std::to_string(i));
    }

    // Détection poteaux et câbles
    int nb_poteaux = 0, nb_cables = 0;

    for (size_t i = 0; i < cluster_indices.size(); ++i) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster_indices[i].indices)
            cluster->points.push_back((*filtered_objects)[idx]);

        float mean_dist = 0;
        for (const auto& pt : cluster->points)
            mean_dist += std::abs(a*pt.x + b*pt.y + c*pt.z + d);
        mean_dist /= cluster->size();

        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;
        float length_xy = std::sqrt(dx*dx + dy*dy);

        // --- CRITERE POTEAU ---
        // Forme primaire : vertical, fin, hauteur suffisante
        if (dz > 3.0
            && length_xy < 1.5
            && cluster->size() >= 50
            && cluster->size() < 500
            && mean_dist > 1.0 && mean_dist < 15.0) {

            std::string name = "poteau_" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                white(cluster, 255, 255, 255);
            viewer->addPointCloud<pcl::PointXYZ>(cluster, white, name);
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
            nb_poteaux++;
            std::cout << "Poteau " << nb_poteaux
                      << " | C" << i
                      << " | pts=" << cluster->size()
                      << " | h=" << mean_dist
                      << " | dz=" << dz
                      << " | xy=" << length_xy << std::endl;
        }

        // --- CRITERE CABLE ---
        // Forme primaire : long horizontalement, peu vertical
        else if (mean_dist > 4.5 && mean_dist < 8.0
                 && length_xy > 4.0
                 && dz < 4.0
                 && cluster->size() < 5000) {

            std::string name = "cable_" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                cyan(cluster, 0, 255, 255);
            viewer->addPointCloud<pcl::PointXYZ>(cluster, cyan, name);
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
            nb_cables++;
            std::cout << "Cable " << nb_cables
                      << " | C" << i
                      << " | pts=" << cluster->size()
                      << " | h=" << mean_dist
                      << " | dz=" << dz
                      << " | xy=" << length_xy << std::endl;
        }
    }

    std::cout << "\nBilan : " << nb_poteaux << " poteau(x), "
              << nb_cables << " cable(s) detectes" << std::endl;

    viewer->resetCamera();
    while (!viewer->wasStopped()) { viewer->spin(); }

    return 0;
}