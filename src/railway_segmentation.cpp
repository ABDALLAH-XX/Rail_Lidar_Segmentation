#define PCL_NO_PRECOMPILE
#include <iostream>
#include <vector>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h> // AJOUTÉ POUR LE PCA
#include <pcl/filters/passthrough.h> // AJOUTÉ
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/common/impl/angles.hpp>
#include <pcl/visualization/pcl_visualizer.h>

// --- Creation d'un type de point personnalisé pour visualiser les différents champs du nuage de points ---
struct EIGEN_ALIGN16 PointSNCF {
   PCL_ADD_POINT4D; // Fournit x, y, z et les fonctions de calcul (getVector4fMap)
   PCL_ADD_RGB; // Fournit r, g, b et les fonctions de calcul (getRGBVector4fMap)
   
   float Classification;
   uint8_t padding1[12];
   float Intensity;
   uint8_t padding2[12];
   float GpsTime;
   uint8_t padding3[12];
   float ReturnNumber;
   uint8_t padding4[12];
   float NumberOfReturns;
   uint8_t padding5[12];
   float UserData;
   uint8_t padding6[12];
   float PointSourceId;
   uint8_t padding7[12];
   uint8_t padding8[12]; // Correspond au dernier SIZE 12 du header
   
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// L'enregistrement doit mapper tes noms de champs PCD aux variables de la struct
POINT_CLOUD_REGISTER_POINT_STRUCT(PointSNCF,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, Classification, Classification)
    (float, Intensity, Intensity)
    (float, GpsTime, GpsTime)
    (float, ReturnNumber, ReturnNumber)
    (float, NumberOfReturns, NumberOfReturns)
    (float, UserData, UserData)
    (float, PointSourceId, PointSourceId)
    
)


int main(int argc, char** argv) {
    if (argc < 2) return -1;
    pcl::PointCloud<PointSNCF>::Ptr cloud(new pcl::PointCloud<PointSNCF>);
    pcl::io::loadPCDFile<PointSNCF>(argv[1], *cloud);


    // 1. DOWNSAMPLING
    pcl::PointCloud<PointSNCF>::Ptr cloud_down(new pcl::PointCloud<PointSNCF>);
    pcl::VoxelGrid<PointSNCF> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.12f, 0.12f, 0.12f);
    vg.filter(*cloud_down);

    // 2. FILTRAGE STATISTIQUE
    pcl::PointCloud<PointSNCF>::Ptr cloud_filtered(new pcl::PointCloud<PointSNCF>);
    pcl::StatisticalOutlierRemoval<PointSNCF> sor;
    sor.setInputCloud(cloud_down);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // 4. CALCUL DES NORMALES
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointSNCF, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    pcl::search::KdTree<PointSNCF>::Ptr tree(new pcl::search::KdTree<PointSNCF>());
    ne.setSearchMethod(tree);
    ne.setKSearch(20); 
    ne.compute(*normals);

    // 4. FILTRAGE PAR NORMALES (Métal vertical)
    pcl::PointCloud<PointSNCF>::Ptr cloud_poteaux(new pcl::PointCloud<PointSNCF>);
    for (size_t i = 0; i < cloud_filtered->size(); ++i) {
        if (std::abs((*normals)[i].normal_z) < 0.15f) { 
            cloud_poteaux->push_back((*cloud_filtered)[i]);
        }
    }

    // 5. CLUSTERING
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointSNCF> ec;
    ec.setClusterTolerance(0.4); 
    ec.setMinClusterSize(100);
    ec.setInputCloud(cloud_poteaux);
    ec.extract(clusters);

    // 6. VISUALISATION ET CALCULS
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Detection SNCF"));
    viewer->setBackgroundColor(0, 0, 0);

    int count = 0;
    for (const auto& indices : clusters) {
        Eigen::Vector4f min_p, max_p;
        pcl::getMinMax3D(*cloud_poteaux, indices, min_p, max_p);
        
        float dim_x = max_p[0] - min_p[0];
        float dim_y = max_p[1] - min_p[1];
        float h = max_p[2] - min_p[2];

        // Filtre de hauteur pour les poteaux
        if (h > 4.0f) {
            count++;
            
            // Coordonnées du centre de la base
            float centerX = (min_p[0] + max_p[0]) / 2.0f;
            float centerY = (min_p[1] + max_p[1]) / 2.0f;
            float baseZ = min_p[2];

            // Coloration du cluster
            pcl::PointCloud<PointSNCF>::Ptr cluster_cloud(new pcl::PointCloud<PointSNCF>);
            pcl::copyPointCloud(*cloud_poteaux, indices, *cluster_cloud);
            for(auto& p : cluster_cloud->points) { p.r=255; p.g=255; p.b=0; }
            
            viewer->addPointCloud<PointSNCF>(cluster_cloud, "poteau_" + std::to_string(count));

            // Préparation du texte (ID + Dimensions + Coordonnées)
            std::stringstream ss;
            ss << "POTEAU " << count << "\n"
               << "Dim: " << std::fixed << std::setprecision(2) << dim_x << "x" << dim_y << "x" << h << "m\n"
               << "Pos: [" << centerX << ", " << centerY << ", " << baseZ << "]";

            // Affichage du texte au sommet
            pcl::PointXYZ text_pos(centerX, centerY, max_p[2] + 0.8f);
            viewer->addText3D(ss.str(), text_pos, 0.25, 1.0, 1.0, 1.0, "info_" + std::to_string(count));
            
            // Log console
            std::cout << "--- Poteau " << count << " ---" << std::endl;
            std::cout << "Dimensions: " << dim_x << "m x " << dim_y << "m x " << h << "m" << std::endl;
            std::cout << "Position Base (X,Y,Z): " << centerX << ", " << centerY << ", " << baseZ << std::endl;
        }
    }

    viewer->addPointCloud<PointSNCF>(cloud_filtered, "background");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "background");

    viewer->spin();
    return 0;
}