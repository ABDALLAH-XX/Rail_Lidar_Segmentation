#define PCL_NO_PRECOMPILE
#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
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

    std::cout << "Fichier charge. Nombre de points: " << cloud->size() << std::endl;

    // --- CHRONOMETRAGE DE L'ALGORITHME ---
    auto start = std::chrono::high_resolution_clock::now();

    // 1. FILTRAGE PAR NUMBERS OF RETURNS
    pcl::PointCloud<PointSNCF>::Ptr cloud_numberOfReturns(new pcl::PointCloud<PointSNCF>);
    for (const auto& point : cloud->points) {
        if ((point.NumberOfReturns) <= 1.0f) {
            cloud_numberOfReturns->push_back(point);
        }
    }
    cloud_numberOfReturns->width = cloud_numberOfReturns->size();
    cloud_numberOfReturns->height = 1;

    std::cout << "Nombre de points après filtrage par NumberOfReturns: " << cloud_numberOfReturns->size() << std::endl;

    // 2. DOWNSAMPLING
    pcl::VoxelGrid<PointSNCF> vg;
    vg.setInputCloud(cloud_numberOfReturns);
    vg.setLeafSize(0.12f, 0.12f, 0.12f);
    vg.filter(*cloud_numberOfReturns);

    std::cout << "Nombre de points après Downsampling: " << cloud_numberOfReturns->size() << std::endl;

    
    // 3. CALCUL DES NORMALES
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointSNCF, pcl::Normal> ne;
    ne.setInputCloud(cloud_numberOfReturns);
    pcl::search::KdTree<PointSNCF>::Ptr tree(new pcl::search::KdTree<PointSNCF>());
    ne.setSearchMethod(tree);
    //ne.setKSearch(15);
    ne.setRadiusSearch(0.5);
    ne.compute(*normals);



    // 4. FILTRAGE PAR NORMALES (Métal vertical)
    pcl::PointCloud<PointSNCF>::Ptr cloud_poteaux(new pcl::PointCloud<PointSNCF>);
    for (size_t i = 0; i < cloud_numberOfReturns->size(); ++i) {
        if (std::abs((*normals)[i].normal_z) < 0.10f) { 
            cloud_poteaux->push_back((*cloud_numberOfReturns)[i]);
        }
    }

    // 5. CLUSTERING
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointSNCF> ec;
    ec.setClusterTolerance(0.2); 
    ec.setMinClusterSize(250);
    ec.setInputCloud(cloud_poteaux);
    ec.extract(clusters);

    

    // 6. VISUALISATION ET CALCULS
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Detection SNCF"));
    viewer->setBackgroundColor(0, 0, 0);

    // On crée une boucle sur les clusters pour déterminer les poteaux
    int count = 0;
    for (const auto& indices : clusters) {
        // 1. Isoler le cluster brut
        pcl::PointCloud<PointSNCF>::Ptr cluster_cloud(new pcl::PointCloud<PointSNCF>);
        pcl::copyPointCloud(*cloud_poteaux, indices, *cluster_cloud);

        // 2. Appliquer RANSAC pour extraire l'axe vertical du poteau
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointSNCF> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.5); // Seuil de 50cm autour de l'axe
        seg.setMaxIterations(1000);
        seg.setInputCloud(cluster_cloud);
        seg.segment(*inliers, *coefficients);
 

        // 3. EXTRACTION : On crée un nuage qui ne contient QUE les points du poteau (sans feuilles)
        pcl::PointCloud<PointSNCF>::Ptr poteau_propre(new pcl::PointCloud<PointSNCF>);
        pcl::ExtractIndices<PointSNCF> extract;
        extract.setInputCloud(cluster_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*poteau_propre);

        // 4. Calcul des dimensions sur le nuage PROPRE (RANSAC-filtered)
        Eigen::Vector4f min_p, max_p;
        pcl::getMinMax3D(*poteau_propre, min_p, max_p);
        
        float dim_x = max_p[0] - min_p[0];
        float dim_y = max_p[1] - min_p[1];
        float h = max_p[2] - min_p[2];

        // Vérification de la verticalité de l'axe RANSAC (direction Z doit être proche de 1)
        float verticalite = std::abs(coefficients->values[5]);

        if (h > 5.5f && h < 15.5f && verticalite > 0.99f && dim_y < 0.85f) {
            count++;
            
            // Coordonnées précises
            float centerX = (min_p[0] + max_p[0]) / 2.0f;
            float centerY = (min_p[1] + max_p[1]) / 2.0f;
            //float baseZ = min_p[2];

            // Visualisation : Poteau en Jaune
            for(auto& p : poteau_propre->points) { p.r=255; p.g=255; p.b=0; }
            viewer->addPointCloud<PointSNCF>(poteau_propre, "poteau_" + std::to_string(count));


            // Affichage des infos
            std::stringstream ss;
            ss << "POLE " << count << " (RANSAC)\n"
            << "Dim: " << std::fixed << std::setprecision(2) << dim_x << "x" << dim_y << "x" << h << "m";

            pcl::PointXYZ text_pos(centerX, centerY, max_p[2] + 0.8f);
            viewer->addText3D(ss.str(), text_pos, 0.25, 1.0, 1.0, 1.0, "info_" + std::to_string(count));
            
            std::cout << "--- Poteau " << count << " détecté au milieu de la végétation ---" << std::endl;
            std::cout << "Dimensions réelles: " << dim_x << "m x " << dim_y << "m x " << h << "m" << std::endl;
            std::cout << "Verticalité (Z-axis): " << verticalite << std::endl;
        }
    }

    // --- FIN DU CHRONO  ---
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Algorithme execute en : " << duration.count() / 1000.0 << " s" << std::endl;

    viewer->addPointCloud<PointSNCF>(cloud_numberOfReturns, "background");
    //viewer->addPointCloudNormals<PointSNCF, pcl::Normal>(cloud_numberOfReturns, normals, 10, 0.4, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "background");

    viewer->spin();
    return 0;
}