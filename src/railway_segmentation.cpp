#define PCL_NO_PRECOMPILE
#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono> 
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

// --- Custom datapoint to visualize every fields of the .laz file ---
struct EIGEN_ALIGN16 PointSNCF {
   PCL_ADD_POINT4D; // Provide x, y, z and computation functions (getVector4fMap)
   PCL_ADD_RGB; // Provide r, g, b and computations functions (getRGBVector4fMap)
   
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
   uint8_t padding8[12]; 
   
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
    std::string filename = argv[1];

    // --- START  ---
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<PointSNCF>::Ptr cloud_raw(new pcl::PointCloud<PointSNCF>);
    
    // 1. FILE CONVERSION AND NUMBEROFRETURNS FILTERING
    
    if (filename.substr(filename.find_last_of(".") + 1) == "laz" || 
        filename.substr(filename.find_last_of(".") + 1) == "las") {
        
        std::cout << "Reading of the .LAZ file via PDAL..." << std::endl;
        
        
        pdal::Option las_opt("filename", filename);
        pdal::Options las_opts;
        las_opts.add(las_opt);

        pdal::LasReader reader;
        reader.setOptions(las_opts);

        pdal::PointTable table;
        reader.prepare(table);
        pdal::PointViewSet viewSet = reader.execute(table);
        pdal::PointViewPtr view = *viewSet.begin();

        // Immediate Memory Allocation (Crucial for performance)
        cloud_raw->points.reserve(view->size());

        double offsetX = 0, offsetY = 0, offsetZ = 0;
        bool firstPoint = true;
        
        for (pdal::PointId id = 0; id < view->size(); ++id) {
            float numReturns = static_cast<float>(view->getFieldAs<uint16_t>(pdal::Dimension::Id::NumberOfReturns, id));
            if (numReturns <=1.0){
                PointSNCF p;
                double rawX = view->getFieldAs<double>(pdal::Dimension::Id::X, id);
                double rawY = view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
                double rawZ = view->getFieldAs<double>(pdal::Dimension::Id::Z, id);

                // Set the first as origin
                if (firstPoint) {
                    offsetX = rawX;
                    offsetY = rawY;
                    offsetZ = rawZ;
                    firstPoint = false;
                    //std::cout << "Global Shift applied: " << std::fixed << offsetX << ", " << offsetY << std::endl;
                }

                // Coordinates (Apply Global Shift to maintain float precision)
                p.x = static_cast<float>(rawX - offsetX);
                p.y = static_cast<float>(rawY - offsetY);
                p.z = static_cast<float>(rawZ - offsetZ);
                
                // Intensity and Return information
                p.Intensity = view->getFieldAs<float>(pdal::Dimension::Id::Intensity, id);
                p.NumberOfReturns = view->getFieldAs<float>(pdal::Dimension::Id::NumberOfReturns, id);
                p.ReturnNumber = view->getFieldAs<float>(pdal::Dimension::Id::ReturnNumber, id);
                
                // Additional LAS metadata
                p.GpsTime = view->getFieldAs<float>(pdal::Dimension::Id::GpsTime, id);
                p.Classification = view->getFieldAs<float>(pdal::Dimension::Id::Classification, id);
                p.PointSourceId = view->getFieldAs<float>(pdal::Dimension::Id::PointSourceId, id);
                
                // RGB Color (Normalize 16-bit LAS color to 8-bit PCL format)
                if (view->hasDim(pdal::Dimension::Id::Red)) {
                    // Read as uint16 to prevent overflow exceptions from PDAL
                    uint16_t r16 = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Red, id);
                    uint16_t g16 = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Green, id);
                    uint16_t b16 = view->getFieldAs<uint16_t>(pdal::Dimension::Id::Blue, id);

                    // 8-bit conversion for PCL
                    p.r = static_cast<uint8_t>(r16 / 256);
                    p.g = static_cast<uint8_t>(g16 / 256);
                    p.b = static_cast<uint8_t>(b16 / 256);
                } else {
                    p.r = p.g = p.b = 200; 
                }

                cloud_raw->points.push_back(p);
            }
        }
    } else {
        // PCL Fallback
        pcl::io::loadPCDFile<PointSNCF>(filename, *cloud_raw);
    }



    std::cout << "File loaded with NumberOfReturns filtering. Total points: " << cloud_raw->size() << std::endl;    

    // 2. DOWNSAMPLING
    pcl::PointCloud<PointSNCF>::Ptr cloud(new pcl::PointCloud<PointSNCF>);
    pcl::VoxelGrid<PointSNCF> vg;
    vg.setInputCloud(cloud_raw);
    vg.setLeafSize(0.08f, 0.08f, 0.08f);
    vg.filter(*cloud);

    std::cout << "Total points after downsampling: " << cloud->size() << std::endl;


    // 3. NORMAL ESTIMATION
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointSNCF, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointSNCF>::Ptr tree(new pcl::search::KdTree<PointSNCF>());
    ne.setSearchMethod(tree);
    //ne.setKSearch(20);
    ne.setRadiusSearch(0.25);
    ne.compute(*normals);



    // 4. NORMAL FILTERING
    pcl::PointCloud<PointSNCF>::Ptr poles_cloud(new pcl::PointCloud<PointSNCF>);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (std::abs((*normals)[i].normal_z) < 0.2f) { 
            poles_cloud->push_back((*cloud)[i]);
        }
    }


    // 5. CLUSTERING
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointSNCF> ec;
    ec.setClusterTolerance(0.4); 
    ec.setMinClusterSize(50);
    ec.setInputCloud(poles_cloud);
    ec.extract(clusters);

    

    // 6. VISUALIZATION & PROCESSING
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("SNCF Poles Detection"));
    viewer->setBackgroundColor(0, 0, 0);


    // On crée le KD-Tree sur le nuage original pour des recherches ultra-rapides
    pcl::search::KdTree<PointSNCF>::Ptr cloud_kdtree(new pcl::search::KdTree<PointSNCF>);
    cloud_kdtree->setInputCloud(cloud_raw); // 'cloud' est ton nuage après downsampling mais avant filtrage des normales
    // For loop to detect every from the cluster
    int count = 0;
    for (const auto& indices : clusters) {
        // 1. Isolate the raw cloud
        pcl::PointCloud<PointSNCF>::Ptr cluster_cloud(new pcl::PointCloud<PointSNCF>);
        pcl::copyPointCloud(*poles_cloud, indices, *cluster_cloud);


        // 2. Appy RANSAC to extract vertical information
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointSNCF> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.5); // 50cm threshold around the axis
        seg.setMaxIterations(1000);
        seg.setInputCloud(cluster_cloud);
        seg.segment(*inliers, *coefficients);

        // 3. POLES EXTRACTION
        pcl::PointCloud<PointSNCF>::Ptr pole_detected(new pcl::PointCloud<PointSNCF>);
        pcl::ExtractIndices<PointSNCF> extract;
        extract.setInputCloud(cluster_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*pole_detected);

        // 4. Compute the poles dimensions
        Eigen::Vector4f min_p, max_p;
        pcl::getMinMax3D(*pole_detected, min_p, max_p);
        
        
        float dim_x = max_p[0] - min_p[0];
        float dim_y = max_p[1] - min_p[1];
        float h = max_p[2] - min_p[2];
        float density = pole_detected->size() / h;

        // 5. Compute the poles Radial deviation

        float sum_r = 0, sum_r_sq = 0;
        int n = pole_detected->size();

        // Computes mean center of the cluster
        float center_x = 0, center_y = 0;
        for (const auto& pt : pole_detected->points) {
            center_x += pt.x;
            center_y += pt.y;
        }
        center_x /= n;
        center_y /= n;

        // Computes radials distances and standard deviation
        std::vector<float> distances;
        for (const auto& pt : pole_detected->points) {
            float r = std::sqrt(std::pow(pt.x - center_x, 2) + std::pow(pt.y - center_y, 2));
            distances.push_back(r);
            sum_r += r;
        }
        float mean_r = sum_r / n;

        for (float r : distances) {
            sum_r_sq += std::pow(r - mean_r, 2);
        }
        float sigma_radial = std::sqrt(sum_r_sq / n);

        
        // Check the verticaly of RANSAC Z Axis (should be close to 1)
        float verticality = std::abs(coefficients->values[5]);
        
        bool is_tall_enough    = (h > 5.5f && h < 16.0f);
        bool is_vertical       = (verticality > 0.8f);
        bool is_thin_enough    = (dim_x > 0.5f && dim_x < 1.2f) && (dim_y > 0.5f && dim_y < 1.2f);
        bool is_cylindrical    = (sigma_radial < 0.10f);
        bool is_dense_enough   = (density > 80.0f);


        //verticality > 0.8f && dim_x <1.5f && dim_y < 1.5f && sigma_radial < 0.26f && density > 80.0f
        if (is_tall_enough && is_vertical && is_thin_enough && is_cylindrical && is_dense_enough) {
            count++;
            
            // Accurate coordinates
            float pole_centerX = (min_p[0] + max_p[0]) / 2.0f;
            float pole_centerY = (min_p[1] + max_p[1]) / 2.0f;
            float pole_topZ = max_p[2];

            // Visualization 
            for(auto& p : pole_detected->points) { p.r=255; p.g=255; p.b=0; }
            viewer->addPointCloud<PointSNCF>(pole_detected, "pole_" + std::to_string(count));


            // Display informations
            std::stringstream ss;
            ss << "POLE " << count << " (RANSAC)\n"
            << "Dim: " << std::fixed << std::setprecision(2) << dim_x << "x" << dim_y << "x" << h << "m";

            pcl::PointXYZ text_pos(pole_centerX, pole_centerY, pole_topZ + 0.8f);
            viewer->addText3D(ss.str(), text_pos, 0.25, 1.0, 1.0, 1.0, "info_" + std::to_string(count));
            
            std::cout << "--- Pole " << count << " detected---" << std::endl;
            std::cout << "Dimensions: " << dim_x << "m x " << dim_y << "m x " << h << "m x" << " Total points: " << pole_detected->size() << " x Density: "
               << density << " x sigma_radial " << sigma_radial << std::endl;

            // Setting search point (top of the pole)
            PointSNCF searchPoint;
            searchPoint.x = pole_centerX;
            searchPoint.y = pole_centerY;
            searchPoint.z = pole_topZ;

            // Cantilever search on the orignal cloud
            std::vector<int> indices;
            std::vector<float> distances;
            float radius = 6.0f;

            
            // A radius is performed around the poles detected on the original cloud in order to find cantilevers
            if (cloud_kdtree->radiusSearch(searchPoint, radius, indices, distances) > 0) {
                pcl::PointCloud<PointSNCF>::Ptr cantilever_raw(new pcl::PointCloud<PointSNCF>);

                // Grab the original points 
                // A single point is treated (for loop)
                for (int idx : indices) {
                    
                    // Acces to the normals of the original cloud
                    //float nz = std::abs((*normals)[idx].normal_z);
                    // float normal_z_val = std::abs(normals->points[idx].normal_z);
                    PointSNCF p = cloud_raw->points[idx];

                    // À l'intérieur de ta boucle for (int idx : indices_radius)
                    float delta_x = p.x - pole_centerX;
                    float delta_y = p.y - pole_centerY;
                    //float dz = p.z;
                    float dist_xy = std::sqrt(delta_x*delta_x + delta_y*delta_y); // Distance horizontale au poteau
                    //std::cout << "distance xy " << dist_xy << std::endl;
                    float z_min = pole_topZ - 6.0f;
                    float z_max = pole_topZ + 1.0f;


                    bool condition_dist = (dist_xy > 0.35f && dist_xy < 10.0f);
                    bool condition_y = (std::abs(delta_y) < 1.5f);
                    bool condition_z = (p.z > z_min && p.z < z_max);

                    // Keep the high points (between the top and 2.5m under)
                    if (condition_dist && condition_y && condition_z) {
                        p.r = 0; p.g = 0; p.b = 255;
                        /*switch(count) {
                            case 1:
                                p.r = 0; p.g = 0; p.b = 255;
                                break;
                            case 2:
                                p.r = 255; p.g = 0; p.b = 255;
                                break;
                            case 3:
                                p.r = 0; p.g = 153; p.b = 0;
                                break;
                        }*/
                        cantilever_raw->push_back(p);
                        
                    }
                    
                    
                    
                }
                // The Cantilever raw cloud is completed which means it can be processed

                // --- ÉTAPE 2 : ON LANCE LE RANSAC (Une seule fois pour tout le nuage) ---
                /*if (!cantilever_raw->empty()) { // On vérifie qu'on a assez de points
                    pcl::ModelCoefficients::Ptr coeff_line(new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);
                    pcl::SACSegmentation<PointSNCF> seg_line;
                    
                    seg_line.setOptimizeCoefficients(true);
                    //seg_line.setModelType(pcl::SACMODEL_LINE);
                    seg_line.setMethodType(pcl::SAC_RANSAC);
                    seg_line.setModelType(pcl::SACMODEL_LINE);
                    seg_line.setAxis(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
                    seg_line.setEpsAngle(15.0f * M_PI / 180.0f);
                    seg_line.setDistanceThreshold(0.5f); // CORRIGÉ : 0.01 était beaucoup trop serré (1cm !)
                    seg_line.setInputCloud(cantilever_raw);
                    seg_line.segment(*inliers_line, *coeff_line);

                    std::cout << "Direction line = "
                        << coeff_line->values[3] << ", "
                        << coeff_line->values[4] << ", "
                        << coeff_line->values[5] << std::endl;

                    pcl::PointCloud<PointSNCF>::Ptr cantilever_detected(new pcl::PointCloud<PointSNCF>);
                    pcl::ExtractIndices<PointSNCF> extract_cantilever;
                    extract_cantilever.setInputCloud(cantilever_raw);
                    extract_cantilever.setIndices(inliers_line);
                    extract_cantilever.setNegative(false);
                    extract_cantilever.filter(*cantilever_detected);

                    std::cout << "Size of the cantilever cloud :" << cantilever_detected->size() << std::endl;
                    std::cout << "Number of inliers :" << inliers_line->indices.size() << std::endl;

                    if (!inliers_line->indices.empty() && !cantilever_detected->empty()) {
                        float cant_centerX = 0.0f, cant_centerY = 0.0f;
                        for (const auto& pt : cantilever_detected->points) {
                            cant_centerX += pt.x;
                            cant_centerY += pt.y;
                        }
                        cant_centerX /= cantilever_detected->size();
                        cant_centerY /= cantilever_detected->size();

                        bool long_enough = cantilever_detected->size() > 0;
                        bool far_from_pole = std::abs(cant_centerX - pole_centerX) > 2.0f;
                        bool anchored_near_pole = std::abs(cant_centerY - pole_centerY) < 10.0f;
                        bool acceptable_shape = long_enough && far_from_pole && anchored_near_pole;

                        if (acceptable_shape) {
                            for (auto& pt : cantilever_detected->points) {
                                pt.r = 0; pt.g = 0; pt.b = 255;
                            }

                            std::string id = "console_" + std::to_string(count);
                            viewer->removePointCloud(id);
                            viewer->addPointCloud<PointSNCF>(cantilever_detected, id);

                            std::cout << "Console " << count << " détectée : "
                                    << cantilever_detected->size() << " points." << std::endl;
                        }
                    }
                }*/

                std::vector<pcl::PointIndices> cantilever_clusters;
                pcl::search::KdTree<PointSNCF>::Ptr cantilever_tree(new pcl::search::KdTree<PointSNCF>);
                cantilever_tree->setInputCloud(cantilever_raw);

                pcl::EuclideanClusterExtraction<PointSNCF> ec_cant;
                float tolerance = 0.2;
                ec_cant.setClusterTolerance(tolerance);
                ec_cant.setMinClusterSize(50);
                ec_cant.setMaxClusterSize(2000);
                ec_cant.setSearchMethod(cantilever_tree);
                ec_cant.setInputCloud(cantilever_raw);
                ec_cant.extract(cantilever_clusters);
                
                std::cout << "tolerance :" << tolerance << std::endl;

                /*for (size_t i = 0; i < cantilever_clusters.size(); ++i) {
                    pcl::PointCloud<PointSNCF>::Ptr cantilever_cluster(new pcl::PointCloud<PointSNCF>);
                    pcl::copyPointCloud(*cantilever_raw, cantilever_clusters[i], *cantilever_cluster);

                    std::cout << "Cantilever cluster " << i + 1
                            << " : " << cantilever_cluster->size()
                            << " points" << std::endl;

                    std::string id = "cantilever_" + std::to_string(count) + "_" + std::to_string(i);
                    viewer->removePointCloud(id);
                    viewer->addPointCloud<PointSNCF>(cantilever_cluster, id);
                }*/

                for (size_t i = 0; i < cantilever_clusters.size(); ++i) {
                    pcl::PointCloud<PointSNCF>::Ptr cantilever_cluster(new pcl::PointCloud<PointSNCF>);
                    pcl::copyPointCloud(*cantilever_raw, cantilever_clusters[i], *cantilever_cluster);

                    if (cantilever_cluster->size() < 30)
                        continue;

                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*cantilever_cluster, centroid);

                    pcl::PCA<PointSNCF> pca;
                    pca.setInputCloud(cantilever_cluster);

                    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                    Eigen::Vector3f main_axis = eigen_vectors.col(0).normalized();
                    float x_axis = main_axis.x();
                    float y_axis = main_axis.y();


                    float align_x = std::abs(main_axis.dot(Eigen::Vector3f::UnitX()));
                    //float vertical_component = std::abs(main_axis.z());

                    Eigen::Vector4f min_p, max_p;
                    pcl::getMinMax3D(*cantilever_cluster, min_p, max_p);

                    float dim_x = max_p[0] - min_p[0];
                    float dim_y = max_p[1] - min_p[1];
                    float dim_z = max_p[2] - min_p[2];

                    float dist_to_pole = std::sqrt(
                        std::pow(centroid[0] - pole_centerX, 2) +
                        std::pow(centroid[1] - pole_centerY, 2)
                    );

                    //bool enough_points   = cantilever_cluster->size() > 50;
                    //bool aligned_with_x  = align_x > 0.35f;
                    //bool not_vertical    = vertical_component < 0.8f;
                    //bool close_to_pole   = dist_to_pole < 2.0f;
                    //bool elongated_in_x  = dim_x > 0.5f;
                    //bool narrow_in_y     = dim_y < 2.5f;
                    bool x_oriented      = (std::abs(x_axis) > std::abs(y_axis));
                    

                    bool keep_cluster = //enough_points &&
                                        //aligned_with_x &&
                                        //not_vertical &&
                                        //close_to_pole &&
                                        //elongated_in_x &&
                                        //narrow_in_y &&
                                        x_oriented;

                    
                    if (keep_cluster) {
                        uint8_t r = 0, g = 0, b = 255;

                        /*switch (count) {
                            case 1:
                                r = 0;   g = 0;   b = 255;   // bleu
                                break;
                            case 2:
                                r = 255; g = 0;   b = 255;   // rose
                                break;
                            default:
                                r = 0;   g = 255; b = 255;   // cyan
                                break;
                        }*/

                        for (auto& pt : cantilever_cluster->points) {
                            pt.r = r;
                            pt.g = g;
                            pt.b = b;
                        }

                        std::string id = "cantilever_cluster_" + std::to_string(count) + "_" + std::to_string(i);
                        viewer->removePointCloud(id);
                        viewer->addPointCloud<PointSNCF>(cantilever_cluster, id);

                        std::cout << "Cluster " << i + 1
                            << " | pts=" << cantilever_cluster->size()
                            << " | axis=(" << main_axis.x() << ", "
                                            << main_axis.y() << ", "
                                            << main_axis.z() << ")"
                            << " | align_x=" << align_x
                            << " | dim=(" << dim_x << ", "
                                            << dim_y << ", "
                                            << dim_z << ")"
                            << " | dist_pole=" << dist_to_pole
                            << std::endl;


                        std::cout << " -> kept as X-aligned cantilever" << std::endl;
                    }
                }
                
                // Cantilver display
                if (!cantilever_raw->empty()) {
                    viewer->addPointCloud<PointSNCF>(cantilever_raw, "cantilever_" + std::to_string(count));
                    //std::cout << "Cantilever detected. Total points : " << cantilever_raw->size() << std::endl;
                }
            }



            
        }
    }

    // --- END  ---
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Running time : " << duration.count() / 1000.0 << " s" << std::endl;

    viewer->addPointCloud<PointSNCF>(cloud_raw, "background");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "background");

    //viewer->addPointCloudNormals<PointSNCF, pcl::Normal>(cloud_raw, normals_raw, 40, 0.2, "normals");


    viewer->spin();
    return 0;
}