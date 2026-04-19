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
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
//#include <pcl/common/impl/angles.hpp>
#include <pcl/visualization/pcl_visualizer.h>

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

    pcl::PointCloud<PointSNCF>::Ptr cloud(new pcl::PointCloud<PointSNCF>);
    
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
        cloud->points.reserve(view->size());

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

                cloud->points.push_back(p);
            }
        }
    } else {
        // PCL Fallback
        pcl::io::loadPCDFile<PointSNCF>(filename, *cloud);
    }



    std::cout << "File loaded with NumberOfReturns filtering. Total points: " << cloud->size() << std::endl;    

    // 2. DOWNSAMPLING
    pcl::VoxelGrid<PointSNCF> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.12f, 0.12f, 0.12f);
    vg.filter(*cloud);

    std::cout << "Total points after downsampling: " << cloud->size() << std::endl;


    // 3. NORMAL ESTIMATION
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointSNCF, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointSNCF>::Ptr tree(new pcl::search::KdTree<PointSNCF>());
    ne.setSearchMethod(tree);
    //ne.setKSearch(20);
    ne.setRadiusSearch(0.5);
    ne.compute(*normals);



    // 4. NORMAL FILTERING
    pcl::PointCloud<PointSNCF>::Ptr poles_cloud(new pcl::PointCloud<PointSNCF>);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (std::abs((*normals)[i].normal_z) < 0.10f) { 
            poles_cloud->push_back((*cloud)[i]);
        }
    }

    // 5. CLUSTERING
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointSNCF> ec;
    ec.setClusterTolerance(0.2); 
    ec.setMinClusterSize(250);
    ec.setInputCloud(poles_cloud);
    ec.extract(clusters);

    

    // 6. VISUALIZATION & PROCESSING
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("SNCF Poles Detection"));
    viewer->setBackgroundColor(0, 0, 0);

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

        // Check the verticaly of RANSAC Z Axis (should be close to 1)
        float verticality = std::abs(coefficients->values[5]);

        if (h > 5.5f && h < 15.5f && verticality > 0.99f && dim_y < 0.85f) {
            count++;
            
            // Accurate coordinates
            float centerX = (min_p[0] + max_p[0]) / 2.0f;
            float centerY = (min_p[1] + max_p[1]) / 2.0f;

            // Visualization 
            for(auto& p : pole_detected->points) { p.r=255; p.g=255; p.b=0; }
            viewer->addPointCloud<PointSNCF>(pole_detected, "pole_" + std::to_string(count));


            // Display informations
            std::stringstream ss;
            ss << "POLE " << count << " (RANSAC)\n"
            << "Dim: " << std::fixed << std::setprecision(2) << dim_x << "x" << dim_y << "x" << h << "m";

            pcl::PointXYZ text_pos(centerX, centerY, max_p[2] + 0.8f);
            viewer->addText3D(ss.str(), text_pos, 0.25, 1.0, 1.0, 1.0, "info_" + std::to_string(count));
            
            std::cout << "--- Pole " << count << " detected---" << std::endl;
            std::cout << "Dimensions: " << dim_x << "m x " << dim_y << "m x " << h << "m" << std::endl;
        }
    }

    // --- END  ---
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Running time : " << duration.count() / 1000.0 << " s" << std::endl;

    viewer->addPointCloud<PointSNCF>(cloud, "background");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "background");

    viewer->spin();
    return 0;
}