#include <iostream>
#include <string>
#include <argh.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include "src/base/log.h"
#include "src/geometry/io/xyz_io.h"
#include "src/geometry/point_cloud/pca_estimate_normals.h"
#include "src/geometry/point_cloud/supervoxel_segmentation.h"
#include "src/geometry/util/distance_3d.h"
#include "src/util/tree/kd_tree.h"

#include "vccs/vccs_knn_supervoxel.h"
#include "vccs/vccs_supervoxel.h"

#include "Metrics.h"


int main(int argc, char *argv[]) {
    LOG_ON(INFO);

    // Parse command line arguments
    auto cmdl = argh::parser(argc, argv);

    // String arguments
    std::string in_file_name = "../test_data/test.xyz";
    std::string out_file_name = "../test_data/segmented.xyz";
    std::string method = "bpss"; // bpss, vccs, vccs_knn
    cmdl({ "-i", "--in" }) >> in_file_name;
    cmdl({ "-o", "--out" }) >> out_file_name;
    cmdl({ "-m", "--method" }) >> method;

    // Boolean flags
    bool in_estimate_normals = cmdl[{ "-n", "--estimate_normals" }];
    
    // Numeric arguments
    int k_neighbors = 15;
    float resolution = 1.0f;
    float vccs_voxel_resolution = 0.3f;
    long n_supervoxels = 10000;
    int n_threads = 0;
    cmdl({ "-k", "--k_neighbors" }) >> k_neighbors;
    cmdl({ "-vccs_vr", "--vccs_voxel_resolution" }) >> vccs_voxel_resolution;
    cmdl({ "-s", "--n_supervoxels" }) >> n_supervoxels;
    cmdl({ "-t", "--n_threads" }) >> n_threads;
    bool use_resolution = false;
    if(cmdl({ "-r", "--resolution"}) >> resolution) {
        std::cout << "RE";
        use_resolution = true;
    }

    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Print command line arguments
    LOG(INFO) << "Command line arguments:";
    LOG(INFO) << "  Input file: " << in_file_name;
    LOG(INFO) << "  Output file: " << out_file_name;
    LOG(INFO) << "  Method: " << method;
    LOG(INFO) << "  Estimate normals: " << in_estimate_normals;
    LOG(INFO) << "  K-neighbors: " << k_neighbors;
    LOG(INFO) << "  Resolution: " << resolution;
    LOG(INFO) << "  VCCS voxel resolution: " << vccs_voxel_resolution;
    LOG(INFO) << "  Number of supervoxels: " << n_supervoxels;
    LOG(INFO) << "  Number of threads: " << n_threads;
    LOG(INFO) << "  Use resolution: " << use_resolution;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Read point cloud from file
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    const std::string file_ending = in_file_name.substr(in_file_name.find_last_of(".") + 1);
    if (file_ending == "xyz" || file_ending == "txt") {
        std::ifstream file(in_file_name);
        if (!file.is_open()) {
            LOG(ERROR) << "Couldn't open file " << in_file_name;
            return -1;
        }

        std::string line;
        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty() || line[0] == '#') continue;

            pcl::PointXYZRGBNormal point;

            std::string segment;
            std::vector<std::string> segments;
            std::stringstream line_as_string_stream(line);

            while (std::getline(line_as_string_stream, segment, ' ')) {
                segments.push_back(segment);
            }

            int count = segments.size();
            if (count >= 3) {
                point.x = std::stof(segments[0]);
                point.y = std::stof(segments[1]);
                point.z = std::stof(segments[2]);
            }
            if (count == 6) {
                point.r = static_cast<uint8_t>(std::stoi(segments[3]));
                point.g = static_cast<uint8_t>(std::stoi(segments[4]));
                point.b = static_cast<uint8_t>(std::stoi(segments[5]));
            }
            if (count == 9) {
                point.normal_x = std::stof(segments[3]);
                point.normal_y = std::stof(segments[4]);
                point.normal_z = std::stof(segments[5]);
                
                point.r = std::stoi(segments[6]);
                point.g = std::stoi(segments[7]);
                point.b = std::stoi(segments[8]);
            }
            
            cloud->points.push_back(point);
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

    } else if (pcl::io::load<pcl::PointXYZRGBNormal>(in_file_name, *cloud) == -1) {
        LOG(ERROR) << "Couldn't read file " << in_file_name;
        return -1;
    }

    int n_points = cloud->size();
    LOG(INFO) << "Loaded point cloud with " << n_points << " points";

    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Estimate normals
    if (in_estimate_normals) {
        LOG(INFO) << "Estimating normals...";
        pcl::NormalEstimationOMP<pcl::PointXYZRGBNormal, pcl::Normal> ne(n_threads);
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr normal_estimation_search_tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
        ne.setSearchMethod(normal_estimation_search_tree);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(k_neighbors);
        ne.compute(*normals);

        for (int i = 0; i < n_points; ++i) {
            cloud->points[i].normal_x = normals->points[i].normal_x;
            cloud->points[i].normal_y = normals->points[i].normal_y;
            cloud->points[i].normal_z = normals->points[i].normal_z;
        }
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Supervoxel segmentation
    LOG(INFO) << "Start supervoxel segmentation...";
    cl::Array<cl::PointWithNormal> oriented_points(n_points);
    for (int i = 0; i < n_points; ++i) {
        oriented_points[i].x = cloud->points[i].x;
        oriented_points[i].y = cloud->points[i].y;
        oriented_points[i].z = cloud->points[i].z;
        oriented_points[i].normal.x = cloud->points[i].normal_x;
        oriented_points[i].normal.y = cloud->points[i].normal_y;
        oriented_points[i].normal.z = cloud->points[i].normal_z;
    }

    // Labels
    cl::Array<int> labels;
    int n_out_supervoxels = 0;

    if (method == "bpss") {

        // Search neighbors
        LOG(INFO) << "Search neighbors...";
        cl::Array<cl::Array<int> > neighbors(n_points);
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr bpss_kd_tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
        bpss_kd_tree->setInputCloud(cloud);
        
        #pragma omp parallel num_threads(n_threads)
        {
            #pragma omp for
            for (int i = 0; i < n_points; ++i) {
                std::vector<int> pointIdxKNNSearch(k_neighbors);
                std::vector<float> pointNKNSquaredDistance(k_neighbors);
                bpss_kd_tree->nearestKSearch(cloud->points[i], k_neighbors, pointIdxKNNSearch, pointNKNSquaredDistance);
                for (int k = 0; k < k_neighbors; ++k) {
                    neighbors[i].push_back(pointIdxKNNSearch[k]);
                }
            }
        }

        // BPSS segmentation
        cl::Array<int> supervoxels;
        LOG(INFO) << "BPSS supervoxel computation";
        if (use_resolution) {
            LOG(INFO) << "Using resolution based supervoxel segmentation: " << resolution;
            VCCSMetric metric(resolution);
            cl::geometry::point_cloud::SupervoxelSegmentation(
                oriented_points, 
                neighbors, 
                static_cast<double>(resolution), 
                metric, 
                &supervoxels, 
                &labels
            );
            n_out_supervoxels = supervoxels.size();
        } else {
            LOG(INFO) << "Using fixed number of supervoxels: " << n_supervoxels;
            VCCSMetric metric(1.0);
            cl::geometry::point_cloud::SupervoxelSegmentation(
                oriented_points, 
                neighbors, 
                static_cast<int>(n_supervoxels), 
                metric, 
                &supervoxels, 
                &labels
            );
            n_out_supervoxels = supervoxels.size();
        }

    } else if (method == "vccs") {
        LOG(INFO) << "VCCS supervoxel computation";

        cl::Array<cl::RPoint3D> points;
        for (int i = 0; i < n_points; ++i) {
            points.push_back(cl::RPoint3D(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
        }

        VCCSSupervoxel vccs(points.begin(), points.end(), vccs_voxel_resolution, resolution);
        cl::Array<VCCSSupervoxel::Supervoxel> vccs_supervoxels;
        vccs.Segment(&labels, &vccs_supervoxels);
        n_out_supervoxels = vccs_supervoxels.size();

    } else if (method == "vccs_knn") {
        LOG(INFO) << "VCCS KNN supervoxel computation";

        cl::Array<cl::RPoint3D> points;
        for (int i = 0; i < n_points; ++i) {
            points.push_back(cl::RPoint3D(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
        }

        cl::KDTree<cl::RPoint3D> kdtree;
        kdtree.SwapPoints(&points);

        VCCSKNNSupervoxel vccs_knn(kdtree, resolution, k_neighbors);
        cl::Array<VCCSKNNSupervoxel::Supervoxel> vccs_knn_supervoxels;
        vccs_knn.Segment(&labels, &vccs_knn_supervoxels);
        n_out_supervoxels = vccs_knn_supervoxels.size();
        
    } else {
        LOG(ERROR) << "Unknown method: " << method;
        return -1;
    }

    LOG(INFO) << n_out_supervoxels << " supervoxels computed";


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Write segmented point cloud to file
    LOG(INFO) << "Writing segmented point cloud to " << out_file_name;
    std::ofstream out_file(out_file_name);
    if (!out_file.is_open()) {
        LOG(ERROR) << "Could not open output file " << out_file_name;
        return -1;
    }

    // Create color labels
    cl::Array<cl::RGB32Color> colors(labels.size());
    std::mt19937 random;
    cl::Array<cl::RGB32Color> supervoxel_colors(n_out_supervoxels);
    for (int i = 0; i < n_out_supervoxels; ++i) {
        supervoxel_colors[i] = cl::RGB32Color(random());
    }
    for (int i = 0; i < labels.size(); ++i) {
        colors[i] = supervoxel_colors[labels[i]];
    }

    // Write points with their labels
    for (int i = 0; i < n_points; ++i) {
        out_file << cloud->points[i].x << " "
                 << cloud->points[i].y << " "
                 << cloud->points[i].z << " "
                 << cloud->points[i].normal_x << " "
                 << cloud->points[i].normal_y << " "
                 << cloud->points[i].normal_z << " "
                 << static_cast<int>(cloud->points[i].r) << " "
                 << static_cast<int>(cloud->points[i].g) << " "
                 << static_cast<int>(cloud->points[i].b) << " "
                 << labels[i] << " "
                 << static_cast<int>(colors[i].red()) << " "
                 << static_cast<int>(colors[i].green()) << " "
                 << static_cast<int>(colors[i].blue()) << "\n";
    }
    out_file.close();

    return 0;

}