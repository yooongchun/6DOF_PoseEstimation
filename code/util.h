#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pcl/common/centroid.h>
#include <Eigen/Geometry> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>
#define Pi 3.1415926
typedef pcl::PointXYZRGBA PointT;  // The point type used for input
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
using namespace pcl;
using namespace std;

void Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr plane, double *sum_x, double *sum_y, double *sum_z) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::SACSegmentation<pcl::PointXYZ> new_seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //设置聚类的内点索引
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//平面模型的因子
																		 // Optional
	new_seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
										   // Mandatory
	new_seg.setModelType(pcl::SACMODEL_PLANE);//
	new_seg.setMethodType(pcl::SAC_RANSAC); // 设置用哪个随机参数估计方法
	new_seg.setMaxIterations(1000);
	new_seg.setDistanceThreshold(0.01);////设置判断是否为模型内点的距离阈值

									   // Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> new_extract;

	// Segment the largest planar component from the remaining cloud

	new_seg.setInputCloud(cloud);
	new_seg.segment(*inliers, *coefficients);
	new_extract.setInputCloud(cloud);
	new_extract.setIndices(inliers);
	new_extract.setNegative(false);
	new_extract.filter(*plane);

	*sum_x = double(coefficients->values[0]);
	*sum_y = double(coefficients->values[1]);
	*sum_z = double(coefficients->values[2]);

}


void FFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered) {

	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.filter(*cloud_filtered);
}

void Euclid_Seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *sourceClouds) {
	
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //设置聚类的内点索引
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//平面模型的因子
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);    //分割模型
	seg.setMethodType(pcl::SAC_RANSAC);       //随机参数估计方法
	seg.setMaxIterations(100);                //最大的迭代的次数
	seg.setDistanceThreshold(0.02);
	int i = 0, nr_points = (int)cloud->points.size();//剩余点云的数量
	while (cloud->points.size() > 0.3 * nr_points) {
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);        //提取内点的索引并存储在其中
		extract.setNegative(true);
		extract.filter(*cloud_f);//?
		*cloud = *cloud_f;
	}

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;   //欧式聚类对象
	ec.setClusterTolerance(0.02);                     // 设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(500);                 //设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(25000);               //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                    //设置点云的搜索机制
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
		}
		//pcl::io::savePCDFile("final.pcd",*cloud_cluster);
		sourceClouds->push_back(cloud_cluster);


	}
	//cout<<sourceClouds->size();

}


void NNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr new_tree(new pcl::search::KdTree<pcl::PointXYZ>);
	new_tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(new_tree);
	n.setKSearch(20);
	n.compute(*normals);
}

void AverageNormal(pcl::PointCloud<pcl::Normal>::Ptr normals, double *x, double *y, double *z) {          ///need imporving

	for (size_t i = 0; i < normals->points.size(); ++i) {
		*x += normals->points[i].normal_x / normals->points.size();
		*y += normals->points[i].normal_y / normals->points.size();
		*z += normals->points[i].normal_z / normals->points.size();
	}

}

double Score(pcl::PointCloud<pcl::Normal>::Ptr normals, double *sumx, double *sumy, double *sumz) {
	double sum_abs, x, y, z, point_abs, dot, score, angle, single_score;
	single_score = 0.0;
	for (size_t i = 0; i < normals->points.size(); ++i) {
		sum_abs = sqrt((*sumx)*(*sumx) + (*sumy)*(*sumy) + (*sumz)*(*sumz));
		x = normals->points[i].normal_x;
		y = normals->points[i].normal_y;
		z = normals->points[i].normal_z;
		point_abs = sqrt(x*x + y * y + z * z);
		dot = fabs((*sumx)*x + (*sumy)*y + (*sumz)*z);

		angle = acos(dot / sum_abs / point_abs) * 180 / Pi;

		single_score += (angle*100.0 / 90.0);


	}
	score = int(single_score) / normals->points.size();
	cout << "score  " << score << endl;
	return score;

}


void NotPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane) {

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::SACSegmentation<pcl::PointXYZRGB> new_seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //设置聚类的内点索引
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//平面模型的因子
																		 // Optional
	new_seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
										   // Mandatory
	new_seg.setModelType(pcl::SACMODEL_PLANE);//
	new_seg.setMethodType(pcl::SAC_RANSAC); // 设置用哪个随机参数估计方法
	new_seg.setMaxIterations(1000);
	new_seg.setDistanceThreshold(0.01);////设置判断是否为模型内点的距离阈值

									   // Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> new_extract;

	// Segment the largest planar component from the remaining cloud

	new_seg.setInputCloud(cloud);
	new_seg.segment(*inliers, *coefficients);
	new_extract.setInputCloud(cloud);
	new_extract.setIndices(inliers);
	new_extract.setNegative(true);
	new_extract.filter(*plane);


}

double Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	double score;
	double sum_x, sum_y, sum_z = 0;
	pcl::PointCloud<pcl::Normal>::Ptr normals_seg(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_orig(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr new_coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr new_inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> new_seg;
	// Optional
	new_seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
										   // Mandatory
	new_seg.setModelType(pcl::SACMODEL_PLANE);//
	new_seg.setMethodType(pcl::SAC_RANSAC); // 设置用哪个随机参数估计方法
	new_seg.setMaxIterations(1000);
	new_seg.setDistanceThreshold(0.01);////设置判断是否为模型内点的距离阈值

									   // Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> new_extract;

	// Segment the largest planar component from the remaining cloud

	new_seg.setInputCloud(cloud);
	new_seg.segment(*new_inliers, *new_coefficients);
	if (new_inliers->indices.size() == 0)
	{
		return (0);
	}
	new_extract.setInputCloud(cloud);
	new_extract.setIndices(new_inliers);
	new_extract.setNegative(false);
	new_extract.filter(*cloud_p);
	new_extract.setNegative(true);
	new_extract.filter(*cloud_f);
	if (cloud_p->size() == 0 || cloud_f->size() == 0) {
		return (0);
	}
	//NNormal(cloud_p,normals_seg);
	NNormal(cloud_f, normals_orig);
	//AverageNormal(normals_seg,&sum_x,&sum_y,&sum_z);
	sum_x = new_coefficients->values[0];
	sum_y = new_coefficients->values[1];
	sum_z = new_coefficients->values[2];
	score = Score(normals_orig, &sum_x, &sum_y, &sum_z);
	return score;

}


double Lccp(pcl::PointCloud<PointT>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud) {

	double voxel_resolution = 0.0075f;
	double seed_resolution = 0.03f;
	double color_importance = 0.0f;
	double spatial_importance = 1.0f;
	double normal_importance = 3.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	// LCCPSegmentation Stuff
	double concavity_tolerance_threshold = 10;
	double smoothness_threshold = 0.1;
	uint32_t min_segment_size = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	unsigned int k_factor = 0;

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(input_cloud_ptr);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	//PCL_INFO ("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);
	//PCL_INFO ("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);


	/// The Main Step: Perform LCCPSegmentation
	//PCL_INFO ("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSanityCheck(use_sanity_criterion);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);



	int a[100] = { 0 };
	int max = 0;
	int index = 0;
	int j = 0;
	for (int i = 0; i<lccp_labeled_cloud->size(); i++) {
		a[lccp_labeled_cloud->points[i].label]++;
	}

	for (int i = 0; i<100; i++) {
		//std::cout<<a[i]<<endl;
		if (a[i]>max) {
			max = a[i];
			index = i;
		}
	}


	test_cloud->width = max;
	test_cloud->height = 1;
	test_cloud->is_dense = false;

	test_cloud->points.resize(max);


	for (int i = 0; i<lccp_labeled_cloud->size(); i++) {

		if (lccp_labeled_cloud->points[i].label == index) {
			test_cloud->points[j].x = lccp_labeled_cloud->points[i].x;
			test_cloud->points[j].y = lccp_labeled_cloud->points[i].y;
			test_cloud->points[j].z = lccp_labeled_cloud->points[i].z;
			j++;
		}

	}
	double percent = double(test_cloud->size()) / double(lccp_labeled_cloud->size());
	return percent;

}

void Estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final, Eigen::Affine3f *tt, double *sum_x, double *sum_y, double *sum_z) {

	pcl::PointCloud<pcl::Normal>::Ptr normals_final(new pcl::PointCloud<pcl::Normal>);
	//NNormal(cloud_final,normals_final);
	//AverageNormal(normals_final,&sum_x,&sum_y,&sum_z);
	Eigen::Vector4f centroid;
	double alpha, gama;
	alpha = atan(*sum_z / (*sum_y));
	gama = asin(-*sum_x);
	double sumx = *sum_x;
	double sumy = *sum_y;
	double sumz = *sum_z;
	double sum_abs, x, y, z, point_abs, dot, angle;
	sum_abs = sqrt((sumx)*(sumx)+(sumy)*(sumy)+(sumz)*(sumz));
	x = 0;
	y = 0;
	z = 1;
	point_abs = sqrt(x*x + y * y + z * z);
	dot = fabs((sumx)*x + (sumy)*y + (sumz)*z);
	//cout<<"dot"<<dot<<sum_abs<<point_abs<<endl;
	//cout<<sum_abs<<endl;
	angle = acos(dot / sum_abs / point_abs) * 180 / Pi;
	cout << "angle" << angle << endl;





	pcl::compute3DCentroid(*cloud_final, centroid);
	*tt = Eigen::Translation3f(centroid[0], centroid[1], centroid[2])*Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(gama, Eigen::Vector3f::UnitZ());

}




