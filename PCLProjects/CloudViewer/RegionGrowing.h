#ifndef REGIONGROWING_H
#define REGIONGROWING_H

#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include "Segmentation.h"
/** RegionGrowing Class => Region Growing segmentation is applied to the point cloud. It is derived from Segmentation Class. */
class RegionGrowing : public Segmentation
{
	
	public:
		/** 
		Empty Constructor
		Initialize tree, normals, indices, colored_cloud variables using initializer list
		*/
		RegionGrowing();
		/** Destructor*/
		~RegionGrowing();
		/** 
		Function used to do region growing segmentation
		Takes Ptr of PointCloud that is store PointXYZRGB to first cloud
		then returns finale colored_cloud that in type Ptr of PointCloud that is store PointXYZRGB 
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& rg_segment_method(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** To Set estimator default settings */
		void estimator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** To set filter default settings */
		void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** To set the necessary settings as default  for region growing */
		void rg_set(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
	private:
		/** In order to perform region growing segmentation, reg_grow object is created  from the RegionGrowing class.*/
		pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg_grow;
		/** Search algorithm used to find the closest points so tree Ptr object is created from Search Class*/
		pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
		/** Region Growing algorithm requires normals */
		pcl::PointCloud <pcl::Normal>::Ptr normals;
		/** To compute normals, normal_estimator object is created from NormalEstimation Class */
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
		/** */
		pcl::IndicesPtr indices;
		/** */
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		/** Holds a segmented PointIndices */
		std::vector <pcl::PointIndices> clusters;
		/** Holds a segmented and colored PointCloud's Ptr */
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
};

#endif
