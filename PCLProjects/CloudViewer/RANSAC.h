#ifndef RANSAC_Mine_H
#define RANSAC_Mine_H

#include <iostream>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Segmentation.h"
/** RANSAC_Mine Class => Ransac segmentation is applied to the point cloud. It is derived from Segmentation Class. */
class RANSAC_Mine : public Segmentation{
	public:
		/** 
		Empty Constructor.
		Initialize cloud_inliers, cloud_outliers, cloud_rgb, cloud_rgb2, inliers, coefficients variables using initializer list
		*/
		RANSAC_Mine();
		/** Destructor*/
		~RANSAC_Mine();
		/** 
		Function used to do ransac segmentation
		Takes Ptr of PointCloud that is store PointXYZRGB to first cloud
		then returns finale cloud that in type Ptr of PointCloud that is store PointXYZRGB 
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ransac_segment_method(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
		/** Returns Outliers Ptr */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getOutliers();
		/** Returns Inliers Ptr */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getInliers();
		/** Set extract settings with taking Ptr of PointCloud */
		void setExtract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** Set seg settings with taking Ptr of PointCloud */
		void setSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** Set seg's default settings */
		void setSegDefaults();
	private :
		/** In order to perform ransac segmentation, seg object is created  from the SACSegmentation class.*/
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		/** */
		pcl::ModelCoefficients::Ptr coefficients;
		/** */
		pcl::PointIndices::Ptr inliers;
		/** Set extract object to filter from this class Extract Indices */
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		/** Keeps the inliers PointCloud's Ptr that is got as a result of filtering */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers;
		/** keeps the outliers PointCloud's Ptr that is got as a result of filtering*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers;
		/** Holds a segmented and colored PointCloud's Ptr */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb;
		/** Used as temporary PointCloud's Ptr */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb2;
};

#endif
