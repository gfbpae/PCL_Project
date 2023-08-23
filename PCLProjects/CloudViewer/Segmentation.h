#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
/** Segmentation Class => Used for coloring point clouds */
class Segmentation 
{
	public:
		/** Empty Constructor*/
		Segmentation();
		/** Destructor*/
		~Segmentation();
		/** 
		Takes Ptr of PointCloud that is store PointXYZRGB 
		Generates random r, g, b values with the help of the rand function and applies them to each of the cloud's points
		*/
		void Colorized(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** */
		void Clustering();
	protected:
};

#endif
