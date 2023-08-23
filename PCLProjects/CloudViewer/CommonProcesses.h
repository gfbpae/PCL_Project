#ifndef COMMONPROCESSES_H
#define COMMONPROCESSES_H

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/cloud_viewer.h>

#include "RANSAC.h"
#include "CommonProcesses.h"
#include "RegionGrowing.h"

using namespace std;
/** CommonProcesses Class => Reading and writing from the file, scaling and visualization operations are done in this class.*/
class CommonProcesses 
{
	public:
		/** Empty Consturcor. Initialize cloud and cloud_rgb variables using initializer list*/
		CommonProcesses();
		/** Destructor*/
		~CommonProcesses();
		/** 
		First input for file name
		Second input for the member where the read information will be stored
		It is used to store the data read from the pcd file.
		*/
		void ReadData(string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
		/** 
		First input for where to get data 
		Second input for viewer screen name
		Used to visualize the point cloud		
		*/
		void PCDViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ,string );
		/**
		First input for where to get data
		Second input for scale value for x axis
		Third input for scale value for y axis
		Fourt input for scale value for z axis
		Used for scaling the point cloud
		*/
		void ScalePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,double x_s=1,double y_s=1,double z_s=1);
		/** Returns the cloud_rgb */
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr& getRGBCloud();
		/** Returns the cloud */
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr& getCloud();
		/** Takes Ptr of PointCloud that is store PointXYZRGB and set this to cloud_rgb */
		void setRGBCloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr&);
		/** Takes Ptr of PointCloud that is store PointXYZRGB and set this to cloud */
		void setCloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr&);
		/** Used to save the given PointCloud to a new pcd*/
		void saveFile(string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&)const;
		private:
		/** It ensures that the values read from the file are kept via pointer*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		/** Keeps the colored PointCloud as via pointer*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb;

};
#endif
