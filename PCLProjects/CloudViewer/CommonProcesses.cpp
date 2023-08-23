#include "CommonProcesses.h"

CommonProcesses::CommonProcesses() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>),cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	ReadData("depth_image0101_sampling.pcd", cloud);
	PCDViewer(cloud, "Deneme1");
	/*RANSAC_Mine deneme;
	setRGBCloud(deneme.ransac_segment_method(getCloud()));
	PCDViewer(getRGBCloud(), "Deneme2");*/
	RegionGrowing deneme;
	setRGBCloud(deneme.rg_segment_method(getCloud()));
	PCDViewer(getRGBCloud(), "Deneme2");
}
CommonProcesses::~CommonProcesses()
{
}

void CommonProcesses::ReadData(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename,*cloud);
}
void CommonProcesses::PCDViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,string x)
{
	pcl::visualization::CloudViewer viewer (x);
	viewer.showCloud (cloud);
   	while (!viewer.wasStopped ())
   	{
   	}
}
void CommonProcesses::ScalePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,double x_s,double y_s,double z_s)
{
	for(int i=0;i<cloud->points.size();i++)
	{
		cloud->points[i].x=(cloud->points[i].x)*x_s;
		cloud->points[i].y=(cloud->points[i].y)*y_s;
		cloud->points[i].z=(cloud->points[i].z)*z_s; 
	}
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr& CommonProcesses::getRGBCloud()
{
	return cloud_rgb;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr& CommonProcesses::getCloud()
{
	return cloud;
}
void CommonProcesses::setRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
	pcl::copyPointCloud(*x, *cloud_rgb);
}
void CommonProcesses::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
	pcl::copyPointCloud(*x, *cloud);
}
void CommonProcesses::saveFile(std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x) const
{
	//pcl::io::savePCDFile(file_name, x);
}















