#include "RANSAC.h"

RANSAC_Mine::RANSAC_Mine():cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>),
						   cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>),
						   cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>),
						   cloud_rgb2(new pcl::PointCloud<pcl::PointXYZRGB>),
						   inliers(new pcl::PointIndices),
						   coefficients(new pcl::ModelCoefficients)
{
}

RANSAC_Mine::~RANSAC_Mine()
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr& RANSAC_Mine::ransac_segment_method(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    setSegDefaults();
    setSeg(cloud);
    setExtract(cloud);
    int nr = cloud->size();

    while (!(getInliers()->empty()))
    {    
        pcl::copyPointCloud(*getInliers(), *cloud_rgb2);
		Colorized(cloud_rgb2);
        *cloud_rgb += *cloud_rgb2;
        setExtract(getOutliers());
    }
    return cloud_rgb;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr& RANSAC_Mine::getOutliers()
{
    extract.setNegative(true);
    extract.filter(*cloud_outliers);
    return cloud_outliers;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr& RANSAC_Mine::getInliers()
{
    extract.setNegative(false);
    extract.filter(*cloud_inliers);
    return cloud_inliers;
}

void RANSAC_Mine::setExtract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
    extract.setInputCloud(x);
    extract.setIndices(inliers);
}

void RANSAC_Mine::setSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
    seg.setInputCloud(x);
    seg.segment(*inliers, *coefficients);
}

void RANSAC_Mine::setSegDefaults()
{
	seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000000);
    seg.setDistanceThreshold(0.01);
}

