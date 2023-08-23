#include "RegionGrowing.h"

RegionGrowing::RegionGrowing():tree(new pcl::search::KdTree<pcl::PointXYZRGB>),
                               normals(new pcl::PointCloud <pcl::Normal>),
                               indices(new std::vector <int>),
                               colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
}

RegionGrowing::~RegionGrowing()
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr& RegionGrowing::rg_segment_method(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    estimator(cloud);
    filter(cloud);
    rg_set(cloud);

    int i_segment;
    int i_point;
    for (i_segment = 0; i_segment < clusters.size(); i_segment++)
    {   
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (i_point = 0; i_point < clusters[i_segment].indices.size(); i_point++)
        {            
            colored_cloud2->push_back(cloud->points[clusters[i_segment].indices[i_point]]);
        }
        Colorized(colored_cloud2);
        *colored_cloud += *colored_cloud2;
    }
    return colored_cloud;
}

void RegionGrowing::estimator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(x);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
}

void RegionGrowing::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
    pass.setInputCloud(x);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*indices);
}

void RegionGrowing::rg_set(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
    reg_grow.setMinClusterSize(50);
    reg_grow.setMaxClusterSize(1000000);
    reg_grow.setSearchMethod(tree);
    reg_grow.setNumberOfNeighbours(30);
    reg_grow.setInputCloud(x);
    reg_grow.setInputNormals(normals);
    reg_grow.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg_grow.setCurvatureThreshold(1.0);
    reg_grow.extract(clusters);
}
