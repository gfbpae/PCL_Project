#include "Segmentation.h"

Segmentation::Segmentation()
{
}

Segmentation::~Segmentation()
{
}

void Segmentation::Colorized(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& x)
{
	float r = rand() % 256;
    float g = rand() % 256;
    float b = rand() % 256;
    for (int i = 0; i < x->size(); i++)
    {
        pcl::PointXYZRGB& point = x->points[i];
        point.r = r;
        point.g = g;
        point.b = b;
    }
}
void Segmentation::Clustering()
{
	
}

