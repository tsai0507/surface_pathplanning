#include <iostream>
#include <vector>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class pc2path
{
private:
    /* data */
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
    Eigen::Vector4f centroid;
    void get_center(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr);
    void cutpc(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr);
    pcl::PointXYZRGBNormal initgetleftbound();
    void extract_indices();

public:
    pc2path(/* args */);
    ~pc2path();
    bool readpcd(std::string);
    bool getcloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr);
    void showpc(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,pcl::PointXYZRGBNormal);
    void pathplanning(void);
    
};

pc2path::pc2path(/* args */)
{
    cloud.reset(new(pcl::PointCloud<pcl::PointXYZRGBNormal>));
}

pc2path::~pc2path()
{
}

bool pc2path::readpcd(std::string filename){
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(filename, *cloud) == -1)
    {
        PCL_ERROR("Cloudn't read file!");
        return 0;
    }
    return 1;
}

bool pc2path::getcloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outcloud){
    if(cloud->size()==0){
        return 0;
    }
    else{
        pcl::copyPointCloud(*cloud, *outcloud);
        return 1;
    }
}

void pc2path::showpc(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr showpc,pcl::PointXYZRGBNormal colorpoint){
    // get_center(showpc);
    // pcl::PointXYZRGBNormal tempc;
    // tempc.x=centroid[0];
    // tempc.y=centroid[1];
    // tempc.z=centroid[2];
    // tempc.g=255;
    // showpc->push_back(tempc);
    if(colorpoint){
        printf("show color\n");
        showpc->push_back(colorpoint);
    }

    pcl::visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
    vis3.addPointCloud<pcl::PointXYZRGBNormal> (showpc);
    // vis3.addPointCloudNormals<pcl::PointXYZRGBNormal> (showpc, 5, 0.2f, "cloud_normals");
    // vis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "VOXELIZED SAMPLES CLOUD");  // fail
    vis3.spin ();   
}

void pc2path::get_center(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud){
    pcl::compute3DCentroid(*cloud, centroid);
    cout << "\n->点云质心为：" 
		 << "(" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" << endl;
	
}

void pc2path::cutpc(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud){
    // get cloud centroid
    get_center(cloud); 
    // translation of pointcloud
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0,3) = -1*centroid[0];
    transform(1,3) = -1*centroid[1];
    transform(2,3) = -1*centroid[2];
    pcl::transformPointCloud (*cloud, *cloud, transform);

    // cut pointcloud    
    pcl::PassThrough<pcl::PointXYZRGBNormal> pass; // 声明直通滤波
	pass.setInputCloud(cloud); // 传入点云数据
	pass.setFilterFieldName("z"); // 设置操作的坐标轴
	pass.setFilterLimits(0.0, centroid(1)*2); // 设置坐标范围
	// pass.setFilterLimitsNegative(true); // 保留数据函数
	pass.filter(*cloud);  // 进行滤波输出
}

void pc2path::extract_indices(){
  
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg;
    
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05); 
    seg.setOptimizeCoefficients(true);
    seg.setRadiusLimits(0.1, 0.15);
    seg.setEpsAngle(100 / (180/3.141592654));
    seg.setMaxIterations(1000000);

    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // extract.setNegative(negative);
    extract.filter(*cloud_filtered);
    
    pcl::copyPointCloud(*cloud_filtered, *cloud);

}

float pc2path::distanceXY(pcl::PointXYZRGBNormal point){
    return std::sqrt((point.x-centroid.x)*(point.x-centroid.x) + (point.y-centroid.y)*(point.y-centroid.y) );
}

pcl::PointXYZRGBNormal pc2path::initgetleftbound(){
    // find top of surface
    extract_indices(); 
    get_center(cloud);
    // find min left point
    // find min x, then find max y (II quadrant)
    // only consider X-Y plane
    pcl::PointXYZRGBNormal startpoint = new pcl::PointXYZRGBNormal;
    pcl::PointXYZRGBNormal minPt, maxPt;
    pcl::getMinMax3D (*cloud, minPt, maxPt);
    std::vector<pcl::PointXYZRGBNormal> vpoint;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if(  cloud->points[i].x < minPt.x + 0.01){
            vpoint.push_back(cloud->points[i]);
        }
    }
    float pointy=0.0;
    for (int i = 0; i < vpoint.size(); i++)
    {
        if( cloud->points[i].y > pointy){
            pointy = cloud->points[i].y;
            startpoint.x = cloud->points[i].x;
            startpoint.y = cloud->points[i].y;
            startpoint.z = cloud->points[i].z;
            startpoint.r = cloud->points[i].r;
            startpoint.g = cloud->points[i].g;
            startpoint.b = cloud->points[i].b;
            startpoint.n_x = cloud->points[i].n_x;
            startpoint.n_y = cloud->points[i].n_y;
            startpoint.n_z = cloud->points[i].n_z;
            startpoint.curvature = cloud->points[i].curvature;
        }
    }
    return startpoint;
}

void pc2path::pathplanning(){
	// make new pc without bias, and find the top of surface
    cutpc(cloud);
    // find left point boundary
    pcl::PointXYZRGBNormal initleftpoint;
    initleftpoint = initgetleftbound();
}


int main(int argc, char **argv)
{
    if (argc < 2){
        pcl::console::print_error("%s pcd_file\n", argv[0]);
        return 1;
    }

    pc2path plan;
    plan.readpcd(argv[1]);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr readpc(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    plan.pathplanning();
    plan.getcloud(readpc);
    plan.showpc(readpc,NULL);
    

    return 0;
}
