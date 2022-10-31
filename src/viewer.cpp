#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
	//------------------------------------ 加载点云 ---------------------------------
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDReader reader;	
	if (reader.read("out2.pcd", *cloud) < 0)
	{
		PCL_ERROR("\a->点云文件不存在！\n");
		system("pause");
		return -1;
	}
	cout << "->加载了 " << cloud->points.size() << " 个数据点" << endl;
	//==============================================================================


	//----------------------------- Visualizer 可视化 ------------------------------
	///必选
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Visualizer_Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));			//不采用智能共享指针boost::shared_ptr创建视窗对象
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample_cloud");		//将点云添加到视窗对象中，并定义一个唯一的ID“sample_cloud”

	///属性设置（可选）
	viewer->setBackgroundColor(0, 0, 0);		//窗口背景色，默认[0,0,0]，范围[0~255,0~255,0~255]
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample_cloud");			//设置点的大小，默认 1
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "sample_cloud");	//设置点云显示的颜色，rgb 在 [0,1] 范围
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "sample_cloud");			//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】
	// viewer->addText("Tree", 10, 10, "text_1");	//在指定位置添加文字

    viewer->spin(); 
	///必选
	// while (!viewer->wasStopped())
	// {
	// 	viewer->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }
	//==============================================================================

	return 0;
}
