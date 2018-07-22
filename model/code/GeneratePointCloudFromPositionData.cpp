#include <iostream> 
#include <fstream>
// PCL 库  
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
// 定义点云类型  
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

int main(int argc,char* argv[])
{	if(argc<2){
	  cerr<<sizeof(argc)/sizeof(int)<<2<<" less paras to run..."<<endl;
	  return -1;
	}
	// 点云变量  
	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	fstream infile;
	infile.open(argv[1], ios::in);
	if (!infile.is_open()){
	  cerr<<"open file error."<<endl;
	  return -1;
	}
	PointCloud::Ptr cloud(new PointCloud);
	float x, y, z;
	int r, g, b;
	while (!infile.eof())
	{
		infile >> x >> y >> z >> r >> g >> b;
		// d 存在值，则向点云增加一个点  
		PointT p;
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = r;
		p.g = g;
		p.b = b;
		// 把p加入到点云中 
		cloud->points.push_back(p);
	}
	infile.close();
	// 设置并保存点云 
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout <<"  point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile(argv[2], *cloud);
	/*pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(cloud);*/
	// 清除数据并退出  
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
	return 0;
}

