#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <algorithm>


using namespace std;


int w_range[2] = {-20,20};//左右范围
int fv_range[2] = {-20,20};//前后范围
float height_range[2] = {-1,3};//高度范围
float resolution = 0.05;//分辨率

static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t)tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

int main(int argc, char** argv)
{
    int64_t tm0 = gtm();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//初始化点云 
	
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);//加载pcd点云并放入cloud中

    cout<<"点云数量："<<cloud->points.size()<<endl;
  
    size_t len = cloud->points.size();

    vector<int> x,y;
    vector<float> z;

    float dx = w_range[1]/resolution;
    float dy = fv_range[1]/resolution; 

    float ix,iy;

    for (size_t i = 0; i < len; i++)
	{
            if(cloud->points[i].y>fv_range[0] && cloud->points[i].y<fv_range[1])
             {
                 if(cloud->points[i].x>w_range[0] && cloud->points[i].x<w_range[1])
                 {
                     if(cloud->points[i].z>height_range[0] && cloud->points[i].z<height_range[1])
                     {
                         //分辨率转换
                         float xr= cloud->points[i].x/resolution;
                         float yr= cloud->points[i].y/resolution;
                         
                         //坐标转换
                         ix = (dx + int(xr));
                         iy = (dy - int(yr));

                         x.push_back(round(ix));
                         y.push_back(round(iy));
                         z.push_back(cloud->points[i].z);
                      }
                  }
           }

        }		
    
    float xmax = *max_element(x.begin(),x.end());
    float xmin = *min_element(x.begin(),x.end());

    cout<<"x坐标最大值："<<xmax<<endl;
    cout<<"x坐标最小值："<<xmin<<endl;

    
    float ymax = *max_element(y.begin(),y.end());
    float ymin = *min_element(y.begin(),y.end());

    cout<<"y坐标最大值："<<ymax<<endl;
    cout<<"y坐标最小值："<<ymin<<endl;

    int imgheight = 1+ (fv_range[1]-fv_range[0])/resolution;
    int imgwidth = 1+ (w_range[1] - w_range[0])/resolution;

    float zmax = *max_element(z.begin(),z.end());
    float zmin = *min_element(z.begin(),z.end());

    cout<<"高度最大值："<<zmax<<endl;
    cout<<"高度最小值："<<zmin<<endl;

    cv::Mat bvimage = cv::Mat::zeros(imgheight, imgwidth, CV_8UC3);//生成一张黑色图


    vector<float> gray;//灰度值

    for (int j=0; j<z.size(); j++){
        gray.push_back((z[j]-zmin)/(zmax-zmin)*256);
       
    }


   for (int i=0; i<x.size(); i++){
        bvimage.at<cv::Vec3b>(y[i], x[i]) = cv::Vec3b(gray[i],gray[i],gray[i]);
    }
    
    int64_t tm1 = gtm();
    cout<<"耗时(s)："<<tm1-tm0<<endl;

    string name = argv[1];
    cv::imshow("BEV", bvimage);
    cv::imwrite(name+".jpg",bvimage);    

    pcl::visualization::CloudViewer viewer("PCD");
    viewer.showCloud(cloud);

    cv::waitKey(0);

    while (!viewer.wasStopped())
    {
	}
     

    return 0;
}
