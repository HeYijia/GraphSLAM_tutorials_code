#include <iostream>
#include <string>
#include <fstream>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include "g2o/core/sparse_optimizer.h"

using namespace cv;
using namespace std;
using namespace pcl;

void showmsg()
{
    cerr<< "please set the dataset file path:"<<endl;
    cout<< "--this project use TUM RGB-D benchmark as test data"<<endl;
    cout<< "--you can down load it from the website!"<<endl;
    cout<< "--three para , argv demo: ./rgbdslam ~/rgbddata/ rgb.txt depth.txt "<<endl;
}

string subFileName(string line)
{
    const char* spacekey = strchr(line.c_str(),' ');
    int index = (int)(spacekey - line.c_str());
    cout <<"index is :" << index <<endl;
    string result = line.substr(index+1);
    return result;
}

float factor = 5000.0;
float cx = 319.5;
float cy = 239.5;
float fx = 525.0;
float fy = 525.0;

int main(int argc, char** argv)
{
    ifstream rgbfile;
    ifstream dptfile;
    string rgbline,dptline;
    string rgbfilepath;
    string dptfilepath;
    string rootpath;
    if( argc < 4 )
    {
        showmsg();
        return -1;
    }
    else
    {
        cout<<argv[1]<<"  "<<argv[2]<<endl;
        rootpath = argv[1];
        rgbfilepath = rootpath + argv[2];
        dptfilepath = rootpath + argv[3];
        cout<<"data file:"<<dptfilepath<<" "<<rgbfilepath<<endl;
        rgbfile.open(rgbfilepath.c_str());
        dptfile.open(dptfilepath.c_str());
        //rgbfile.open("/home/hyj/rgbddata/rgb.txt");
        //dptfile.open("/home/hyj/rgbddata/depth.txt");

        if(!rgbfile.is_open() && !dptfile.is_open())
        {
            cerr<<" file is not exit"<<endl;
            return -1;
        }
        else
        {
            for(int i =0;i<3;i++)
            {
                getline(rgbfile,rgbline);
                getline(dptfile,dptline);
                cout<<rgbline<<endl;
            }
        }

    }
    string imgpath;
    Mat current_rgb,current_dpt;

    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);
    
    //Mat next_rgb,next_dpt;
    
    while(!rgbfile.eof())
    {

        getline(rgbfile,rgbline);
        cout<<rgbline<<endl;
        string dat = subFileName(rgbline);
        current_rgb = imread(rootpath+dat);
        
        getline(dptfile,dptline);
        dat = subFileName(dptline);
        current_dpt = imread(rootpath+dat,-1);
        
        cloud->width = current_dpt.cols;
        cloud->height = current_dpt.rows;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for(int v = 0; v < current_dpt.rows; v++)
        {
            const uchar *rgbptr = current_rgb.ptr<uchar>(v);
            for(int u = 0; u< current_dpt.cols; u++)
            {
                PointXYZRGBA xyzrgb;

                // rgb
                const uchar *pixel = rgbptr;
                xyzrgb.b = pixel[0];
                xyzrgb.g = pixel[1];
                xyzrgb.r = pixel[2];
                rgbptr += 3;

                // depth
                float Z = current_dpt.ptr<ushort>(v)[u]/factor;
                float X = (u - cx)*Z/fx;
                float Y = (v - cy)*Z/fy;

                if(Z == 0.0)
                {
                    xyzrgb.x = xyzrgb.y = xyzrgb.z = numeric_limits<float>::quiet_NaN();
                    
                }
                else
                {
                    xyzrgb.z = Z;
                    xyzrgb.x = X;
                    xyzrgb.y = Y;

                }
                cloud->at(u,v) = xyzrgb;
            }
        }
    }
    clock_t begin,end;
    double t;
    begin = clock();
    /* your code */
    end = clock();
    t = (double)(end - begin )/CLOCKS_PER_SEC;
    cout << "time spend: "<< t <<endl;
    /*
    visualization::CloudViewer viewer("test");
    viewer.showCloud(cloud);
    while( ! viewer.wasStopped())
    {
        imshow("rgb",current_rgb);
        imshow("dpt",current_dpt);

        waitKey(0);

    }*/

    return 0;
}
