#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std; 

const string OUT_DIR = "D:\\frame_saver_output\\"; 

class SimpleOpenNIViewer 
{ 
public: 
    SimpleOpenNIViewer () : viewer ("PCL Viewer") 
    { 
                frames_saved = 0; 
                save_one = false; 
    } 

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
    { 
                if (!viewer.wasStopped()) { 
                        viewer.showCloud (cloud); 

                        if( save_one ) { 
                                save_one = false; 
                                std::stringstream out; 
                                out << frames_saved; 
                                std::string name = "cloud.pcd"; 
                                pcl::io::savePCDFileASCII( name, *cloud ); 
                        } 
                } 
    } 

    void run () 
    { 
                pcl::Grabber* interface = new pcl::OpenNIGrabber(); 

                boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
                        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 

                interface->registerCallback (f); 

                interface->start (); 

                char c; 

                while (!viewer.wasStopped()) 
                { 
                        //sleep (1); 

                      
                        if(getchar()) { 
                                cout << "Saving frame " << frames_saved << ".\n"; 
                                frames_saved++; 
                                save_one = true; 
                        } 
                } 

                interface->stop (); 
        } 

        pcl::visualization::CloudViewer viewer; 

        private: 
                int frames_saved; 
                bool save_one; 

}; 

int main () 
{ 
    SimpleOpenNIViewer v; 
    v.run (); 
    return 0; 
} 
