#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
//for downsampling
#include <pcl/filters/voxel_grid.h>

using namespace std;

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
                                std::string name = "scene.pcd"; 
                                pcl::io::savePCDFileASCII( name, *cloud );
				downsample();
				sleep(3);
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
                        if(getchar()){

                                cout << "Saving frame " << frames_saved << ".\n"; 
                                frames_saved++; 
                                save_one = true; 
				sleep(4);
					//correspondence("model_hphone_ds.pcd","scene_ds.pcd");
				system("./correspondence_grouping model_face_ds.pcd scene_ds.pcd");
					
			}
			
                }

                interface->stop (); 
    } 
    void downsample(){
	  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	  // Fill in the cloud data
	  pcl::PCDReader reader;
	  // Replace the path below with the path where you saved your file
	  reader.read ("scene.pcd", *cloud);

	  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	       << " data points (" << pcl::getFieldsList (*cloud) << ").";

	  // Create the filtering object
	  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	  sor.setInputCloud (cloud);
	  sor.setLeafSize (0.01f, 0.01f, 0.01f);
	  sor.filter (*cloud_filtered);

	  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
	       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

	  pcl::PCDWriter writer;
	  writer.write ("scene_ds.pcd", *cloud_filtered, 
		 Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
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
