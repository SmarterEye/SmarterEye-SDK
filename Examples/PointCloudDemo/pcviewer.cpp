#include "pcviewer.h"

#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <iostream>
#include <time.h>

#ifdef _WINDOWS
static const std::string kHomeDir("d:");
#else
static const std::string kHomeDir = getenv("HOME") ? getenv("HOME") : "/var";;
#endif

std::shared_ptr<pcl::visualization::PCLVisualizer> customColorVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->addPointCloud(cloud, "points");
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  return viewer;
}

PCViewer::PCViewer()
    : viewer_(nullptr),
      save_pcd_(false),
      save_dir_("")
{}

PCViewer::PCViewer(bool save_pcd, const std::string &pcd_save_dir)
    : viewer_(nullptr),
      save_pcd_(save_pcd),
      save_dir_(pcd_save_dir)
{
    if (save_dir_.empty() || !boost::filesystem::is_directory(save_dir_)) {
        save_dir_ = kHomeDir + "/adas_data";
        if (!boost::filesystem::exists(save_dir_)) {
            boost::filesystem::create_directory(save_dir_);
        }
    }

    if (*save_dir_.rbegin() != '/') {
        save_dir_ += "/";
    }
}

PCViewer::~PCViewer()
{
    viewer_->close();
}

void PCViewer::update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int threadId)
{
    if (save_pcd_) {
        static size_t count = 0;

        std::stringstream ss;
        ss << save_dir_ << "pointcloud_" << count << ".pcd";
        auto pcd_path = ss.str();
		
        if (cloud && !cloud->empty() && (pcl::io::savePCDFile(pcd_path, *cloud)) == 0) {
            count++;
            //std::cout << "currentTime =  " << currentTimetoStr() << " saved pcd file: " << pcd_path << ", count " << count << " threadId = " << threadId << std::endl;

            // only save onece, because saving and drawing runs slowly at the same time
            save_pcd_ = true;
        }
    }

    /*if (viewer_ == nullptr) {
        viewer_ = customColorVis(cloud);
    }

    if (!viewer_->wasStopped()) {
        viewer_->updatePointCloud(cloud, "points");
        viewer_->spinOnce();
    }*/
}

bool PCViewer::wasStopped() const
{
    return viewer_ != nullptr && viewer_->wasStopped();
}

void PCViewer::close()
{
    if (viewer_) {
        viewer_->removeAllPointClouds();
        viewer_->close();
    }
}
