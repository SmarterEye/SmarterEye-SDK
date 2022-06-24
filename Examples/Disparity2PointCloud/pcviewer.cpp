#include "pcviewer.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <time.h>

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

PCViewer::PCViewer(): 
      save_dir_(""),
      viewer_(nullptr),
      save_pcd_(false)
{}

PCViewer::PCViewer(bool save_pcd, const std::string &pcd_save_dir): 
    save_dir_(pcd_save_dir),
    viewer_(nullptr),
    save_pcd_(save_pcd)
{

}

PCViewer::~PCViewer()
{
    viewer_->close();
}

void PCViewer::update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    if (save_pcd_) {
        if (cloud && !cloud->empty() && (pcl::io::savePCDFile(save_dir_, *cloud)) == 0) {
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
