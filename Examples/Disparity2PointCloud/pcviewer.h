#ifndef PCVIEWER_H
#define PCVIEWER_H

#include <pcl/visualization/cloud_viewer.h>

class PCViewer
{
public:
    PCViewer();
    PCViewer(bool save_pcd, const std::string &pcd_save_dir = "");
    ~PCViewer();

    void update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    bool wasStopped() const;
    void close();
    std::string save_dir_;
private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    bool save_pcd_;
};

#endif // PCVIEWER_H
