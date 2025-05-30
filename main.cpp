#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "CustomICP.h"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("0.pcd", *source) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *target) == -1) {
        PCL_ERROR("Couldn't read the PCD files!\n");
        return (-1);
    }

    CustomICP icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaxCorrespondenceDistance(0.1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*output);
    pcl::io::savePCDFile("11.pcd", *output);
  
    std::cout << "Final transformation matrix:" << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    //std::cout << "converged: " << icp.hasConverged() << "; score: " << icp.getFitnessScore() << std::endl;
    

    // 可视化结果
    pcl::visualization::PCLVisualizer viewer("ICP Result");
    viewer.addPointCloud<pcl::PointXYZ>(source, "source");
    viewer.addPointCloud<pcl::PointXYZ>(target, "target");
    viewer.addPointCloud<pcl::PointXYZ>(output, "aligned");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "aligned");

    viewer.spin();

    return 0;
}