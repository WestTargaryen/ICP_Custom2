//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//
//int main() {
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
//
//	if (pcl::io::loadPCDFile<pcl::PointXYZ>("0.pcd", *source) == -1 ||
//		pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *target) == -1) {
//		PCL_ERROR("Couldn't read the PCD files!\n");
//		return (-1);
//	}
//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//	icp.setInputSource(source);
//	icp.setInputTarget(target);
//	icp.setMaximumIterations(100);
//	icp.setTransformationEpsilon(1e-8);
//	icp.setMaxCorrespondenceDistance(0.1);
//	pcl::PointCloud<pcl::PointXYZ> Final;
//	icp.align(Final);
//
//	pcl::io::savePCDFile("11.pcd", Final);
//	std::cout << "converged: " << icp.hasConverged() << "; score: " << icp.getFitnessScore() << std::endl;
//	std::cout << "transformation: " << std::endl << icp.getFinalTransformation() << std::endl;
//	
//	return (0);
//}
