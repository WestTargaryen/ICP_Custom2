//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <chrono>            // ʱ�����
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/registration/icp.h> 
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//// ������Ʒ��߲�ƴ�ӵ�����������
//void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
//{
//	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	n.setNumberOfThreads(10);  
//	n.setInputCloud(cloud);    
//	n.setSearchMethod(tree);   
//	n.setKSearch(10);         
//	n.compute(*normals);       
//
//	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
//}
//
//// ���ӻ���׼���
//void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
//	pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& icp)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("��׼���"));
//
//	int v1 = 0, v2 = 1;
//	viewer->createViewPort(0, 0, 0.5, 1, v1);  
//	viewer->createViewPort(0.5, 0, 1, 1, v2);  
//	viewer->setBackgroundColor(0, 0, 0, v1);   
//	viewer->setBackgroundColor(0.05, 0, 0, v2);
//
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 255, 0, 0);   // Դ���ƺ�ɫ
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);   // Ŀ�������ɫ
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(icp, 0, 255, 0);     // ��׼�������ɫ
//
//	viewer->addPointCloud(source, src_h, "source cloud", v1);  
//	viewer->addPointCloud(target, tgt_h, "target cloud", v1); 
//	viewer->addPointCloud(target, tgt_h, "target cloud1", v2); 
//	viewer->addPointCloud(icp, transe, "icp cloud", v2);       
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
//	}
//}
//
//void icppoint2planells(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
//	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	cloud_with_normal(source, source_with_normals);
//	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	cloud_with_normal(target, target_with_normals);
//
//	// �㵽���ICP��׼
//	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
//
//	// ʹ��������С���˷����Ƶ㵽��ĸ���任
//	//pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr PointToPlane
//	//(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
//
//	pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr PointToPlane
//	(new pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
//
//	icp.setTransformationEstimation(PointToPlane);  // ���õ㵽��ľ������
//	icp.setInputSource(source_with_normals);        
//	icp.setInputTarget(target_with_normals);        
//	icp.setTransformationEpsilon(1e-10);            
//	icp.setMaxCorrespondenceDistance(0.1);           
//	icp.setMaximumIterations(200);                
//	pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
//	icp.align(*p_icp_cloud);  
//	pcl::io::savePCDFile("icppoint2planells3.pcd", *p_icp_cloud);
//	std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
//	std::cout << "transformation: " << icp.getFinalTransformation() << std::endl;
//
//}
//
//void icp(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//	icp.setInputSource(source);
//	icp.setInputTarget(target);
//	icp.setMaximumIterations(100);
//	icp.setTransformationEpsilon(1e-8);
//	icp.setMaxCorrespondenceDistance(0.1);
//	pcl::PointCloud<pcl::PointXYZ> Final;
//	icp.align(Final);
//	pcl::io::savePCDFile("icp.pcd", Final);
//	std::cout << "converged: " << icp.hasConverged() << "; score: " << icp.getFitnessScore() << std::endl;
//	std::cout << "transformation: " << std::endl << icp.getFinalTransformation() << std::endl;
//
//}
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
//	auto start_time = std::chrono::high_resolution_clock::now();
//
//	//icp(source, target);// �㵽���icp Elapsed time: 3.5356 seconds
//	icppoint2planells(source, target);
//	//ʹ��TransformationEstimationPointToPlaneLLS�ĵ㵽��icp Elapsed time : 0.390378 seconds  Ч���͵㵽���icp��࣬�����ٶȺܿ�
//	//Ĭ������ĵ㵽��icp Elapsed time: 0.383399 seconds Ҳ����˵Ĭ����ʹ��TransformationEstimationPointToPlaneLLS
//	//ʹ��TransformationEstimationSymmetricPointToPlaneLLS�ĵ㵽��icp Elapsed time: 0.48261 seconds   Ч����Ĭ�ϵ㵽��icp�ã����Ƕ���һ��
//
//	auto end_time = std::chrono::high_resolution_clock::now();
//	std::chrono::duration<double> elapsed = end_time - start_time;
//	std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
//
//	return (0);
//}
