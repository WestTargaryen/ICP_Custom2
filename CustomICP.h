#pragma once
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <Eigen/Dense>
#include <omp.h> // 并行支持
#include <pcl/features/normal_3d.h> // PCL法线计算头文件
#include <pcl/common/angles.h>

// 收敛标准类
class ConvergenceCriteria {
public:
	enum ConvergenceState {
		NOT_CONVERGED,
		ITERATIONS,
		TRANSFORM,
		ABS_MSE,
		REL_MSE,
		FAILURE_AFTER_MAX_ITERATIONS
	};

	ConvergenceCriteria()
		: max_iterations_(50)
		, rotation_threshold_(0.99998)
		, translation_threshold_(1e-8)
		, mse_threshold_absolute_(1e-6)
		, mse_threshold_relative_(1e-6)
		, max_iterations_similar_transforms_(5)
		, failure_after_max_iter_(false)
		, iterations_(0)
		, iterations_similar_transforms_(0)
		, correspondences_prev_mse_(std::numeric_limits<double>::max())
		, convergence_state_(NOT_CONVERGED) {



	}

	void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
	void setRotationThreshold(double threshold) { rotation_threshold_ = threshold; }
	void setTranslationThreshold(double threshold) { translation_threshold_ = threshold; }
	void setMSEThresholdAbsolute(double threshold) { mse_threshold_absolute_ = threshold; }
	void setMSEThresholdRelative(double threshold) { mse_threshold_relative_ = threshold; }
	void setMaxIterationsSimilarTransforms(int iterations) { max_iterations_similar_transforms_ = iterations; }
	void setFailureAfterMaxIterations(bool failure) { failure_after_max_iter_ = failure; }

	bool hasConverged(const Eigen::Matrix4f& transformation, const pcl::CorrespondencesPtr& correspondences) {
		if (convergence_state_ != NOT_CONVERGED) {
			iterations_similar_transforms_ = 0;
			convergence_state_ = NOT_CONVERGED;
		}

		bool is_similar = false;
		iterations_++;

		// 1. 迭代次数检查
		if (iterations_ >= max_iterations_) {
			if (!failure_after_max_iter_) {
				convergence_state_ = ITERATIONS;
				return true;
			}
			convergence_state_ = FAILURE_AFTER_MAX_ITERATIONS;
		}

		// 2. 变换矩阵检查
		double cos_angle = 0.5 * (transformation.coeff(0, 0) + transformation.coeff(1, 1) +
			transformation.coeff(2, 2) - 1);
		double translation_sqr = transformation.coeff(0, 3) * transformation.coeff(0, 3) +
			transformation.coeff(1, 3) * transformation.coeff(1, 3) +
			transformation.coeff(2, 3) * transformation.coeff(2, 3);

		if (cos_angle >= rotation_threshold_ && translation_sqr <= translation_threshold_) {
			if (iterations_similar_transforms_ >= max_iterations_similar_transforms_) {
				convergence_state_ = TRANSFORM;
				return true;
			}
			is_similar = true;
		}

		// 3. 均方误差检查
		double correspondences_cur_mse = calculateMSE(correspondences);

		if (std::numeric_limits<double>::max() == correspondences_prev_mse_) {
			std::cout << "[ConvergenceCriteria] Previous / Current MSE for correspondences distances is: INIT / "
				<< correspondences_cur_mse << std::endl;
		}
		else {
			std::cout << "[ConvergenceCriteria] Previous / Current MSE for correspondences distances is: "
				<< correspondences_prev_mse_ << " / " << correspondences_cur_mse << std::endl;
		}
		if (std::abs(correspondences_cur_mse - correspondences_prev_mse_) < mse_threshold_absolute_) {
			if (iterations_similar_transforms_ >= max_iterations_similar_transforms_) {
				convergence_state_ = ABS_MSE;
				return true;
			}
			is_similar = true;
		}
		if (correspondences_prev_mse_ > 0 &&
			std::abs(correspondences_cur_mse - correspondences_prev_mse_) / correspondences_prev_mse_ < mse_threshold_relative_) {
			if (iterations_similar_transforms_ >= max_iterations_similar_transforms_) {
				convergence_state_ = REL_MSE;
				return true;
			}
			is_similar = true;
		}

		if (is_similar) {
			iterations_similar_transforms_++;
		}
		else {
			iterations_similar_transforms_ = 0;
		}
		correspondences_prev_mse_ = correspondences_cur_mse;
		return false;
	}

	int getIterations() const { return iterations_; }

private:
	int max_iterations_;
	double rotation_threshold_;
	double translation_threshold_;
	double mse_threshold_absolute_;
	double mse_threshold_relative_;
	int max_iterations_similar_transforms_;
	bool failure_after_max_iter_;

	int iterations_;
	int iterations_similar_transforms_;
	double correspondences_prev_mse_;
	ConvergenceState convergence_state_;

	double calculateMSE(const pcl::CorrespondencesPtr& correspondences) {
		if (correspondences->empty()) return 0.0;

		double sum = 0.0;
		for (const auto& corr : *correspondences) {
			sum += corr.distance * corr.distance;
		}
		return sum / static_cast<double>(correspondences->size());
	}
};

class CustomICP {
public:
	CustomICP() {
		convergence_criteria_.setRotationThreshold(0.99998); // 约0.8°
		convergence_criteria_.setTranslationThreshold(1e-8);
		convergence_criteria_.setMSEThresholdAbsolute(1e-8);
		convergence_criteria_.setMSEThresholdRelative(1e-8);
		convergence_criteria_.setMaxIterationsSimilarTransforms(5);
	}

	void setInputSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source) {
		source_ = source;
		source_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		ne.setInputCloud(source_);
		ne.setSearchMethod(tree);
		ne.setKSearch(50);
		ne.compute(*source_normals_);
		transformed_source_normals_ = source_normals_;
	}
	void setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
		target_ = target;
		target_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		ne.setInputCloud(target_);
		ne.setSearchMethod(tree);
		ne.setKSearch(50);
		ne.compute(*target_normals_);
	}
	void setMaximumIterations(int max_iter) {
		max_iterations_ = max_iter;
		convergence_criteria_.setMaxIterations(max_iter);
	}
	void setMaxCorrespondenceDistance(double dist) { max_correspondence_distance_ = dist; }
	void setReciprocal(bool enable) { is_reciprocal_ = enable; }

	// 收敛标准设置函数
	void setRotationThreshold(double threshold) { convergence_criteria_.setRotationThreshold(threshold); }
	void setTranslationThreshold(double threshold) { convergence_criteria_.setTranslationThreshold(threshold); }
	void setMSEThresholdAbsolute(double threshold) { convergence_criteria_.setMSEThresholdAbsolute(threshold); }
	void setMSEThresholdRelative(double threshold) { convergence_criteria_.setMSEThresholdRelative(threshold); }
	void setMaxIterationsSimilarTransforms(int iterations) { convergence_criteria_.setMaxIterationsSimilarTransforms(iterations); }

	void align(pcl::PointCloud<pcl::PointXYZ>& output) {
		final_transformation_ = Eigen::Matrix4f::Identity();
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_transformed(new pcl::PointCloud<pcl::PointXYZ>);
		*source_transformed = *source_;

		pcl::KdTreeFLANN<pcl::PointXYZ> target_tree, source_tree;
		target_tree.setInputCloud(target_);

		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f delta_transformation;
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

		do {
			// 1、估计点对
			findCorrespondences(source_transformed, &target_tree, correspondences, &source_tree);
			std::cout << "Iteration " << convergence_criteria_.getIterations()
				<< ", correspondences found: " << correspondences->size() << std::endl;

			if (correspondences->size() < min_number_correspondences_) {
				std::cerr << "Not enough correspondences found: " << correspondences->size() << ", needs at least " << min_number_correspondences_ << std::endl;
				break;
			}

			// 2、对应点对筛选
			pcl::CorrespondencesPtr filtered_correspondences(new pcl::Correspondences);
			filterCorrespondencesByNormal(correspondences, filtered_correspondences);
			std::cout << "After filtering, correspondences left: " << filtered_correspondences->size() << std::endl;

			if (filtered_correspondences->size() < min_number_correspondences_) {
				std::cerr << "Not enough correspondences after filtering: " << filtered_correspondences->size() << std::endl;
				break;
			}
			correspondences_ = filtered_correspondences;

			// 3、变换估计
			estimateTransformation(source_transformed, target_, filtered_correspondences, delta_transformation);

			transformation = delta_transformation * transformation;
			final_transformation_ = transformation;

			transformPointCloud(source_transformed, source_transformed, delta_transformation);
			transformNormals(delta_transformation);

		} while (!convergence_criteria_.hasConverged(delta_transformation, correspondences_));

		output = *source_transformed;
		std::cout << "ICP converged after " << convergence_criteria_.getIterations() << std::endl;
	}

	Eigen::Matrix4f getFinalTransformation() const { return final_transformation_; }

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_ = nullptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_ = nullptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals_ = nullptr;
	pcl::PointCloud<pcl::Normal>::Ptr transformed_source_normals_ = nullptr;
	pcl::PointCloud<pcl::Normal>::Ptr target_normals_ = nullptr;

	int max_iterations_ = 50;
	double max_correspondence_distance_ = 0.1;
	bool is_reciprocal_ = true;
	int min_number_correspondences_ = 5;
	Eigen::Matrix4f final_transformation_ = Eigen::Matrix4f::Identity();
	pcl::CorrespondencesPtr correspondences_;
	ConvergenceCriteria convergence_criteria_;

	// 变换法线方向
	void transformNormals(const Eigen::Matrix4f& transform) {
		if (!transformed_source_normals_) return;
		Eigen::Matrix3f R = transform.block<3, 3>(0, 0); 

#pragma omp parallel for
		for (size_t i = 0; i < transformed_source_normals_->size(); ++i) {
			Eigen::Vector3f n = transformed_source_normals_->points[i].getNormalVector3fMap();
			n = R * n; 
			n.normalize();
			transformed_source_normals_->points[i].getNormalVector3fMap() = n;
		}
	}

	// 查找对应点（支持单向/双向匹配）
	void findCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
		pcl::KdTreeFLANN<pcl::PointXYZ>* target_tree,
		pcl::CorrespondencesPtr correspondences,
		pcl::KdTreeFLANN<pcl::PointXYZ>* source_tree = nullptr) {

		correspondences->clear();
		if (source->empty() || target_->empty()) return;

		if (is_reciprocal_) {
			source_tree = new pcl::KdTreeFLANN<pcl::PointXYZ>();
			source_tree->setInputCloud(source);
		}

		const int num_threads = omp_get_max_threads();
		std::vector<std::vector<pcl::Correspondence>> thread_correspondences(num_threads);

		for (auto& corr : thread_correspondences) {
			corr.reserve(source->size() / num_threads);
		}

		const float max_dist_sqr = max_correspondence_distance_ * max_correspondence_distance_;

#pragma omp parallel for default(none) \
    shared(source, target_tree, source_tree, thread_correspondences, max_dist_sqr)
		for (int i = 0; i < static_cast<int>(source->size()); ++i) {
			const int thread_id = omp_get_thread_num();
			const auto& pt_src = source->points[i];

			std::vector<int> nn_indices(1);
			std::vector<float> nn_dists(1);
			if (target_tree->nearestKSearch(pt_src, 1, nn_indices, nn_dists) != 1)
				continue;

			if (nn_dists[0] > max_dist_sqr)
				continue;

			int target_idx = nn_indices[0];

			// 双向匹配验证
			if (is_reciprocal_) {
				const auto& pt_tgt = target_->points[target_idx];
				std::vector<int> src_reciprocal(1);
				std::vector<float> dist_reciprocal(1);

				if (source_tree->nearestKSearch(pt_tgt, 1, src_reciprocal, dist_reciprocal) != 1)
					continue;

				if (src_reciprocal[0] != i || dist_reciprocal[0] > max_dist_sqr)
					continue;
			}

			pcl::Correspondence corr;
			corr.index_query = i;
			corr.index_match = target_idx;
			corr.distance = std::sqrt(nn_dists[0]);
			thread_correspondences[thread_id].push_back(corr);
		}

		for (int i = 0; i < num_threads; ++i) {
			correspondences->insert(correspondences->end(),
				thread_correspondences[i].begin(),
				thread_correspondences[i].end()
			);
		}

		std::sort(correspondences->begin(), correspondences->end(),
			[](const pcl::Correspondence& a, const pcl::Correspondence& b) {
				return a.index_query < b.index_query;
			});
	}

	// 基于法线的对应点筛选
	void filterCorrespondencesByNormal(
		const pcl::CorrespondencesPtr& correspondences,
		pcl::CorrespondencesPtr& filtered, double angle_threshold = pcl::deg2rad(10.0)) {
		filtered->clear();
		if (!correspondences || !transformed_source_normals_ || !target_normals_) return;

		const float cos_threshold = std::cos(angle_threshold);
		const size_t num_threads = omp_get_max_threads();
		std::vector<std::vector<pcl::Correspondence>> thread_filtered(num_threads);

#pragma omp parallel for
		for (size_t i = 0; i < correspondences->size(); ++i) {
			const auto& corr = correspondences->at(i);
			int src_idx = corr.index_query;
			int tgt_idx = corr.index_match;

			if (src_idx >= transformed_source_normals_->size() || tgt_idx >= target_normals_->size())
				continue;

			Eigen::Vector3f n1 = transformed_source_normals_->points[src_idx].getNormalVector3fMap();
			Eigen::Vector3f n2 = target_normals_->points[tgt_idx].getNormalVector3fMap();
			float dot = std::abs(n1.dot(n2)); 

			if (dot >= cos_threshold) {
				thread_filtered[omp_get_thread_num()].push_back(corr);
			}
		}

		for (const auto& thread_corr : thread_filtered) {
			filtered->insert(filtered->end(), thread_corr.begin(), thread_corr.end());
		}

		PCL_INFO("[filterCorrespondencesByNormal] Filtered %zu -> %zu correspondences\n",
			correspondences->size(), filtered->size());
	}

	// 重载函数，接收pcl::Correspondences格式的对应关系
	void estimateTransformation(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
		const pcl::CorrespondencesPtr& correspondences,
		Eigen::Matrix4f& transformation) {

		// 从pcl::Correspondences中提取索引
		std::vector<int> src_indices, tgt_indices;
		src_indices.reserve(correspondences->size());
		tgt_indices.reserve(correspondences->size());

		for (const auto& corr : *correspondences) {
			src_indices.push_back(corr.index_query);
			tgt_indices.push_back(corr.index_match);
		}

		// 调用原函数进行变换估计
		estimateTransformation(source, target, src_indices, tgt_indices, transformation);
	}

	// 变换矩阵估计（SVD）
	void estimateTransformation(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
		const std::vector<int>& src_indices,
		const std::vector<int>& tgt_indices,
		Eigen::Matrix4f& transformation) {
		size_t n = src_indices.size();
		if (n < 3) return;

		// 质心计算
		Eigen::Vector3f centroid_src = Eigen::Vector3f::Zero();
		Eigen::Vector3f centroid_tgt = Eigen::Vector3f::Zero();

		for (size_t i = 0; i < n; ++i) {
			centroid_src += source->points[src_indices[i]].getVector3fMap();
			centroid_tgt += target->points[tgt_indices[i]].getVector3fMap();
		}
		centroid_src /= n;
		centroid_tgt /= n;

		// 去质心化点云
		Eigen::MatrixXf src_centered(3, n), tgt_centered(3, n);
		for (size_t i = 0; i < n; ++i) {
			src_centered.col(i) = source->points[src_indices[i]].getVector3fMap() - centroid_src;
			tgt_centered.col(i) = target->points[tgt_indices[i]].getVector3fMap() - centroid_tgt;
		}

		// 协方差矩阵计算
		Eigen::Matrix3f H = src_centered * tgt_centered.transpose();

		// SVD分解
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU(), V = svd.matrixV();

		// 处理反射矩阵
		if (U.determinant() * V.determinant() < 0) {
			V.col(2) *= -1;
		}

		// 构建旋转矩阵
		Eigen::Matrix3f R = V * U.transpose();
		Eigen::Vector3f t = centroid_tgt - R * centroid_src;

		// 组合变换矩阵
		transformation.setIdentity();
		transformation.block<3, 3>(0, 0) = R;
		transformation.block<3, 1>(0, 3) = t;
	}

	// 点云变换
	void transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		const Eigen::Matrix4f& transformation) {
		output->resize(input->size());

#pragma omp parallel for
		for (size_t i = 0; i < input->size(); ++i) {
			// 坐标变换
			Eigen::Vector4f pt(input->points[i].x, input->points[i].y, input->points[i].z, 1.0);
			pt = transformation * pt;
			output->points[i].x = pt.x();
			output->points[i].y = pt.y();
			output->points[i].z = pt.z();
		}
	}
};