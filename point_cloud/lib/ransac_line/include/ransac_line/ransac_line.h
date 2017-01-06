#ifndef RANSAC_LINE_H_
#define RANSAC_LINE__H_


#include <iostream>
#include <vector>
#include <algorithm>
#include <set>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>


using namespace std;
using namespace pcl;


#define SampleSize 2

class Ransac_Line{
private:


	std::vector<PointXYZ> input_;  //输入的待拟合点云
	std::vector<int> indices_;
	std::vector<int> shuffled_indices_;
	int max_iterations_;  //最大迭代次数
	/** \brief Total number of internal loop iterations that we've done so far. */
	int iterations_;
	/** \brief Desired probability of choosing at least one sample free from outliers. */
	double probability_;
	/** The maximum number of samples to try until we get a good one */
	static const unsigned int max_sample_checks_ = 1000;
	double threshold_; //点到直线的距离阈值
	/** \brief The model found after the last computeModel () as point cloud indices. */
	std::vector<int> model_;
	/** \brief The coefficients of our model computed directly from the model found. */
	Eigen::VectorXf model_coefficients_;
	/** \brief The indices of the points that were chosen as inliers after the last computeModel () call. */
      std::vector<int> inliers_;
	  /** \brief A vector holding the distances to the computed model. Used internally. */
	  std::vector<double> error_sqr_dists_;

public:
	//构造函数
	Ransac_Line() :input_(),indices_(),shuffled_indices_(),max_iterations_(1000),iterations_(0),probability_(0.99), threshold_(std::numeric_limits<double>::max()){};

	Ransac_Line(const std::vector<PointXYZ> &cloud, double threshold, int max_iterations)
		:input_(),
		indices_(),
		max_iterations_(max_iterations),
		threshold_(threshold),
		iterations_(0),
		probability_(0.99)

	{
		setInputCloud(cloud);
	}

	Ransac_Line(const std::vector<PointXYZ> &cloud, const std::vector<int>&indices, double threshold, int max_iterations)
		:input_(cloud),
		indices_(indices),
		max_iterations_(max_iterations),
		threshold_(threshold),
	     iterations_(0),
	   probability_(0.99)

	{
		if (indices_.size() > input_.size())
		{
			std::cout << "Invalid index vector given with size" << indices_.size() << " while the input PointCloud has size" << input_.size() << "!\n";
			indices_.clear();
		}
		shuffled_indices_ = indices_;



	}

	/** \brief Set the distance to model threshold.
	* \param[in] threshold distance to model threshold
	*/
	inline void
		setDistanceThreshold(double threshold)  { threshold_ = threshold; }

	/** \brief Get the distance to model threshold, as set by the user. */
	inline double
		getDistanceThreshold() { return (threshold_); }

	/** \brief Set the maximum number of iterations.
	* \param[in] max_iterations maximum number of iterations
	*/
	inline void
		setMaxIterations(int max_iterations) { max_iterations_ = max_iterations; }

	/** \brief Get the maximum number of iterations, as set by the user. */
	inline int
		getMaxIterations() { return (max_iterations_); }



	inline void
		getRandomSamples( std::vector<int>  &indices,
		size_t nr_samples,
		std::set<int> &indices_subset)
	{
			indices_subset.clear();
			while (indices_subset.size() < nr_samples)
				//indices_subset.insert ((*indices)[(int) (indices->size () * (rand () / (RAND_MAX + 1.0)))]);
				indices_subset.insert(indices[static_cast<int> (static_cast<double>(indices.size()) * (rand() / (RAND_MAX + 1.0)))]);
		}


	/** \brief Return the best model found so far.
	* \param[out] model the resultant model
	*/
	inline void
		getModel(std::vector<int> &model) { model = model_; }

	/** \brief Return the best set of inliers found so far for this model.
	* \param[out] inliers the resultant set of inliers
	*/
	inline void
		getInliers(std::vector<int> &inliers) { inliers = inliers_; }

	/** \brief Return the model coefficients of the best model found so far.
	* \param[out] model_coefficients the resultant model coefficients, as documented in \ref sample_consensus
	*/
	inline void
		getModelCoefficients(Eigen::VectorXf &model_coefficients) { model_coefficients = model_coefficients_; }

	/** \brief Get a pointer to the vector of indices used. */
	inline std::vector<int>
		getIndices() const { return (indices_); }


	inline	void setInputCloud(const std::vector<PointXYZ> &cloud)//输入点云
	{
		input_ = cloud;

		if (indices_.empty())
		{
			// Prepare a set of indices to be used (entire cloud)
			indices_.resize(cloud.size());
			for (size_t i = 0; i < cloud.size(); ++i)
				indices_[i] = static_cast<int> (i);
		}
		shuffled_indices_ = indices_;
	}

	inline void
		drawIndexSample(std::vector<int> &sample)
	{
			size_t sample_size = sample.size();
			size_t index_size = shuffled_indices_.size();
			for (unsigned int i = 0; i < sample_size; ++i)
				// The 1/(RAND_MAX+1.0) trick is when the random numbers are not uniformly distributed and for small modulo
				// elements, that does not matter (and nowadays, random number generators are good)
				//std::swap (shuffled_indices_[i], shuffled_indices_[i + (rand () % (index_size - i))]);
		     std::swap(shuffled_indices_[i], shuffled_indices_[i + (rand() % (index_size - i))]);

			std::copy(shuffled_indices_.begin(), shuffled_indices_.begin() + sample_size, sample.begin());
		}





	void getSamples(int &iterations,std::vector<int> &samples);

	bool isSampleGood(const std::vector<int> &samples) const;//判断选取的采样点是否合理，对于直线模型来说就是验证两点是否是同一点

	bool computeModelCoefficients(const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);

    int countWithinDistance(const Eigen::VectorXf &model_coefficients,const double threshold);

	void selectWithinDistance(const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers);

	bool computeModel();




	inline bool
		isModelValid(const Eigen::VectorXf &model_coefficients)
	{
			if (model_coefficients.size() != 6)
			{
				std::cout<<"[Ransac_LIne::selectWithinDistance] Invalid number of model coefficients given"<< model_coefficients.size()<<"!\n";
				return (false);
			}

			return (true);
		}




};

#endif 
