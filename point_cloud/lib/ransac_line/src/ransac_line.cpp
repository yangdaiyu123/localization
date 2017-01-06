#include"ransac_line.h"


bool Ransac_Line::isSampleGood(const std::vector<int> &samples) const
{
	if ((input_[samples[0]].x != input_[samples[1]].x)
		&&
		(input_[samples[0]].y != input_[samples[1]].y)
		&&
		(input_[samples[0]].z != input_[samples[1]].z))
		return (true);
	else
	return (false);
}


void Ransac_Line::getSamples(int &iterations,std::vector<int> &samples)
{

	// We're assuming that indices_ have already been set in the constructor
	if (indices_.size() <SampleSize)
	{
//		std::cout<<" Can not select"<<samples.size()<<" unique points out of "<<indices_.size()<<"!\n";
		// one of these will make it stop :)
		samples.clear();
		iterations = INT_MAX - 1;
		return;
	}

	// Get a second point which is different than the first
	samples.resize(SampleSize);
	for (unsigned int iter = 0; iter < max_sample_checks_; ++iter)
	{
		// Choose the random indices

		drawIndexSample(samples);


		// If it's a good sample, stop here
		if (isSampleGood(samples))
		{
//			std::cout << "Ransac_Line::getSamples] Selected" << samples.size() << "samples.\n";
			return;
		}
	}
//	std::cout << "[Ransac_Line::getSamples] WARNING: Could not select" << SampleSize << " sample points in" << max_sample_checks_<<" iterations!\n";
	samples.clear();

}




//////////////////////////////////////////////////////////////////////////
bool
Ransac_Line::computeModelCoefficients(
const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
	// Need 2 samples
	if (samples.size() != 2)
	{
//		std::cout << "[Ransac_Line::computeModelCoefficients] Invalid set of samples given" << samples.size()<<" \n";
		return (false);
	}

	if (fabs(input_[samples[0]].x - input_[samples[1]].x) <= std::numeric_limits<float>::epsilon() &&
		fabs(input_[samples[0]].y - input_[samples[1]].y) <= std::numeric_limits<float>::epsilon() &&
		fabs(input_[samples[0]].z - input_[samples[1]].z) <= std::numeric_limits<float>::epsilon())
	{
		return (false);
	}

	model_coefficients.resize(6);
	model_coefficients[0] = input_[samples[0]].x;
	model_coefficients[1] = input_[samples[0]].y;
	model_coefficients[2] = input_[samples[0]].z;

	model_coefficients[3] = input_[samples[1]].x - model_coefficients[0];
	model_coefficients[4] = input_[samples[1]].y - model_coefficients[1];
	model_coefficients[5] = input_[samples[1]].z - model_coefficients[2];

	model_coefficients.template tail<3>().normalize();
	return (true);
}


//////////////////////////////////////////////////////////////////////////
 int
Ransac_Line::countWithinDistance(
const Eigen::VectorXf &model_coefficients, const double threshold)
{
	// Needs a valid set of model coefficients
	if (!isModelValid(model_coefficients))
		return (0);

	double sqr_threshold = threshold * threshold;

	int nr_p = 0;

	// Obtain the line point and direction
	Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	line_dir.normalize();

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < indices_.size(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		double sqr_distance = (line_pt - input_[indices_[i]].getVector4fMap()).cross3(line_dir).squaredNorm();

		if (sqr_distance < sqr_threshold)
			nr_p++;
	}
	return (nr_p);
}


 void Ransac_Line::selectWithinDistance(
	 const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
 {
		 // Needs a valid set of model coefficients
		 if (!isModelValid(model_coefficients))
			 return;

		 double sqr_threshold = threshold * threshold;

		 int nr_p = 0;
		 inliers.resize(indices_.size());
		 error_sqr_dists_.resize(indices_.size());

		 // Obtain the line point and direction
		 Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
		 Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
		 line_dir.normalize();

		 // Iterate through the 3d points and calculate the distances from them to the line
		 for (size_t i = 0; i < indices_.size(); ++i)
		 {
			 // Calculate the distance from the point to the line
			 // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
			 double sqr_distance = (line_pt - input_[indices_[i]].getVector4fMap()).cross3(line_dir).squaredNorm();

			 if (sqr_distance < sqr_threshold)
			 {
				 // Returns the indices of the points whose squared distances are smaller than the threshold
				 inliers[nr_p] = indices_[i];
				 error_sqr_dists_[nr_p] = sqr_distance;
				 ++nr_p;
			 }
		 }
		 inliers.resize(nr_p);
		 error_sqr_dists_.resize(nr_p);
	 }





bool Ransac_Line::computeModel()
{

	// Warn and exit if no threshold was set
	if (threshold_ == std::numeric_limits<double>::max())
	{
//		std::cout<<"Ransac_Line::computeModel] No threshold set!\n";
		return (false);
	}

	iterations_ = 0;
	int n_best_inliers_count = -INT_MAX;
	double k = 1.0;

	std::vector<int> selection;
	Eigen::VectorXf model_coefficients;

	double log_probability = log(1.0 - probability_);
	double one_over_indices = 1.0 / static_cast<double> (getIndices().size());

	int n_inliers_count = 0;
	unsigned skipped_count = 0;
	// supress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
	const unsigned max_skip = max_iterations_ * 10;

	// Iterate
	while (iterations_ < k && skipped_count < max_skip)
	{
		// Get X samples which satisfy the model criteria
		getSamples(iterations_, selection);

		if (selection.empty())
		{
//			std::cout<<"[Ransac_Line::computeModel] No samples could be selected!\n";
			break;
		}

		// Search for inliers in the point cloud for the current plane model M
		if (!computeModelCoefficients(selection, model_coefficients))
		{
			//++iterations_;
			++skipped_count;
			continue;
		}

		// Select the inliers that are within threshold_ from the model
		//sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
		//if (inliers.empty () && k > 1.0)
		//  continue;

		n_inliers_count = countWithinDistance(model_coefficients, threshold_);

		// Better match ?
		if (n_inliers_count > n_best_inliers_count)
		{
			n_best_inliers_count = n_inliers_count;

			// Save the current model/inlier/coefficients selection as being the best so far
			model_ = selection;
			model_coefficients_ = model_coefficients;

			// Compute the k parameter (k=log(z)/log(1-w^n))
			double w = static_cast<double> (n_best_inliers_count)* one_over_indices;
			double p_no_outliers = 1.0 - pow(w, static_cast<double> (selection.size()));
			p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
			p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
			k = log_probability / log(p_no_outliers);
		}

		++iterations_;
//		std::cout << "[Ransac_Line::computeModel] Trial" << iterations_ << " out of" << k << ":" << n_inliers_count << "inliers (best is:"<<n_best_inliers_count<<"so far.n";
		if (iterations_ > max_iterations_)
		{
//			std::cout<<"[Ransac_Line::computeModel] RANSAC reached the maximum number of trials.\n";
			break;
		}
	}

//	std::cout << "[pcl::RandomSampleConsensus::computeModel] Model:" << model_.size()<< "size," << n_best_inliers_count<<" inliers.\n";

	if (model_.empty())
	{
		inliers_.clear();
		return (false);
	}

	// Get the set of inliers that correspond to the best model found so far
     selectWithinDistance(model_coefficients_, threshold_, inliers_);
	 return (true);

}
