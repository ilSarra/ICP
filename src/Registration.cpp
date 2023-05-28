#include "Registration.h"


struct PointDistance
{ 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // This class should include an auto-differentiable cost function. 
  // To rotate a point given an axis-angle rotation, use
  // the Ceres function:
  // AngleAxisRotatePoint(...) (see ceres/rotation.h)
  // Similarly to the Bundle Adjustment case initialize the struct variables with the source and  the target point.
  // You have to optimize only the 6-dimensional array (rx, ry, rz, tx ,ty, tz).
  // WARNING: When dealing with the AutoDiffCostFunction template parameters,
  // pay attention to the order of the template parameters
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  PointDistance(Eigen::Vector3d s, Eigen::Vector3d t) : source(s), target(t) {}

  template<typename T>
  bool operator()(const T* const transformation, T* residuals) const {
    T p[3];
    p[0] = (T)(source.x);
    p[1] = (T)(source.y);
    p[2] = (T)(source.z);

    ceres::AngleAxisRotatePoint(transformation, p, p);

    p[0] += transformation[3];
    p[1] += transformation[4];
    p[2] += transformation[5];

    *residuals = sqrt(
      pow(p[0] - target[0], 2) +
      pow(p[1] - target[1], 2) +
      pow(p[2] - target[2], 2)
    )
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& source, Eigen::Vector3d& target) {
    return (new ceres::AutoDiffCostFunction<PointDistance, 1, 6>(new PointDistance(source, target)));
  }

  Eigen::Vector3d source;
  Eigen::Vector3d target;
};


Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename)
{
  open3d::io::ReadPointCloud(cloud_source_filename, source_ );
  open3d::io::ReadPointCloud(cloud_target_filename, target_ );
  Eigen::Vector3d gray_color;
  source_for_icp_ = source_;
}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target)
{
  source_ = cloud_source;
  target_ = cloud_target;
  source_for_icp_ = source_;
}


void Registration::draw_registration_result()
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  //different color
  Eigen::Vector3d color_s;
  Eigen::Vector3d color_t;
  color_s<<1, 0.706, 0;
  color_t<<0, 0.651, 0.929;

  target_clone.PaintUniformColor(color_t);
  source_clone.PaintUniformColor(color_s);
  source_clone.Transform(transformation_);

  auto src_pointer =  std::make_shared<open3d::geometry::PointCloud>(source_clone);
  auto target_pointer =  std::make_shared<open3d::geometry::PointCloud>(target_clone);
  open3d::visualization::DrawGeometries({src_pointer, target_pointer});
  return;
}



void Registration::execute_icp_registration(double threshold, int max_iteration, double relative_rmse, std::string mode)
{ 
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //ICP main loop
  //Check convergence criteria and the current iteration.
  //If mode=="svd" use get_svd_icp_transformation if mode=="lm" use get_lm_icp_transformation.
  //Remember to update transformation_ class variable, you can use source_for_icp_ to store transformed 3d points.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  int iter_ = 0;
  double prev_rmse_ = 0;

  while (iter_++ < max_iteration) {
    Eigen::Matrix4d cur_transformation;

    std::vector<size_t> source_indices;
    std::vector<size_t> target_indices;
    double cur_rmse;

    // C++11
    tie(source_indices, target_indices, cur_rmse) = Registration::find_closest_point(threshold);

    if (mode == "svd") {
      cur_transformation = Registration::get_svd_icp_transformation(source_indices, target_indices);
    }

    else if (mode == "lm") {
      cur_transformation = Registration::get_lm_icp_registration(source_indices, target_indices);
    }

    Registration::source_for_icp_ = Registration::source_for_icp_.transform(cur_transformation);
    Registration::transformation_ = cur_transformation * Registration::transformation_;
  }

  return;
}


std::tuple<std::vector<size_t>, std::vector<size_t>, double> Registration::find_closest_point(double threshold)
{ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Find source and target indices: for each source point find the closest one in the target and discard if their 
  //distance is bigger than threshold
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  std::vector<size_t> target_indices;
  std::vector<size_t> source_indices;
  Eigen::Vector3d source_point;
  double rmse;

  open3d::geometry::KDTreeFlann target_kdtree(target_);
  
  // For each point in source point cloud
  for (int i = 0; i < source_for_icp_.points_.size(); i++) {
    std::vector<double> cur_distances;
    std::vector<int> cur_indices;

    // Find closest point in target point cloud
    target_kdtree.SearchKNN<Eigen::Vector3d>(source_for_icp_.points_[i], 1, cur_indices, cur_distances);

    // Discard match points pair if distance is bigger than threshold
    if (cur_distances[i] <= threshold) {
      source_indices.push_back(i);
      target_indices.push_back(cur_indices[0]);
    }
  }

  rmse = Registration::compute_rmse();

  return {source_indices, target_indices, rmse};
}

Eigen::Matrix4d Registration::get_svd_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices){
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Find point clouds centroids and subtract them. 
  //Use SVD (Eigen::JacobiSVD<Eigen::MatrixXd>) to find best rotation and translation matrix.
  //Use source_indices and target_indices to extract point to compute the 3x3 matrix to be decomposed.
  //Remember to manage the special reflection case.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(4, 4);

  Eigen::Matrix3d C = Eigen::Matrix3d::Zero(3, 3);  // Cross-covariance matrix
  Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();

  // Compute centroids
  for (int i = 0; i < source_indices.size(); i++) {
    source_centroid += source_for_icp_.points[source_indices[i]];
    target_centroid += target_.points[target_indices[i]]
  }

  source_centroid /= source_indices.size();
  target_centroid /= target_indices.size();

  // Compute cross-covariance matrix C
  for (int i = 0; i < source_indices.size(); i++) {
    Eigen::Vector3d cur_source = source_for_icp_[source_indices[i]];
    Eigen::Vector3d cur_target = target_[target_indices[i]];
    
    Eigen::Matrix3d tmp_mul = (cur_source - source_centroid) * (cur_target - target_centroid).transpose();
    C += tmp_mul;
  }

  C /= source_indices.size();

  // Compute SVD
  Eigen::JacobiSVD<Eigen::Matrix3d,  Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV> svd(C);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Manage special reflection case
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity(3, 3);
  
  if (U.determinant() * V.determinant() == -1) {
    S(2,2) = -1;  // Use operator() to access matrix values since operator[] does not allow for more than one index
  }

  // Compute rotation matrix and translation vector
  Eigen::Matrix3d R = V * S * U.transpose();
  Eigen::Vector3d t = target_centroid - R * source_centroid;

  // Now insert the values in translation matrix
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      transformation(row, col) = R(row, col);
    }
  }

  for (int row = 0; row < 3; row++) {
    transformation(row, 3) = t[row];
  }

  return transformation;
}

Eigen::Matrix4d Registration::get_lm_icp_registration(std::vector<size_t> source_indices, std::vector<size_t> target_indices)
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Use LM (Ceres) to find best rotation and translation matrix. 
  //Remember to convert the euler angles in a rotation matrix, store it coupled with the final translation on:
  //Eigen::Matrix4d transformation.
  //The first three elements of std::vector<double> transformation_arr represent the euler angles, the last ones
  //the translation.
  //use source_indices and target_indices to extract point to compute the matrix to be decomposed.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(4,4);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = 4;
  options.max_num_iterations = 100;

  std::vector<double> transformation_arr(6, 0.0);
  int num_points = source_indices.size();

  // For each point....
  for( int i = 0; i < num_points; i++ )
  {
    
  }

  return transformation;
}


void Registration::set_transformation(Eigen::Matrix4d init_transformation)
{
  transformation_=init_transformation;
}


Eigen::Matrix4d  Registration::get_transformation()
{
  return transformation_;
}

double Registration::compute_rmse()
{
  open3d::geometry::KDTreeFlann target_kd_tree(target_);
  open3d::geometry::PointCloud source_clone = source_;
  source_clone.Transform(transformation_);
  int num_source_points  = source_clone.points_.size();
  Eigen::Vector3d source_point;
  std::vector<int> idx(1);
  std::vector<double> dist2(1);
  double mse;
  for(size_t i=0; i < num_source_points; ++i) {
    source_point = source_clone.points_[i];
    target_kd_tree.SearchKNN(source_point, 1, idx, dist2);
    mse = mse * i/(i+1) + dist2[0]/(i+1);
  }
  return sqrt(mse);
}

void Registration::write_tranformation_matrix(std::string filename)
{
  std::ofstream outfile (filename);
  if (outfile.is_open())
  {
    outfile << transformation_;
    outfile.close();
  }
}

void Registration::save_merged_cloud(std::string filename)
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  source_clone.Transform(transformation_);
  open3d::geometry::PointCloud merged = target_clone+source_clone;
  open3d::io::WritePointCloud(filename, merged );
}


