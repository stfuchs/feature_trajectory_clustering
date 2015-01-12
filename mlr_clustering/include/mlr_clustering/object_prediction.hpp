/**
 * @file   object_prediction.hpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Mon Jan 12 17:27:04 2015
 * 
 * @brief  
 * 
 * 
 */

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/ml/ml.hpp>

#include "mlr_common/kernel.hpp"

template<typename... Ts>
struct ObjectPrediction
{
  typedef type_array<Ts...> traj_types;
  typedef type_array<typename Ts::IdT...> id_types;
  typedef MultiType<std::vector,id_types> VecIds;

  void predict(Eigen::MatrixXf const& K, VecIds const& ids)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(K);
    if (eigensolver.info() != Eigen::Success)
    {
      std::cout << "Eigendecomposition of Kernel failed!" << std::endl;
      return;
    }
    const cv::Mat K_map(K.rows(), K.cols(), CV_32FC1, K.data());
    cv::Mat log_likelihoods(K.rows(), 1, CV_64FC1);
    cv::Mat labels(K.rows(), 1, CV_32SC1);
    cv::EM gmm(6);
    gmm.train(K_map,log_likelihoods,labels);
  }
  
};
