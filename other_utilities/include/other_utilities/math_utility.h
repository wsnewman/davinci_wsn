#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf/tf.h>

#pragma once

namespace math_util{
	Eigen::Affine3d transformTFToEigen(const tf::Transform &t);
}
