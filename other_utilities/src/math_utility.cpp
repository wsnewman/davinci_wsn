#include <other_utilities/math_utility.h>

Eigen::Affine3d math_util::transformTFToEigen(const tf::Transform &t){
	Eigen::Affine3d e;
	for (int i = 0; i < 3; i++) {
		e.matrix()(i, 3) = t.getOrigin()[i];
        	for (int j = 0; j < 3; j++) {
            		e.matrix()(i, j) = t.getBasis()[i][j];
        	}
    	}
    	// Fill in identity in last row
    	for (int col = 0; col < 3; col++)
        	e.matrix()(3, col) = 0;
    	e.matrix()(3, 3) = 1;
    	return e;
}
