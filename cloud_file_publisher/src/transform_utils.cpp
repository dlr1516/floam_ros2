#include "transform_utils.h"

namespace rimlab_kitti {

	bool readTransformLine(std::istream &in, Transform3 &transform) {
		std::array<float, 12> values;
		if (in >> values[0] >> values[1] >> values[2] >> values[3] >> values[4] >> values[5] >> values[6] >> values[7] >> values[8] >> values[9] >> values[10]
																																						   >> values[11]) {
			transform.matrix() << values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10], values[11];
			//std::cout << "transformation matrix:\n" << transform.matrix() << std::endl;
			return true;
		} else {
			std::cerr << "Cannot read transform matrix from line: 12 floats required" << std::endl;
			return false;
		}
	}

	bool readTimeTransformLine(std::istream &in, float &time, Transform3 &transform) {
		if ((in >> time) && readTransformLine(in, transform)) {
			return true;
		} else {
			std::cerr << "Cannot read line: invalid time stamp or transformation matrix" << std::endl;
			return false;
		}
	}

	bool readTimePosQuatLine(std::istream &in, float &time, Transform3 &transform) {
		float x, y, z, qx, qy, qz, qw;
		if (in >> time >> x >> y >> z >> qx >> qy >> qz >> qw) {
			Eigen::Quaternionf q(qw, qx, qy, qz);
			Eigen::Translation3f v(x, y, z);
			transform = v * q;
			return true;
		} else {
			std::cerr << "Cannot read line in format: time x y z qx qy qz qw" << std::endl;
			return false;
		}
	}

	bool readCalibLine(std::istream &in, std::string &label, Eigen::Affine3f &transform) {
		if ((in >> label) && readTransformLine(in, transform)) {
			return true;
		} else {
			std::cerr << "Cannot read line: invalid calibration data label or transformation matrix" << std::endl;
			return false;
		}
	}

	bool readTransformFile(const std::string &filename, VectorTransform3 &transforms) {
		Transform3 tr;
		std::ifstream file(filename);
		if (!file) {
			std::cerr << "Cannot open transformation file \"" << filename << "\"" << std::endl;
			return false;
		}
		while (readTransformLine(file, tr)) {
			transforms.push_back(tr);
		}
		file.close();
		return true;
	}

	bool readTimeTransformFile(const std::string &filename, std::vector<float> &times, VectorTransform3 &transforms) {
		Transform3 tr;
		float t;
		std::ifstream file(filename);
		if (!file) {
			std::cerr << "Cannot open transformation file \"" << filename << "\"" << std::endl;
			return false;
		}
		while (readTimeTransformLine(file, t, tr)) {
			times.push_back(t);
			transforms.push_back(tr);
		}
		file.close();
		return true;
	}

	bool readTimePosQuatFile(const std::string &filename, std::vector<float> &times, VectorTransform3 &transforms) {
		Transform3 tr;
		float t;
		std::ifstream file(filename);
		if (!file) {
			std::cerr << "Cannot open transformation file \"" << filename << "\"" << std::endl;
			return false;
		}
		while (readTimePosQuatLine(file, t, tr)) {
			times.push_back(t);
			transforms.push_back(tr);
		}
		file.close();
		return true;
	}
	
	void writePoseQuat(std::ostream& out, const float& t, const Eigen::Affine3f& pose) {
        	Eigen::Vector3f transl = pose.translation();
        	Eigen::Quaternionf quat(pose.rotation()); // Eigen coefficients order [x, y, z, w]
        	out << t
                << " " << transl(0) << " " << transl(1) << " " << transl(2)
                << " " << quat.coeffs()(0) << " " << quat.coeffs()(1) << " " << quat.coeffs()(2) << " " << quat.coeffs()(3)
                << "\n";
    	}

    	void writePoseMat(std::ostream& out, const Eigen::Affine3f& pose) {
        	out
                << pose.matrix()(0, 0) << " " << pose.matrix()(0, 1) << " " << pose.matrix()(0, 2) << " " << pose.matrix()(0, 3) << " "
                << pose.matrix()(1, 0) << " " << pose.matrix()(1, 1) << " " << pose.matrix()(1, 2) << " " << pose.matrix()(1, 3) << " "
                << pose.matrix()(2, 0) << " " << pose.matrix()(2, 1) << " " << pose.matrix()(2, 2) << " " << pose.matrix()(2, 3) << " "
                << "\n";
    	}

	void computeDistances(const VectorTransform3 &transforms, std::vector<float> &distances) {
		if (transforms.empty()) {
			return;
		}
		distances.resize(transforms.size());
		distances[0] = 0.0f;
		for (int i = 1; i < (int) distances.size(); ++i) {
			distances[i] = distances[i - 1] + (transforms[i - 1].inverse() * transforms[i]).translation().norm();
		}
	}

	int findDistanceIdx(const std::vector<float> &distances, float len, float& factor) {
		auto it = lower_bound(distances.begin(), distances.end(), len);
		factor = 0.0;
		if (it == distances.end()) {
			factor = 0.0;
			return distances.size();
		}
		else if (it == distances.begin()) {
			factor = 0.0;
			return 0;
		}
		auto itPrev = it - 1;
		factor = (len - *itPrev) / (*it - *itPrev);
//		std::cout << "len " << len << ": samples "
//				<< (int)std::distance(distances.begin(), itPrev) << " at distance " << *itPrev << " and "
//				<< (int)std::distance(distances.begin(), it) << " at distance " << *it
//				<< " factor " << factor << ": distPrev * (1 - " << factor << ") + distCurr * " << factor << " = "
//				<< (*itPrev * (1- factor) + *it * factor)
//				<< std::endl;
		return (int) std::distance(distances.begin(), itPrev);
	}

	int findDistanceIncrIdx(const std::vector<float> &distances, int startIdx, float lenStep, float& factor) {
		if (startIdx < 0 || startIdx >= (int)distances.size()) {
			return distances.size();
		}
		float len = distances[startIdx] + lenStep;
		auto it = lower_bound(distances.begin() + startIdx, distances.end(), len);
		factor = 0.0;
		if (it != distances.begin()) {
			auto itPrev = it - 1;
			factor = (*it - len) / (*it - *itPrev);
		}
		return (int) std::distance(distances.begin(), it);
	}

	float computeTranslationNorm(const Transform3 &transformDelta) {
		return transformDelta.translation().norm();
	}

	float computeRotationAngle(const Transform3 &transformDelta) {
		float m00 = transformDelta.matrix()(0, 0);
		float m11 = transformDelta.matrix()(1, 1);
		float m22 = transformDelta.matrix()(2, 2);
		float delta = 0.5f * (m00 + m11 + m22 - 1.0f);
		return acos(std::max(std::min(delta, 1.0f), -1.0f));
	}

	void interpolateTransform(const Transform3& transf0,const Transform3& transf1,float f,Transform3& transfInterp) {
		if (f < 0.0f || f > 1.0f) {
			std::cerr << __FILE__ << "," << __LINE__ << ": invalid interpolation factor " << f << ": must be between 0.0 and 1.0" << std::endl;
			return;
		}
		Quaternion rot0(transf0.linear());
		Quaternion rot1(transf1.linear());
		Vector3 transl0 = transf0.translation();
		Vector3 transl1 = transf1.translation();
		transfInterp.translation() = (1.0-f) * transl0 + f * transl1;
		transfInterp.linear() = rot0.slerp(f, rot1).toRotationMatrix();
	}

	void computeErrors(const VectorTransform3 &transformsRes, const std::vector<float> &distancesRes, const VectorTransform3 &transformsGt,
			const std::vector<float> &distancesGt, float lenSeg, float lenStep, VectorErrorData &errors) {
		ErrorData err;
		Transform3 transformDeltaRes, transformDeltaGt, transformError;
		int numRes, numGt, firstIdxGt, lastIdxGt, firstIdxRes, lastIdxRes;
		float factorRes, factorGt;
		//float len;

		if (transformsRes.size() != distancesRes.size() || transformsGt.size() != distancesGt.size()) {
			std::cout << "Error: different size of distances and trasforms" << std::endl;
			return;
		}
		numRes = transformsRes.size();
		numGt = transformsGt.size();

		firstIdxRes = 0;
		firstIdxGt = 0;
		//len = 0.0f;
		while (firstIdxRes < numRes && firstIdxGt < numGt) {
			//len += lenIncr;
			//lastIdxRes = findDistanceIdx(distancesRes, lenSeg);
			//lastIdxGt = findDistanceIdx(distancesGt, lenSeg);
			lastIdxRes = findDistanceIncrIdx(distancesRes, firstIdxRes, lenSeg, factorRes);
			lastIdxGt = findDistanceIncrIdx(distancesGt, firstIdxGt, lenSeg, factorGt);
			if (lastIdxRes >= numRes || lastIdxGt >= numGt) {
				break;
			}
			transformDeltaRes = transformsRes[firstIdxRes].inverse() * transformsRes[lastIdxRes];
			transformDeltaGt = transformsGt[firstIdxGt].inverse() * transformsGt[lastIdxGt];
			transformError = transformDeltaGt.inverse() * transformDeltaRes;
			// Computes the error
			err.firstIdx = firstIdxRes;
			err.lastIdx = lastIdxRes;
			err.length = distancesRes[lastIdxRes] - distancesRes[firstIdxRes];
			err.errTransl = computeTranslationNorm(transformError) / lenSeg;
			err.errRot = computeRotationAngle(transformError) / lenSeg;
			errors.push_back(err);

			//			std::cout << "\n---\ninterval res [" << firstIdxRes << "," << lastIdxRes << "]  gt [" << firstIdxGt << "," << lastIdxGt << "]\n"
			//					<< "transformsRes[firstIdxRes]\n" << transformsRes[firstIdxRes].matrix() << "\n" << "transformsRes[lastIdxRes]\n" << transformsRes[lastIdxRes].matrix() << "\n"
			//					<< "transformsGt[firstIdxGt]\n" << transformsGt[firstIdxGt].matrix() << "\n" << "transformsGt[lastIdxGt]\n" << transformsGt[lastIdxGt].matrix() << "\n"
			//					<< "transformDeltaRes\n" << transformDeltaRes.matrix() << "\n" << "transformDeltaGt\n" << transformDeltaGt.matrix() << "\n"
			//					<< "transformError\n" << transformError.matrix() << "\n" << "errTransl " << err.errTransl << "  errRot[deg] " << (180 / M_PI * err.errRot)
			//					<< std::endl;
			std::cout << "---\ninterval res [" << firstIdxRes << "," << lastIdxRes << "](dist " << distancesRes[firstIdxRes] << "," << distancesRes[lastIdxRes] << " m) " 
					<< " factorRes " << factorRes << ", "
					<< "gt [" << firstIdxGt << "," << lastIdxGt << "](dist " << distancesGt[firstIdxGt] << "," << distancesGt[lastIdxGt] << " m) "
					<< " factorGt " << factorGt << ": "
					<< "errTransl[%] " << (100.0 * err.errTransl) << "  errRot[10^{-1} * deg/m] " << (18000.0 / M_PI * err.errRot)
					<< std::endl;
			if (err.errTransl > 0.50) {
				std::cout << "transformDeltaRes\n" << transformDeltaRes.matrix() << "\n"
						<< "transformDeltaGt\n" << transformDeltaGt.matrix() << "\n"
						<< "transformError\n" << transformError.matrix() << "\n";
			}

			//firstIdxRes = lastIdxRes;
			//firstIdxGt = lastIdxGt;
			//std::cout << "BEFORE: firstIdxRes " << firstIdxRes << ", firstIdxGt " << firstIdxGt << "\n";
			firstIdxRes = findDistanceIncrIdx(distancesRes, firstIdxRes, lenStep, factorRes);
			firstIdxGt = findDistanceIncrIdx(distancesGt, firstIdxGt, lenStep, factorGt);
			//std::cout << " AFTER: firstIdxRes " << firstIdxRes << ", firstIdxGt " << firstIdxGt << "\n";
		}
	}

	void computeErrorsInterp(const VectorTransform3 &transformsRes, const std::vector<float> &distancesRes, const VectorTransform3 &transformsGt,
			const std::vector<float> &distancesGt, float lenSeg, float lenStep, VectorErrorData &errors) {
		ErrorData err;
		Transform3 transformFirstRes, transformFirstGt, transformLastRes, transformLastGt; 
		Transform3 transformDeltaRes, transformDeltaGt, transformError;
		int numRes, numGt, firstIdxGt, lastIdxGt, firstIdxRes, lastIdxRes;
		float distStart, factorFirstRes, factorFirstGt, factorLastRes, factorLastGt;
		//float len;

		if (transformsRes.size() != distancesRes.size() || transformsGt.size() != distancesGt.size()) {
			std::cerr << "Error: different size of distances and trasforms" << std::endl;
			return;
		}
		numRes = transformsRes.size();
		numGt = transformsGt.size();
		if (numRes == 0 || numGt == 0) {
			std::cerr << "Error: one empty sequence: numRes " << numRes << " numGt " << numGt << std::endl;
			return;
		}
		//std::cout << "numRes " << numRes << " numGt " << numGt << std::endl;

		// Compares the relative pose of path segments of length lenSeg in result and groundtruth paths.
		// The first and last pose correspond respectively to distances distStart and distStart + lenSeg.
		// The initial value of distStart is 0 and is incremented of lenStep at each iteration
		distStart = 0.0;
		while (distStart + lenSeg < distancesRes.back() && distStart + lenSeg < distancesGt.back()) {
			// Computes the indices corresponding to distStart in result and groundtruth paths
			firstIdxRes = findDistanceIdx(distancesRes, distStart, factorFirstRes);
			//std::cout << "firstIdxRes " << firstIdxRes << std::endl;
			firstIdxGt = findDistanceIdx(distancesGt, distStart, factorFirstGt);
			//std::cout << "factorFirstGt " << factorFirstGt << std::endl;
			if (firstIdxRes >= numRes || firstIdxGt >= numGt) {
				break;
			}
			// Computes the indices corresponding to distStart in result and groundtruth paths
			lastIdxRes = findDistanceIdx(distancesRes, distStart + lenSeg, factorLastRes);
			//std::cout << "lastIdxRes " << lastIdxRes << std::endl;
			lastIdxGt = findDistanceIdx(distancesGt, distStart + lenSeg, factorLastGt);
			//std::cout << "lastIdxGt " << lastIdxGt << std::endl;
			if (lastIdxRes >= numRes || lastIdxGt >= numGt) {
				break;
			}
			// The first poses of result and groundtruth path segments are obtained with interpolation
			if (firstIdxRes == 0) {
				transformFirstRes = transformsRes[0];
			}
			else {
				interpolateTransform(transformsRes[firstIdxRes-1],transformsRes[firstIdxRes],factorFirstRes,transformFirstRes);
			}
			if (firstIdxGt == 0) {
				transformFirstGt = transformsGt[0];
			}
			else {
				interpolateTransform(transformsGt[firstIdxGt-1],transformsGt[firstIdxGt],factorFirstGt,transformFirstGt);
			}
			// The last poses of result and groundtruth path segments are obtained with interpolation
			interpolateTransform(transformsRes[lastIdxRes-1],transformsRes[lastIdxRes],factorLastRes,transformLastRes);
			interpolateTransform(transformsGt[lastIdxGt-1],transformsGt[lastIdxGt],factorLastGt,transformLastGt);
			// Computes the relative pose of result and groundtruth; then the error
			transformDeltaRes = transformFirstRes.inverse() * transformLastRes;
			transformDeltaGt = transformFirstGt.inverse() * transformLastGt;
			transformError = transformDeltaGt.inverse() * transformDeltaRes;
			// Saves the error
			err.firstIdx = firstIdxRes;
			err.lastIdx = lastIdxRes;
			err.length = distancesRes[lastIdxRes] - distancesRes[firstIdxRes];
			err.errTransl = computeTranslationNorm(transformError) / lenSeg;
			err.errRot = computeRotationAngle(transformError) / lenSeg;
			errors.push_back(err);

			std::cout << "res [" << firstIdxRes << "," << lastIdxRes << "](dist "
					<< (distancesRes[firstIdxRes] * (1.0 - factorFirstRes) + distancesRes[firstIdxRes+1] * factorFirstRes) << ","
					<< (distancesRes[lastIdxRes] * (1.0 - factorLastRes) + distancesRes[lastIdxRes+1] * factorLastRes) << ") "
					<< "gt [" << firstIdxGt << "," << lastIdxGt << "](dist "
					<< (distancesGt[firstIdxGt] * (1.0 - factorFirstGt) + distancesGt[firstIdxGt+1] * factorFirstGt) << ","
					<< (distancesGt[lastIdxGt] * (1.0 - factorLastGt) + distancesGt[lastIdxGt+1] * factorLastGt) << "): "
					<< "errTransl[%] " << (100.0 * err.errTransl) << "  errRot[10^{-1} * deg/m] " << (18000.0 / M_PI * err.errRot)
					<< std::endl;

			// Increments the starting distance
			distStart += lenStep;
		}
	}
	
	Transform3 computeTransform(float tx,float ty,float tz,float qw,float qx,float qy,float qz) {
		Transform3 transform;	
		Eigen::Quaternionf q(qw, qx, qy, qz);
		Eigen::Translation3f v(tx, ty, tz);
		transform = v * q;
		return transform; 
	}

}
