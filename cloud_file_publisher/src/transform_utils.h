#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>


namespace rimlab_kitti {

	using Vector3 = Eigen::Vector3f;
	using Quaternion = Eigen::Quaternionf;
	using Transform3 = Eigen::Affine3f;
	//using VectorTransform3 = std::vector<Transform3, Eigen::aligned_allocator<Transform3> >;
	using VectorTransform3 = std::vector<Transform3>;

	struct ErrorData {
		int firstIdx;
		int lastIdx;
		float length;
		float errTransl;
		float errRot;
	};
	using VectorErrorData = std::vector<ErrorData>;

	bool readTransformLine(std::istream &in, Transform3 &transform);

	bool readTimeTransformLine(std::istream &in, float &time, Transform3 &transform);

	bool readTimePosQuatLine(std::istream &in, float &time, Transform3 &transform);

	bool readTransformFile(const std::string &filename, VectorTransform3 &transforms);

	bool readTimeTransformFile(const std::string &filename, std::vector<float>& times, VectorTransform3 &transforms);

	bool readTimePosQuatFile(const std::string &filename, std::vector<float> &times, VectorTransform3 &transforms);
	
	void writePoseQuat(std::ostream& out, const float& t, const Eigen::Affine3f& pose);
	
	void writePoseMat(std::ostream& out, const Eigen::Affine3f& pose);

	void computeDistances(const VectorTransform3 &transforms, std::vector<float> &distances);

	int findDistanceIdx(const std::vector<float> &distances, float len, float& factor);

	int findDistanceIncrIdx(const std::vector<float> &distances, int startIdx, float lenStep);

	float computeTranslationNorm(const Transform3 &transformDelta);

	float computeRotationAngle(const Transform3 &transformDelta);

	void interpolateTransform(const Transform3& transf0,const Transform3& transf1,float f,Transform3& transfInterp);

	void computeErrors(const VectorTransform3 &transformsRes, const std::vector<float> &distancesRes, const VectorTransform3 &transformsGt,
			const std::vector<float> &distancesGt, float lenSeg, float lenStep, VectorErrorData& errors);

	void computeErrorsInterp(const VectorTransform3 &transformsRes, const std::vector<float> &distancesRes, const VectorTransform3 &transformsGt,
			const std::vector<float> &distancesGt, float lenSeg, float lenStep, VectorErrorData& errors);
			
	Transform3 computeTransform(float tx,float ty,float tz,float qw,float qx,float qy,float qz);

}

#endif
