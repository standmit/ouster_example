/**
 * @file
 * @brief PCL point datatype for use with the OS-1
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace ouster_ros {
namespace OS1 {

struct EIGEN_ALIGN16 PointOS1base {
    PCL_ADD_POINT4D;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PointOS1base():
			data({0.f, 0.f, 0.f, 0.f}),
			intensity(0.f)
    {}

	PointOS1base(
				float x_,
				float y_,
				float z_,
				float intensity_
	):
			data({x_, y_, z_, 0.f}),
			intensity(intensity_)
	{}

    static inline PointOS1base make(float x, float y, float z, float intensity) {
        return PointOS1base(x, y, z, intensity);
    }
};

struct EIGEN_ALIGN16 PointOS1 : public PointOS1base {
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PointOS1():
		PointOS1base(),
		t(0),
		reflectivity(0),
		ring(0),
		noise(0),
		range(0)
    {}

	PointOS1(
			float x_,
			float y_,
			float z_,
			float intensity_,
			uint32_t t_,
			uint16_t reflectivity_,
			uint8_t ring_,
			uint16_t noise_,
			uint32_t range_
	):
			PointOS1base(x_, y_, z_, intensity_),
			t(t_),
			reflectivity(reflectivity_),
			ring(ring_),
			noise(noise_),
			range(range_)
    {}

    static inline PointOS1 make(float x, float y, float z, float intensity,
                                uint32_t t, uint16_t reflectivity, uint8_t ring,
                                uint16_t noise, uint32_t range) {
        return PointOS1(x, y, z, intensity, t, reflectivity, ring, noise, range);
    }
};

}
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointOS1base,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointOS1,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, t, t)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (uint16_t, noise, noise)
    (uint32_t, range, range)
)
// clang-format on
