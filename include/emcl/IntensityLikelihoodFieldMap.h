//SPDX-FileCopyrightText: 2024
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef INTENSITY_LIKELIHOOD_FIELD_MAP_H__
#define INTENSITY_LIKELIHOOD_FIELD_MAP_H__

#include <vector>
#include <string>
#include "emcl/LikelihoodFieldMap.h"
#include <nav_msgs/OccupancyGrid.h>

namespace emcl2 {

class IntensityLikelihoodFieldMap : public LikelihoodFieldMap
{
public:
	IntensityLikelihoodFieldMap(const nav_msgs::OccupancyGrid &map,
	                            double likelihood_range,
	                            const std::string &intensity_map_path,
	                            double geo_weight,
	                            double intensity_weight,
	                            double intensity_sigma,
	                            double intensity_min_raw,
	                            double intensity_max_raw);
	~IntensityLikelihoodFieldMap();

	// Combined likelihood using both geometric and intensity (override)
	double likelihood(double x, double y, float observed_intensity) override;

	// Check if this map supports intensity
	bool hasIntensity() const override { return true; }

	// Intensity-only lookup (for debugging)
	double getExpectedIntensity(double x, double y);

private:
	std::vector<uint8_t*> expected_intensity_;  // Expected intensity at each cell (0-255)

	double geo_weight_;        // w1: weight for geometric likelihood
	double intensity_weight_;  // w2: weight for intensity likelihood
	double intensity_sigma_;   // sigma for Gaussian intensity matching (in 0-255 scale)
	double intensity_min_raw_; // Minimum raw intensity value from sensor
	double intensity_max_raw_; // Maximum raw intensity value from sensor

	// Scale raw intensity to 0-255 range
	double scaleIntensity(double raw_intensity) const;

	bool loadIntensityMap(const std::string &path);
};

}

#endif
