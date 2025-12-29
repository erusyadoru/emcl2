//SPDX-FileCopyrightText: 2024
//SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl/IntensityLikelihoodFieldMap.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <cmath>

namespace emcl2 {

IntensityLikelihoodFieldMap::IntensityLikelihoodFieldMap(
		const nav_msgs::OccupancyGrid &map,
		double likelihood_range,
		const std::string &intensity_map_path,
		double geo_weight,
		double intensity_weight,
		double intensity_sigma,
		double intensity_min_raw,
		double intensity_max_raw)
	: LikelihoodFieldMap(map, likelihood_range),
	  geo_weight_(geo_weight),
	  intensity_weight_(intensity_weight),
	  intensity_sigma_(intensity_sigma),
	  intensity_min_raw_(intensity_min_raw),
	  intensity_max_raw_(intensity_max_raw)
{
	ROS_INFO("=== IntensityLikelihoodFieldMap ===");
	ROS_INFO("geo_weight: %f, intensity_weight: %f, intensity_sigma: %f",
	         geo_weight_, intensity_weight_, intensity_sigma_);
	ROS_INFO("intensity_min_raw: %f, intensity_max_raw: %f",
	         intensity_min_raw_, intensity_max_raw_);

	// Initialize intensity array
	for(int x = 0; x < width_; x++){
		expected_intensity_.push_back(new uint8_t[height_]);
		for(int y = 0; y < height_; y++)
			expected_intensity_[x][y] = 128;  // Default to middle gray
	}

	// Load intensity map
	if(!intensity_map_path.empty()){
		if(loadIntensityMap(intensity_map_path)){
			ROS_INFO("Intensity map loaded successfully");
		} else {
			ROS_WARN("Failed to load intensity map, using default values");
		}
	} else {
		ROS_WARN("No intensity map path provided");
	}

	ROS_INFO("=== END IntensityLikelihoodFieldMap ===");
}

IntensityLikelihoodFieldMap::~IntensityLikelihoodFieldMap()
{
	for(auto &e : expected_intensity_)
		delete [] e;
}

bool IntensityLikelihoodFieldMap::loadIntensityMap(const std::string &path)
{
	std::ifstream file(path, std::ios::binary);
	if(!file.is_open()){
		ROS_ERROR("Cannot open intensity map: %s", path.c_str());
		return false;
	}

	// Read PGM header
	std::string magic;
	file >> magic;

	if(magic != "P5" && magic != "P2"){
		ROS_ERROR("Invalid PGM format (expected P5 or P2): %s", magic.c_str());
		return false;
	}

	// Skip comments
	file >> std::ws;
	while(file.peek() == '#'){
		std::string comment;
		std::getline(file, comment);
		file >> std::ws;
	}

	int img_width, img_height, max_val;
	file >> img_width >> img_height >> max_val;

	ROS_INFO("Intensity map: %d x %d, max_val: %d", img_width, img_height, max_val);

	if(img_width != width_ || img_height != height_){
		ROS_WARN("Intensity map size (%d x %d) differs from occupancy map (%d x %d)",
		         img_width, img_height, width_, height_);
		// Continue anyway, will use what we can
	}

	// Read one byte (newline after header)
	file.get();

	// Read pixel data
	if(magic == "P5"){
		// Binary format
		for(int y = height_ - 1; y >= 0; y--){  // PGM is top-to-bottom, we want bottom-to-top
			for(int x = 0; x < width_; x++){
				if(x < img_width && y < img_height){
					unsigned char pixel;
					file.read(reinterpret_cast<char*>(&pixel), 1);
					if(file){
						expected_intensity_[x][y] = pixel;
					}
				}
			}
			// Skip remaining pixels if intensity map is wider
			for(int x = width_; x < img_width; x++){
				unsigned char dummy;
				file.read(reinterpret_cast<char*>(&dummy), 1);
			}
		}
	} else {
		// ASCII format (P2)
		for(int y = height_ - 1; y >= 0; y--){
			for(int x = 0; x < width_; x++){
				if(x < img_width && y < img_height){
					int pixel;
					file >> pixel;
					if(file){
						expected_intensity_[x][y] = static_cast<uint8_t>(pixel * 255 / max_val);
					}
				}
			}
		}
	}

	file.close();

	// Debug: print some sample values
	int sample_count = 0;
	for(int x = 0; x < width_ && sample_count < 5; x += width_/5){
		for(int y = 0; y < height_ && sample_count < 5; y += height_/5){
			ROS_INFO("Sample intensity at (%d,%d): %d", x, y, expected_intensity_[x][y]);
			sample_count++;
		}
	}

	return true;
}

double IntensityLikelihoodFieldMap::scaleIntensity(double raw_intensity) const
{
	// Scale raw intensity from [min_raw, max_raw] to [0, 255]
	if(intensity_max_raw_ <= intensity_min_raw_)
		return 128.0;  // Invalid range, return middle value

	double scaled = (raw_intensity - intensity_min_raw_) / (intensity_max_raw_ - intensity_min_raw_) * 255.0;

	// Clamp to [0, 255]
	if(scaled < 0.0) scaled = 0.0;
	if(scaled > 255.0) scaled = 255.0;

	return scaled;
}

double IntensityLikelihoodFieldMap::likelihood(double x, double y, float observed_intensity)
{
	int ix = (int)floor((x - origin_x_) / resolution_);
	int iy = (int)floor((y - origin_y_) / resolution_);

	if(ix < 0 || iy < 0 || ix >= width_ || iy >= height_)
		return -1.0;  // Out of bounds

	// Geometric likelihood (from parent class)
	double geo_ll = likelihoods_[ix][iy];

	// Intensity likelihood
	double expected = static_cast<double>(expected_intensity_[ix][iy]);
	double observed_scaled = scaleIntensity(static_cast<double>(observed_intensity));

	// Gaussian matching: exp(-(diff^2) / (2 * sigma^2))
	// Both expected and observed_scaled are now in 0-255 range
	double diff = observed_scaled - expected;
	double intensity_ll = std::exp(-(diff * diff) / (2.0 * intensity_sigma_ * intensity_sigma_));

	// Weighted combination: w1 * geo + w2 * intensity
	double combined = geo_weight_ * geo_ll + intensity_weight_ * intensity_ll;

	return combined;
}

double IntensityLikelihoodFieldMap::getExpectedIntensity(double x, double y)
{
	int ix = (int)floor((x - origin_x_) / resolution_);
	int iy = (int)floor((y - origin_y_) / resolution_);

	if(ix < 0 || iy < 0 || ix >= width_ || iy >= height_)
		return -1.0;

	return static_cast<double>(expected_intensity_[ix][iy]);
}

}
