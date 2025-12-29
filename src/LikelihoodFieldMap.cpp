//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/LikelihoodFieldMap.h"
#include "emcl/Pose.h"
#include <ros/ros.h>
#include <random>
#include <algorithm>

namespace emcl2 {

LikelihoodFieldMap::LikelihoodFieldMap(const nav_msgs::OccupancyGrid &map, double likelihood_range)
{
	width_ = map.info.width;
	height_ = map.info.height;

	origin_x_ = map.info.origin.position.x;
	origin_y_ = map.info.origin.position.y;

	resolution_ = map.info.resolution;

	// DEBUG: Print map info
	ROS_INFO("=== LikelihoodFieldMap DEBUG ===");
	ROS_INFO("Map size: %d x %d, resolution: %f", width_, height_, resolution_);
	ROS_INFO("Map origin: (%f, %f)", origin_x_, origin_y_);
	ROS_INFO("likelihood_range: %f", likelihood_range);

	for(int x=0; x<width_; x++){
		likelihoods_.push_back(new double[height_]);

		for(int y=0; y<height_; y++)
			likelihoods_[x][y] = 0.0;
	}

	// DEBUG: Count cell types
	int occupied_count = 0;
	int free_count = 0;
	int unknown_count = 0;

	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++){
			int v = map.data[x + y*width_];
			if(v > 50){
				setLikelihood(x, y, likelihood_range);
				occupied_count++;
			}
			else if(0 <= v and v <= 50){
				free_cells_.push_back(std::pair<int, int>(x,y));
				free_count++;
			}
			else{
				unknown_count++;
			}
		}

	// DEBUG: Print cell counts
	ROS_INFO("Cell counts - Occupied (>50): %d, Free (0-50): %d, Unknown (<0): %d",
		occupied_count, free_count, unknown_count);
	ROS_INFO("Total cells: %d", width_ * height_);

	normalize();
	ROS_INFO("=== END LikelihoodFieldMap DEBUG ===");
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
	for(auto &e : likelihoods_)
		delete [] e;
}


double LikelihoodFieldMap::likelihood(double x, double y)
{
	int ix = (int)floor((x - origin_x_)/resolution_);
	int iy = (int)floor((y - origin_y_)/resolution_);

	if(ix < 0 or iy < 0 or ix >= width_ or iy >= height_)
		return -1.0;  // Out of bounds - return special value to avoid false penetration detection

	return likelihoods_[ix][iy];
}

double LikelihoodFieldMap::likelihood(double x, double y, float observed_intensity)
{
	// Default implementation ignores intensity
	(void)observed_intensity;
	return likelihood(x, y);
}

void LikelihoodFieldMap::setLikelihood(int x, int y, double range)
{
	int cell_num = (int)ceil(range/resolution_);
	std::vector<double> weights;
	for(int i=0;i<=cell_num;i++)
		weights.push_back(1.0 - (double)i/cell_num);

	for(int i=-cell_num; i<=cell_num; i++)
		for(int j=-cell_num; j<=cell_num; j++)
			if(i+x >= 0 and j+y >= 0 and i+x < width_ and j+y < height_)
				likelihoods_[i+x][j+y] = std::max(likelihoods_[i+x][j+y],
										std::min(weights[abs(i)], weights[abs(j)]));
}

void LikelihoodFieldMap::normalize(void)
{
	double maximum = 0.0;
	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			maximum = std::max(likelihoods_[x][y], maximum);

	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			likelihoods_[x][y] /= maximum;
}

void LikelihoodFieldMap::drawFreePoses(int num, std::vector<Pose> &result)
{
	std::random_device seed_gen;
	std::mt19937 engine{seed_gen()};
	std::vector<std::pair<int, int> > chosen_cells;

	sample(free_cells_.begin(), free_cells_.end(), back_inserter(chosen_cells), num, engine);

	for(auto &c : chosen_cells){
		Pose p;
		p.x_ = c.first*resolution_ + resolution_*rand()/RAND_MAX + origin_x_;
		p.y_ = c.second*resolution_ + resolution_*rand()/RAND_MAX + origin_y_;
		p.t_ = 2*M_PI*rand()/RAND_MAX - M_PI;
		result.push_back(p);
	}
}

}
