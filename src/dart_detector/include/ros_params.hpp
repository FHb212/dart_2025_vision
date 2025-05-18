
#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <string>
#include <vector>

struct RosParams {
  std::vector<int64_t> hsv_inrange_thresh_lo{};
  std::vector<int64_t> hsv_inrange_thresh_hi{};
  double gaussian_blur_radius{};
};

#endif  // ROS_PARAMS_HPP
