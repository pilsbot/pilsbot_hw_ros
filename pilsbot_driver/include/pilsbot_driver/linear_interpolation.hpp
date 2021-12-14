#pragma once

#include <vector>
#include <algorithm>
#include <iostream>

template<typename FROM, typename TO> class LinearInterpolator {
public:
  struct Point {
    FROM x; TO y;
  };

  typedef std::vector<Point> CalibrationList;

private:
  CalibrationList cal_;
public:
  LinearInterpolator(CalibrationList& cal) : cal_(cal){};
  //default mapping
  LinearInterpolator() : cal_({{0, 0},{1,1}}){};

  TO operator()(FROM from){
    Point low,high;
    if(from <= cal_.front().x) {
      if(from < cal_.front().x) {
        std::cout << "Warn: X val " << from << " is lower than calibration range "
            << cal_.front().x << "-" << cal_.back().x << std::endl;
      }
      low = cal_[0];
      high = cal_[1];
    } else if (from >= cal_[cal_.size()-2].x) {
      if(from > cal_.back().x) {
        std::cout << "Warn: X val " << from << " is higher than calibration range "
            << cal_.front().x << "-" << cal_.back().x << std::endl;
      }
      low = cal_[cal_.size()-2];
      high = cal_[cal_.size()-1];
    } else {
      auto lower_bound = std::partition_point(cal_.begin(), cal_.end(),
          [from](const auto& s) { return s.x < from; });
      low = *(lower_bound-1);
      high = *lower_bound;
    }

    auto dx = (high.x - low.x);
    auto dy = (high.y - low.y);
    return low.y + (from - low.x) * dy / dx;
  }
};
