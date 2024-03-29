#pragma once

#include <vector>
#include <algorithm>
#include <iostream>

namespace linear_interpolator {
  typedef std::vector<double> CalibrationListSerialized;  // interpreted as pairs
}

template<typename FROM, typename TO> class LinearInterpolator {
public:
  struct Point {
    double x; double y;
  };

  typedef std::vector<Point> CalibrationList;

private:
  CalibrationList cal_;
public:
  LinearInterpolator(CalibrationList& cal) : cal_(cal){};
  //default mapping
  LinearInterpolator() : cal_({{0, 0},{1,1}}){};

  bool deserialize(const linear_interpolator::CalibrationListSerialized raw_cal) {
     if(raw_cal.size() < 4 || raw_cal.size() % 2 != 0)
     {
       std::cerr << "number of calibration values is invalid (>4, %2), is:"
           << raw_cal.size() << std::endl << "Using default 1:1 mapping." << std::endl;
       return false;
     } else {
       CalibrationList cal;
       for(unsigned i = 0; i < raw_cal.size(); i+=2) {
         cal.push_back({raw_cal[i], raw_cal[i+1]});
       }
       std::sort(cal.begin(), cal.end(),
           [](const auto& lhs, const auto& rhs) {
             return lhs.x < rhs.x;
           });

       //check sanity
       for(unsigned i = 1; i < cal.size(); i++) {
         if(cal[i].x == cal[i-1].x) {
           std::cerr << "More than one Element with x=" << cal[i].x << std::endl;
           return false;
         }
       }

       std::cout << "[linear interpolation] Got Calibration Values:" << std::endl;
       for (const auto& point : cal)
         std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
       cal_ = cal;
     }
     return true;
  }

  TO operator()(FROM from){
    Point low,high;
    if(from <= cal_.front().x) {
      if(from < cal_.front().x) {
        std::cout << "Warn: X val " << from << " is lower than calibration range "
            << cal_.front().x << "-" << cal_.back().x << std::endl;
      }
      // edge case: we hit exactly first Point
      low = cal_[0];
      high = cal_[1];
    } else if (from >= cal_[cal_.size()-2].x) {
      if(from > cal_.back().x) {
        std::cout << "Warn: X val " << from << " is higher than calibration range "
            << cal_.front().x << "-" << cal_.back().x << std::endl;
      }
      // use last point pair
      low = cal_[cal_.size()-2];    // next-to last
      high = cal_[cal_.size()-1];   // last element
    } else {
      auto lower_bound = std::partition_point(cal_.begin(), cal_.end(),
          [from](const auto& s) { return s.x < from; });
      low = *(lower_bound-1);
      high = *(lower_bound);
    }

    const auto dx = (high.x - low.x);
    const auto dy = (high.y - low.y);
    return low.y + (from - low.x) * dy / dx;
  }
};
