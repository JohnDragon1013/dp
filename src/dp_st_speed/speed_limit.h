//
// Created by jydragon on 18-8-1.
//

#ifndef LATTICEPLAN_SPEED_LIMIT_H
#define LATTICEPLAN_SPEED_LIMIT_H

#include <utility>
#include <vector>
#include <algorithm>
class SpeedLimit {
public:
    SpeedLimit() = default;

    void AppendSpeedLimit(const double s, const double v){
        if (!speed_limit_points_.empty()) {
            //DCHECK_GE(s, speed_limit_points_.back().first);
        }
        speed_limit_points_.emplace_back(s, v);
    }

    const std::vector<std::pair<double, double>>& speed_limit_points() const{
        return speed_limit_points_;
    };

    double GetSpeedLimitByS(const double s) const{
        auto compare_s = [](const std::pair<double, double>& point, const double s) {
            return point.first < s;
        };

        auto it_lower = std::lower_bound(speed_limit_points_.begin(),speed_limit_points_.end(), s, compare_s);

        if (it_lower == speed_limit_points_.end()) {
            return (it_lower - 1)->second;
        }
        return it_lower->second;
    }

    void Clear(){ speed_limit_points_.clear(); }

private:
    // use a vector to represent speed limit
    // the first number is s, the second number is v
    // It means at distance s from the start point, the speed limit is v.
    std::vector<std::pair<double, double>> speed_limit_points_;
};




#endif //LATTICEPLAN_SPEED_LIMIT_H
