#pragma once

#include <limits>

namespace reeds_shepp_ompl {

enum ReedsSheppPathSegmentType {
    RS_NOP = 0,
    RS_LEFT = 1,
    RS_STRAIGHT = 2,
    RS_RIGHT = 3
};

class ReedsSheppPath {
public:
    ReedsSheppPath(const ReedsSheppPathSegmentType *type = nullptr,
                   double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                   double w = 0., double x = 0.);
    double length() const {
        return totalLength_;
    }

    const ReedsSheppPathSegmentType *type_;
    double length_[5];
    double totalLength_;
};

ReedsSheppPath getPath(double x0, double y0, double th0, double x1, double y1, double th1, double rho);

} // namespace reeds_shepp_ompl
