#pragma once
#include <cmath>
#include <deque>
#include <algorithm>

// Ground-only initialization guard. A negative ENU Z is legal; stability,
// timestamps and landed state matter, NOT the sign of the coordinate.
class PoseStability {
    struct Sample { double t,x,y,z; };
    std::deque<Sample> samples_;
public:
    double duration{3.0}, z_range{0.12}, xy_range{0.25}, max_gap{0.5};
    void clear() { samples_.clear(); }
    bool update(double t,double x,double y,double z) {
        if (!std::isfinite(t)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||t<=0) {
            clear(); return false;
        }
        // Cached/repeated stamps do not count as extra stability evidence and
        // must not erase the genuinely new samples accumulated so far.
        if (!samples_.empty() && t==samples_.back().t) return false;
        if (!samples_.empty() && (t<samples_.back().t || t-samples_.back().t>max_gap)) clear();
        samples_.push_back({t,x,y,z});
        // Keep one boundary sample so a regular sampled stream can reach the
        // full duration instead of always falling just short of it.
        while (samples_.size()>1 && samples_[1].t<=t-duration) samples_.pop_front();
        double xmin=x,xmax=x,ymin=y,ymax=y,zmin=z,zmax=z;
        for (const auto& s:samples_) {
            xmin=std::min(xmin,s.x); xmax=std::max(xmax,s.x);
            ymin=std::min(ymin,s.y); ymax=std::max(ymax,s.y);
            zmin=std::min(zmin,s.z); zmax=std::max(zmax,s.z);
        }
        return t-samples_.front().t>=duration && zmax-zmin<=z_range &&
               std::hypot(xmax-xmin,ymax-ymin)<=xy_range;
    }
};
