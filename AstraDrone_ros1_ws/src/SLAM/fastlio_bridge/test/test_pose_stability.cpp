#include "../src/pose_stability.hpp"
#include <cassert>
#include <limits>
int main() {
    PoseStability s;
    for (int i=0;i<20;++i) assert(!s.update(1+i*.1,0,0,-1.87+i*.1));
    bool ok=false;
    for (int i=0;i<40;++i) ok=s.update(3+i*.1,0,0,0);
    assert(ok);
    assert(!s.update(12,0,0,0)); // dropout
    assert(!s.update(1,0,0,0)); // time reset
    assert(!s.update(2,0,0,std::numeric_limits<double>::quiet_NaN()));
    for (int i=0;i<40;++i) ok=s.update(3+i*.1,0,0,-2);
    assert(ok); // negative ENU is not itself invalid
    s.clear();
    for (int i=0;i<40;++i) {
        ok=s.update(3+i*.1,0,0,0);
        assert(!s.update(3+i*.1,0,0,0));
    }
    assert(ok); // repeated cached samples cannot reset the whole window
}
