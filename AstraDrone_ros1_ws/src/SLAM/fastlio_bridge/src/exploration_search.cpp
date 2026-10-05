// Optional C ABI acceleration for the Python explorer. Same 8-neighbour
// Dijkstra and no-corner-cutting rule as exploration_grid.py, in metres.
// No ROS/Gazebo truth input, no change to footprint or collision constraints.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

extern "C" int astra_grid_search_weighted(const uint8_t* safe, int h, int w, int start,
                                  double resolution, const float* penalties,
                                  double* distances, int32_t* parents) noexcept {
  if (!safe || !distances || !parents || h < 1 || w < 1 ||
      static_cast<int64_t>(h)*w > 2000000 || start < 0 || start >= h*w ||
      !std::isfinite(resolution) || resolution <= 0) return -1;
  try {
    const int n = h*w;
    if (penalties) for (int i=0; i<n; ++i)
      if (!std::isfinite(penalties[i]) || penalties[i]<0 || penalties[i]>100) return -1;
    std::fill(distances, distances+n, std::numeric_limits<double>::infinity());
    std::fill(parents, parents+n, -1);
    if (!safe[start]) return 0;
    using Item = std::pair<double,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    distances[start]=0.; queue.emplace(0.,start);
    int reached=0;
    while (!queue.empty()) {
      const auto entry=queue.top(); queue.pop();
      const double cost=entry.first;
      const int u=entry.second, y=u/w, x=u%w;
      if (cost != distances[u]) continue;
      ++reached;
      for (int dy=-1; dy<=1; ++dy) for (int dx=-1; dx<=1; ++dx) {
        if (!(dx || dy)) continue;
        const int yy=y+dy, xx=x+dx;
        if (yy<0 || yy>=h || xx<0 || xx>=w) continue;
        const int v=yy*w+xx;
        if (!safe[v] || (dx && dy && (!safe[yy*w+x] || !safe[y*w+xx]))) continue;
        // Soft preference ONLY: never makes an unsafe cell traversable.
        // Distances with penalties are equivalent-length costs, not path length.
        const double next=cost+resolution*((dx && dy)?std::sqrt(2.):1.)*
                              (1.+(penalties ? penalties[v] : 0.));
        if (next+1e-9 < distances[v]) {
          distances[v]=next; parents[v]=u; queue.emplace(next,v);
        }
      }
    }
    return reached;
  } catch (...) { return -2; }  // Never unwind a C++ exception through ctypes.
}

// Retain the unweighted ABI for existing deployments and microbenchmarks.
extern "C" int astra_grid_search(const uint8_t* safe, int h, int w, int start,
                                  double resolution, double* distances,
                                  int32_t* parents) noexcept {
  return astra_grid_search_weighted(safe,h,w,start,resolution,nullptr,distances,parents);
}
