#pragma once

// Speckle rejection on the projected LaserScan.

#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

namespace stretch_core
{

struct SpeckleFilterConfig
{
  bool enabled{true};
  int min_points{2};
  int neighbor_window{3};
  int min_neighbors{2};
  float range_tolerance{0.15f};
};

struct ScanBins
{
  std::vector<float> ranges;
  std::vector<int> hit_counts;
};

inline bool speckleHasValidHit(const ScanBins & bins, float range_max, size_t index)
{
  return bins.hit_counts[index] > 0 &&
         std::isfinite(bins.ranges[index]) &&
         bins.ranges[index] < range_max;
}

// A scan spanning a full turn wraps, so bin 0 and bin N-1 are neighbours. A partial scan
// must not wrap or the two ends would vouch for each other across a gap they never saw.
inline bool speckleIsFullCircle(float angle_min, float angle_max, float angle_increment)
{
  const float span = angle_max - angle_min;
  const float full_circle = 2.0f * static_cast<float>(M_PI);
  return span >= full_circle - (1.5f * angle_increment);
}

inline void applySpeckleFilter(
  const SpeckleFilterConfig & config,
  float angle_min,
  float angle_max,
  float angle_increment,
  float range_max,
  ScanBins & bins)
{
  if (!config.enabled ||
    config.min_points <= 0 ||
    config.neighbor_window <= 0 ||
    config.min_neighbors <= 0)
  {
    return;
  }

  if (bins.ranges.empty() || bins.hit_counts.size() != bins.ranges.size()) {
    return;
  }

  const int num_ranges = static_cast<int>(bins.ranges.size());
  const bool wrap_scan = speckleIsFullCircle(angle_min, angle_max, angle_increment);
  const float range_tolerance = std::max(0.0f, config.range_tolerance);
  std::vector<float> filtered_ranges = bins.ranges;

  for (int i = 0; i < num_ranges; ++i) {
    const size_t bin = static_cast<size_t>(i);
    if (!speckleHasValidHit(bins, range_max, bin) ||
      bins.hit_counts[bin] >= config.min_points)
    {
      continue;
    }

    int similar_neighbors = 0;
    for (int offset = -config.neighbor_window; offset <= config.neighbor_window; ++offset) {
      if (offset == 0) {
        continue;
      }

      int neighbor = i + offset;
      if (wrap_scan) {
        neighbor %= num_ranges;
        if (neighbor < 0) {
          neighbor += num_ranges;
        }
      } else if (neighbor < 0 || neighbor >= num_ranges) {
        continue;
      }

      const size_t neighbor_bin = static_cast<size_t>(neighbor);
      if (speckleHasValidHit(bins, range_max, neighbor_bin) &&
        std::abs(bins.ranges[neighbor_bin] - bins.ranges[bin]) <= range_tolerance)
      {
        ++similar_neighbors;
        if (similar_neighbors >= config.min_neighbors) {
          break;
        }
      }
    }

    if (similar_neighbors < config.min_neighbors) {
      filtered_ranges[bin] = std::numeric_limits<float>::infinity();
      bins.hit_counts[bin] = 0;
    }
  }

  bins.ranges.swap(filtered_ranges);
}

}  // namespace stretch_core
