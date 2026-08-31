#pragma once

// Open-addressing "first point in a voxel wins" set.
//
// The merger builds a std::unordered_map per frame (dual_lidar_pointcloud_merger_node.cpp
// voxelDownsample). At ~460k points a frame that is a node allocation and a pointer chase
// per point -- tens of milliseconds against a 100 ms budget at 10 Hz. This is the same
// decimation rule with the buckets flattened into one preallocated array: no allocation in
// the hot path, linear probing, and the storage is reused across frames.
//

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace stretch_core
{

class VoxelHashSet
{
public:
  static constexpr uint64_t kOccupied = 1ULL << 63;

  // Pack floor(p / leaf) into one key. Coordinates are masked to 21 bits, so two points
  // more than 52 km apart could alias -- far outside any lidar's range.
  static inline uint64_t key(float x, float y, float z, float inv_leaf)
  {
    const int64_t ix = static_cast<int64_t>(std::floor(x * inv_leaf));
    const int64_t iy = static_cast<int64_t>(std::floor(y * inv_leaf));
    const int64_t iz = static_cast<int64_t>(std::floor(z * inv_leaf));
    return ((static_cast<uint64_t>(ix) & 0x1FFFFFULL) << 42) |
           ((static_cast<uint64_t>(iy) & 0x1FFFFFULL) << 21) |
           (static_cast<uint64_t>(iz) & 0x1FFFFFULL);
  }

  // Size for the worst case (every point its own voxel) and keep the load factor at or
  // below 0.5, which is where linear probing stays close to O(1).
  void reset(size_t expected_points)
  {
    size_t capacity = 1024;
    while (capacity < expected_points * 2) {
      capacity <<= 1;
  }
    if (slots_.size() != capacity) {
      slots_.assign(capacity, 0);
    } else {
      std::fill(slots_.begin(), slots_.end(), 0);
  }
    mask_ = capacity - 1;
  }

  // True the first time a voxel is seen -- that point becomes the voxel's representative.
  bool insert(uint64_t voxel_key)
  {
    const uint64_t stored = voxel_key | kOccupied;
    size_t slot = static_cast<size_t>(mix(voxel_key)) & mask_;
    while (true) {
      const uint64_t current = slots_[slot];
      if (current == 0) {
        slots_[slot] = stored;
        return true;
      }
      if (current == stored) {
        return false;
      }
      slot = (slot + 1) & mask_;
  }
  }

private:
  // splitmix64 finalizer. The packed key has strong low-bit structure (z varies fastest),
  // which plain masking would turn into long probe runs.
  static inline uint64_t mix(uint64_t v)
  {
    v += 0x9E3779B97F4A7C15ULL;
    v = (v ^ (v >> 30)) * 0xBF58476D1CE4E5B9ULL;
    v = (v ^ (v >> 27)) * 0x94D049BB133111EBULL;
    return v ^ (v >> 31);
  }

  std::vector<uint64_t> slots_;
  size_t mask_{0};
};

}  // namespace stretch_core
