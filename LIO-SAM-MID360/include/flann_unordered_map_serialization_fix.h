#pragma once

// Workaround for some libflann versions that do not provide
// `flann::serialization::Serializer` specialization for `std::unordered_map`,
// causing compilation failures when headers instantiate LSH index serialization.
//
// Error typically looks like:
//   flann/util/serialization.h:34:14: error: ‘std::unordered_map<...>’ has no member named ‘serialize’
//
// We provide the missing specialization for the exact type used by FLANN LSH:
//   std::unordered_map<unsigned int, std::vector<unsigned int>>

#include <unordered_map>
#include <vector>

namespace flann {
namespace serialization {

// Forward declaration (matches libflann's primary template).
template <typename T>
struct Serializer;

template <>
struct Serializer<std::unordered_map<unsigned int, std::vector<unsigned int>>> {
  using MapT = std::unordered_map<unsigned int, std::vector<unsigned int>>;

  template <typename InputArchive>
  static inline void load(InputArchive& ar, MapT& map_val) {
    size_t size = 0;
    ar & size;

    map_val.clear();
    // Reserve is available in C++11+, but keep it defensive.
    map_val.reserve(size);

    for (size_t i = 0; i < size; ++i) {
      unsigned int key;
      ar & key;
      std::vector<unsigned int> value;
      ar & value;
      map_val.emplace(key, std::move(value));
    }
  }

  template <typename OutputArchive>
  static inline void save(OutputArchive& ar, const MapT& map_val) {
    size_t size = map_val.size();
    ar & size;
    for (typename MapT::const_iterator it = map_val.begin(); it != map_val.end(); ++it) {
      ar & it->first;
      ar & it->second;
    }
  }
};

}  // namespace serialization
}  // namespace flann


