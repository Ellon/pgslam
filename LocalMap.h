#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include "types.h"

namespace pgslam {

template<typename T>
class LocalMap {
public:
  IMPORT_PGSLAM_TYPES(T)

  using CompositionZ = boost::circular_buffer<Vertex>;
  using DataElement = std::pair<Vertex,Keyframe>;
  class DataBuffer : public boost::circular_buffer<DataElement> {
  public:
    DataBuffer();
    DataBuffer(size_t capacity);
    DataBuffer(const Graph & g, const CompositionZ & comp);
  };

public:
  LocalMap(size_t capacity);
  LocalMap(const Graph & g, const CompositionZ & comp);

  size_t Capacity();

  void UpdateFromGraph(const Graph & g);
  void UpdateFromDataBuffer(const DataBuffer & db);
  void UpdateToNewComposition(const Graph & g, const CompositionZ & comp);

  bool HasCloud() const;
  const DP & Cloud() const;
  DP CloudInWorldFrame() const;

  CompositionZ Composition() const;

  Vertex ReferenceVertex() const;
  const Keyframe & ReferenceKeyframe() const;

  bool HasSameVertexSet(const CompositionZ & comp) const;
  bool HasSameReferenceVertex(const CompositionZ & comp) const;
  bool HasSameComposition(const CompositionZ & comp) const;

  bool IsOutdated(const Graph & graph) const;
  bool IsReferenceKeyframeOutdated(const Graph & graph) const;

private:
  void BuildCloudFromData();

private:
  DataBuffer data_;
  DP cloud_;
  TransformationPtr rigid_transformation_;
};

} // pgslam

#include "LocalMap.hpp"

#endif // LOCAL_MAP_H
