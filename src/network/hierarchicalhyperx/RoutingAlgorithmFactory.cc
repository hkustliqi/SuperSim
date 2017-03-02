/*
 * Copyright 2016 Hewlett Packard Enterprise Development LP
 *
 * Licensed under the Apache License, Version 2.0 (the 'License');
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "network/hierarchicalhyperx/RoutingAlgorithmFactory.h"

#include <cassert>

namespace HierarchicalHyperX {

RoutingAlgorithmFactory::RoutingAlgorithmFactory(
    u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _globalLinksPerRouter, u32 _concentration,
    Json::Value _settings)
    : ::RoutingAlgorithmFactory(), baseVc_(_baseVc), numVcs_(_numVcs),
      globalDimensionWidths_(_globalDimensionWidths),
      globalDimensionWeights_(_globalDimensionWeights),
      localDimensionWidths_(_localDimensionWidths),
      localDimensionWeights_(_localDimensionWeights),
      globalLinksPerRouter_(_globalLinksPerRouter),
      concentration_(_concentration), settings_(_settings) {}

RoutingAlgorithmFactory::~RoutingAlgorithmFactory() {}

RoutingAlgorithm* RoutingAlgorithmFactory::createRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u32 _inputPort) {
  std::string algorithm = settings_["algorithm"].asString();
  u32 latency = settings_["latency"].asUInt();
  bool randomGroup = settings_["random_group"].asBool();
  f64 congestionThreshold = settings_["congestion_threshold"].asFloat();
  u32 localDetour = settings_["num_local_deroute"].asUInt();
  f64 bias = settings_["bias"].asFloat();

  if (algorithm == "dimension_order") {
    return new HierarchicalHyperX::DimOrderRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_);
  } else if (algorithm == "valiant") {
    return new HierarchicalHyperX::ValiantRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_,
        randomGroup);
  } else if (algorithm == "global_random") {
    return new HierarchicalHyperX::GlobalRandomRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_);
  } else if (algorithm == "global_local_random") {
    return new HierarchicalHyperX::GlobalAndLocalRandomRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_);
  } else if (algorithm == "min_adaptive") {
    return new HierarchicalHyperX::MinimalAdaptiveRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_,
        congestionThreshold, localDetour);
  } else if (algorithm == "progressive_adaptive") {
    return new HierarchicalHyperX::ProgressiveAdaptiveRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_,
        randomGroup, bias);
  } else if (algorithm == "progressive_adaptive_avg_offset") {
    return new HierarchicalHyperX::
      ProgressiveAdaptiveAverageOffsetRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_,
        randomGroup, bias);
  } else if (algorithm == "progressive_adaptive_avg") {
    return new HierarchicalHyperX::ProgressiveAdaptiveAverageRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_,
        randomGroup);
  } else if (algorithm == "threshold_progressive_adaptive") {
    return new HierarchicalHyperX::ThresholdProgressiveAdaptiveRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_,
        globalDimensionWidths_, globalDimensionWeights_, localDimensionWidths_,
        localDimensionWeights_, concentration_, globalLinksPerRouter_,
        congestionThreshold, randomGroup);
  } else if (algorithm == "threshold_progressive_adaptive_g") {
    return new HierarchicalHyperX::
        ThresholdProgressiveAdaptiveGRoutingAlgorithm(
            _name, _parent, _router, latency, baseVc_, numVcs_,
            globalDimensionWidths_, globalDimensionWeights_,
            localDimensionWidths_, localDimensionWeights_, concentration_,
            globalLinksPerRouter_, congestionThreshold);
  } else {
    fprintf(stderr, "Unknown routing algorithm: '%s'\n", algorithm.c_str());
    assert(false);
  }
}

}  // namespace HierarchicalHyperX
