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
#include "network/hyperx/RoutingAlgorithmFactory.h"

#include <cassert>

#include "network/hyperx/DimOrderRoutingAlgorithm.h"
#include "network/hyperx/MinRoutingAlgorithm.h"
#include "network/hyperx/ValiantsRoutingAlgorithm.h"
#include "network/hyperx/LeastCongestedQueueRoutingAlgorithm.h"
#include "network/hyperx/UgalRoutingAlgorithm.h"
#include "network/hyperx/DalRoutingAlgorithm.h"
#include "network/hyperx/SkippingDimensionsRoutingAlgorithm.h"

#include "network/RoutingAlgorithm.h"

namespace HyperX {

RoutingAlgorithmFactory::RoutingAlgorithmFactory(
    u32 _baseVc, u32 _numVcs, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    Json::Value _settings)
    : ::RoutingAlgorithmFactory(), baseVc_(_baseVc), numVcs_(_numVcs),
      dimensionWidths_(_dimensionWidths), dimensionWeights_(_dimensionWeights),
      concentration_(_concentration), settings_(_settings) {}

RoutingAlgorithmFactory::~RoutingAlgorithmFactory() {}

RoutingAlgorithm* RoutingAlgorithmFactory::createRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u32 _inputPort) {
  std::string algorithm = settings_["algorithm"].asString();
  u32 latency = settings_["latency"].asUInt();

  if (algorithm == "dimension_order") {
    return new HyperX::DimOrderRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, settings_);
  } else if (algorithm == "unordered_minimal") {
    return new HyperX::MinRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, settings_);
  } else if (algorithm == "valiants") {
    return new HyperX::ValiantsRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, settings_);
  } else if (algorithm == "least_congested_queue") {
    return new HyperX::LeastCongestedQueueRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, settings_);
  } else if (algorithm == "universal_global_adaptive") {
    return new HyperX::UgalRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, settings_);
  } else if (algorithm == "dimensionally_adaptive") {
    return new HyperX::DalRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, _inputPort, settings_);
  } else if (algorithm == "skipping_dimensions") {
    return new HyperX::SkippingDimensionsRoutingAlgorithm(
        _name, _parent, _router, latency, baseVc_, numVcs_, dimensionWidths_,
        dimensionWeights_, concentration_, _inputPort, settings_);
  } else {
    fprintf(stderr, "Unknown routing algorithm: '%s'\n", algorithm.c_str());
    assert(false);
  }
}

}  // namespace HyperX
