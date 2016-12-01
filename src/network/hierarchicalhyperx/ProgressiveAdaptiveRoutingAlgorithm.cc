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
#include "network/hierarchicalhyperx/ProgressiveAdaptiveRoutingAlgorithm.h"

#include <strop/strop.h>

#include <cassert>
#include <unordered_set>
#include <unordered_map>
#include <set>

#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"

namespace HierarchicalHyperX {

ProgressiveAdaptiveRoutingAlgorithm::ProgressiveAdaptiveRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter)
    : ValiantRoutingAlgorithm
      (_name, _parent,  _router, _latency, _baseVc, _numVcs,
       _globalDimensionWidths,
       _globalDimensionWeights, _localDimensionWidths, _localDimensionWeights,
       _concentration, _globalLinksPerRouter, true) {
  assert(numVcs_ >= 2 * localDimWidths_.size() + 2 * globalDimWidths_.size());
}

ProgressiveAdaptiveRoutingAlgorithm::~ProgressiveAdaptiveRoutingAlgorithm() {}

void ProgressiveAdaptiveRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->packet()->message()->getDestinationAddress();
  Packet* packet = _flit->packet();

  if (packet->getRoutingExtension() == nullptr) {
    RoutingInfo* ri = new RoutingInfo();
    ri->intermediateAddress = nullptr;
    ri->localDst = nullptr;
    ri->localDstPort = nullptr;
    ri->localDerouteCount = 0;
    ri->globalHopCount = 0;
    ri->intermediateDone = false;
    ri->valiantMode = false;
    packet->setRoutingExtension(ri);
  }
  RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());

  std::unordered_set<u32> outputPorts;
  // routing depends on mode
  if (ri->valiantMode == false) {
    outputPorts = ProgressiveAdaptiveRoutingAlgorithm::routing
      (_flit, *destinationAddress);
    assert(outputPorts.size() >= 1);
  } else {
    // use Valiant routing
    outputPorts = ValiantRoutingAlgorithm::routing(
        _flit, *destinationAddress);
    assert(outputPorts.size() >= 1);
  }

  // reset localDst once in a new group
  if (*outputPorts.begin() >= getPortBase(concentration_, localDimWidths_,
                                          localDimWeights_)) {
    ri->globalHopCount++;
    // delete local router
    if (ri->localDst != nullptr) {
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
    }
    ri->localDst = nullptr;
    if (ri->localDstPort != nullptr) {
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
    }
    ri->localDstPort = nullptr;
    packet->setRoutingExtension(ri);
  }

  // figure out which VC set to use
  u32 vcSet;
  if (ri->globalHopCount == 0) {
    vcSet = packet->getHopCount() - 1;
  } else {
    vcSet = 2*localDimWidths_.size() - 1 + ri->globalHopCount;
  }

  // format the response
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort < concentration_) {
      for (u32 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      assert(_response->size() > 0);
      // delete the routing extension
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
      delete reinterpret_cast<const std::vector<u32>*>(ri->intermediateAddress);
      delete ri;
      packet->setRoutingExtension(nullptr);
    } else {
      for (u32 vc = baseVc_ + vcSet; vc < baseVc_ + numVcs_;
           vc += 2 * localDimWidths_.size()
               + 2 * globalDimWidths_.size()) {
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> ProgressiveAdaptiveRoutingAlgorithm::routing(
    Flit* _flit, const std::vector<u32>& _destinationAddress) const {
  // ex: [1,...,m,1,...,n]
  const std::vector<u32>& routerAddress = router_->address();
  Packet* packet = _flit->packet();
  u32 globalDimensions = globalDimWidths_.size();
  u32 localDimensions = localDimWidths_.size();

  // determine if already at destination virtual global router
  u32 globalDim;
  u32 globalPortBase = 0;
  for (globalDim = 0; globalDim < globalDimensions; globalDim++) {
    if (routerAddress.at(localDimensions + globalDim)
        != _destinationAddress.at(localDimensions + globalDim + 1)) {
      break;
    }
    globalPortBase += ((globalDimWidths_.at(globalDim) - 1)
                       * globalDimWeights_.at(globalDim));
  }

  std::vector<u32> globalOutputPorts;
  std::unordered_set<u32> outputPorts;

  RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());

  // within first non-dst group
  if (ri->globalHopCount == 0 && globalDim != globalDimensions) {
    // choose a random local dst
    if (ri->localDst == nullptr) {
      std::vector<u32> diffGlobalDims;
      diffGlobalDims.push_back(globalDim);
      setLocalDst(diffGlobalDims, _destinationAddress, &globalOutputPorts,
                  _flit, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
    }

    // UGAL
    std::unordered_set<u32> MINOutputPorts = DimOrderRoutingAlgorithm::routing(
        _flit, _destinationAddress);
    std::unordered_set<u32> VALOutputPorts = ValiantRoutingAlgorithm::routing(
        _flit, _destinationAddress);
    // choose random port to evaluate queue size
    int MINOutputSize = MINOutputPorts.size();
    assert(MINOutputSize > 0);
    int MINRandom = gSim->rnd.nextU64(0, MINOutputSize - 1);
    auto MINIt = MINOutputPorts.begin();
    std::advance(MINIt, MINRandom);
    int MINOutputPort = *(MINIt);
    f64 MINAvailability = 0.0;
    for (u32 vc = packet->getHopCount() - 1; vc < numVcs_;
         vc += 2 * localDimensions + 2 * globalDimensions) {
      MINAvailability += router_->congestionStatus(MINOutputPort, vc);
    }
    int VALOutputSize = VALOutputPorts.size();
    assert(VALOutputSize > 0);
    int VALRandom = gSim->rnd.nextU64(0, VALOutputSize - 1);
    auto VALIt = VALOutputPorts.begin();
    std::advance(VALIt, VALRandom);
    int VALOutputPort = *(VALIt);
    f64 VALAvailability = 0.0;
    for (u32 vc = packet->getHopCount() - 1; vc < numVcs_;
         vc += 2 * localDimensions + 2 * globalDimensions) {
      VALAvailability += router_->congestionStatus(VALOutputPort, vc);
    }
    // UGAL
    if (MINAvailability <= 2 * VALAvailability) {
      outputPorts = MINOutputPorts;
    } else {
      // switch to valiant
      outputPorts = VALOutputPorts;
      ri->valiantMode = true;
    }

  } else {
    // not in first group, just use dimension order
    return DimOrderRoutingAlgorithm::routing(
        _flit, _destinationAddress);
  }
  assert(outputPorts.size() >= 1);
  return outputPorts;
}

}  // namespace HierarchicalHyperX
