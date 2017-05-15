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
#include "network/hierarchicalhyperx/ProgressiveAdaptiveRandomRoutingAlgorithm.h"

#include <strop/strop.h>

#include <cassert>
#include <unordered_set>
#include <unordered_map>
#include <set>

#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"

namespace HierarchicalHyperX {
  u32 ProgressiveAdaptiveRandomRoutingAlgorithm::numMinimalRoutes_;
  u32 ProgressiveAdaptiveRandomRoutingAlgorithm::numNonMinimalRoutes_;
ProgressiveAdaptiveRandomRoutingAlgorithm::
  ProgressiveAdaptiveRandomRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter,
    bool _randomGroup, f64 _bias, f64 _threshold)
    : ValiantRoutingAlgorithm
      (_name, _parent,  _router, _latency, _baseVc, _numVcs,
       _globalDimensionWidths,
       _globalDimensionWeights, _localDimensionWidths, _localDimensionWeights,
       _concentration, _globalLinksPerRouter, _randomGroup),
      bias_(_bias), threshold_(_threshold) {
  assert(numVcs_ >= 2 * globalDimWidths_.size() + 3);
  ProgressiveAdaptiveRandomRoutingAlgorithm::numMinimalRoutes_ = 0;
  ProgressiveAdaptiveRandomRoutingAlgorithm::numNonMinimalRoutes_ = 0;
}

ProgressiveAdaptiveRandomRoutingAlgorithm::
  ~ProgressiveAdaptiveRandomRoutingAlgorithm() {
}

void ProgressiveAdaptiveRandomRoutingAlgorithm::processRequest(
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
    outputPorts = ProgressiveAdaptiveRandomRoutingAlgorithm::routing
      (_flit, *destinationAddress);
    assert(outputPorts.size() >= 1);
  } else {
    // use Valiant routing
    assert(ri->intermediateAddress != nullptr);

    outputPorts = ValiantRoutingAlgorithm::routing(
                    _flit, *destinationAddress, randomGroup_);
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
    if (ri->valiantMode == false) {
      vcSet = 0;
    } else {
      vcSet = 1;
    }
  } else {
    if (ri->valiantMode == true) {
      if (ri->intermediateDone == false) {
        vcSet = 1 + ri->globalHopCount;
      } else {
        vcSet = 2 + ri->globalHopCount;
      }
    } else {
      vcSet = 2 + globalDimWidths_.size() + ri->globalHopCount;
    }
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
           vc += 2 * globalDimWidths_.size() + 3) {
        _response->add(outputPort, vc);      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> ProgressiveAdaptiveRandomRoutingAlgorithm::routing(
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
                  packet, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
    }

    // UGAL
    std::unordered_set<u32> MINOutputPorts = DimOrderRoutingAlgorithm::routing(
        _flit, _destinationAddress);

    // choose random port to evaluate queue size
    int MINOutputSize = MINOutputPorts.size();
    assert(MINOutputSize > 0);
    int MINRandom = gSim->rnd.nextU64(0, MINOutputSize - 1);
    auto MINIt = MINOutputPorts.begin();
    std::advance(MINIt, MINRandom);
    u32 MINOutputPort = *(MINIt);
    f64 MINVCAvailability = 0.0;
    f64 MINAvailability = 0.0;

    // select right vc for evaluation
    if (MINOutputPort >= getPortBase(concentration_, localDimWidths_,
                                     localDimWeights_)) {
      for (u32 vc = baseVc_ + 3 + globalDimWidths_.size();
           vc < baseVc_ + numVcs_; vc += 2 * globalDimWidths_.size() + 3) {
        MINVCAvailability += router_->congestionStatus(MINOutputPort, vc);
      }
    } else {
      for (u32 vc = baseVc_; vc < baseVc_ + numVcs_;
           vc += 2 * globalDimWidths_.size() + 3) {
        MINVCAvailability += router_->congestionStatus(MINOutputPort, vc);
      }
    }
    for (u32 vc = baseVc_; vc < baseVc_ + numVcs_; vc += 1) {
      MINAvailability += router_->congestionStatus(MINOutputPort, vc);
    }

    // remove local dst for Non MIN routing
    if (ri->localDst != nullptr) {
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
      ri->localDst = nullptr;
      ri->localDstPort = nullptr;
    }
    std::unordered_set<u32> NonMINOutputPorts =
      ValiantRoutingAlgorithm::routing(_flit,
                                       _destinationAddress, randomGroup_);
    assert(NonMINOutputPorts.size() > 0);
    if (ri->localDst != nullptr) {
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
      ri->localDst = nullptr;
      ri->localDstPort = nullptr;
    }
    int NonMINRandom = gSim->rnd.nextU64(0, NonMINOutputPorts.size() - 1);
    auto NonMINIt = NonMINOutputPorts.begin();
    std::advance(NonMINIt, NonMINRandom);
    u32 NonMINOutputPort = *(NonMINIt);
    f64 NonMINAvailability = 0.0;
    f64 NonMINVCAvailability = 0.0;

    if (NonMINOutputPort >= getPortBase(concentration_, localDimWidths_,
                                     localDimWeights_)) {
      for (u32 vc = baseVc_ + 2; vc < baseVc_ + numVcs_;
           vc += 2 * globalDimWidths_.size() + 3) {
        NonMINVCAvailability += router_->congestionStatus(
          NonMINOutputPort, vc);
      }
    } else {
      for (u32 vc = baseVc_ + 1; vc < baseVc_ + numVcs_;
           vc += 2 * globalDimWidths_.size() + 3) {
        NonMINVCAvailability += router_->congestionStatus(
          NonMINOutputPort, vc);
      }
    }
    for (u32 vc = baseVc_ + 1; vc < baseVc_ + numVcs_; vc += 1) {
      NonMINAvailability += router_->congestionStatus(
        NonMINOutputPort, vc);
    }

    std::vector<u32> dstRouterAdd(_destinationAddress);
    dstRouterAdd.erase(dstRouterAdd.begin());
    u32 MINPathLen = getHopDistance(routerAddress, dstRouterAdd,
      localDimWidths_, globalDimWidths_, globalDimWeights_);

    std::vector<u32> intermediateAdd(*(ri->intermediateAddress));
    intermediateAdd.erase(intermediateAdd.begin());
    u32 NonMINPathLen = 0;
    NonMINPathLen += getHopDistance(routerAddress, intermediateAdd,
      localDimWidths_, globalDimWidths_, globalDimWeights_);
    NonMINPathLen += getHopDistance(intermediateAdd, dstRouterAdd,
    localDimWidths_, globalDimWidths_, globalDimWeights_);

    // UGAL
    if ((NonMINOutputPort != MINOutputPort &&
         MINAvailability * MINPathLen  <=
         (NonMINAvailability + 0.01) * NonMINPathLen + bias_)
        || (NonMINOutputPort == MINOutputPort &&
           MINVCAvailability * MINPathLen <=
           (NonMINVCAvailability + 0.01) * NonMINPathLen + bias_)
        ) {
      assert(ri->intermediateAddress != nullptr);
      outputPorts =  MINOutputPorts;
      ProgressiveAdaptiveRandomRoutingAlgorithm::numMinimalRoutes_++;
    } else {
      ri->valiantMode = true;
      // intermediate Add has been set
      assert(ri->intermediateAddress != nullptr);
      // switch to valiant
      outputPorts = NonMINOutputPorts;
      ProgressiveAdaptiveRandomRoutingAlgorithm::numNonMinimalRoutes_++;
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
