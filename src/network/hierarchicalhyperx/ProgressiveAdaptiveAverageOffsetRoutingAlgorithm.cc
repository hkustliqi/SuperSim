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
#include "network/hierarchicalhyperx/ProgressiveAdaptiveAverageOffsetRoutingAlgorithm.h"

#include <strop/strop.h>

#include <cassert>
#include <unordered_set>
#include <unordered_map>
#include <set>

#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"

namespace HierarchicalHyperX {

ProgressiveAdaptiveAverageOffsetRoutingAlgorithm::
  ProgressiveAdaptiveAverageOffsetRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter,
    bool _randomGroup)
    : ValiantRoutingAlgorithm
      (_name, _parent,  _router, _latency, _baseVc, _numVcs,
       _globalDimensionWidths,
       _globalDimensionWeights, _localDimensionWidths, _localDimensionWeights,
       _concentration, _globalLinksPerRouter, _randomGroup) {
  assert(numVcs_ >= 2 * globalDimWidths_.size() + 3);
}

ProgressiveAdaptiveAverageOffsetRoutingAlgorithm::
  ~ProgressiveAdaptiveAverageOffsetRoutingAlgorithm() {}

void ProgressiveAdaptiveAverageOffsetRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->packet()->message()->getDestinationAddress();
  Packet* packet = _flit->packet();
  const std::vector<u32>& routerAddress = router_->address();

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
    outputPorts = ProgressiveAdaptiveAverageOffsetRoutingAlgorithm::routing
      (_flit, *destinationAddress);
    assert(outputPorts.size() >= 1);
  } else {
    // use Valiant routing
    if (ri->intermediateAddress == nullptr) {
      std::vector<u32>* re = new std::vector<u32>(1 + routerAddress.size());
      u32 rnd = setIntermediateAdd(re);
      ri->intermediateAddress = re;
      std::vector<u32>* newLocalDstPort = new std::vector<u32>;
      newLocalDstPort->push_back(rnd);
      // printf("Intermediate address is %s \n",
      //   strop::vecString<u32>(*re).c_str());

      // reset LocalDst to current routerAddress
      assert(ri->localDst != nullptr);
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
      std::vector<u32>* newLocalDst = new std::vector<u32> (
                                        localDimWidths_.size());
      for (u32 i = 0; i < localDimWidths_.size(); i++) {
        newLocalDst->at(i) = routerAddress.at(i);
      }
      ri->localDst = newLocalDst;
      ri->localDstPort = newLocalDstPort;
    }

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
    vcSet = 1 + ri->globalHopCount;
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
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32>
  ProgressiveAdaptiveAverageOffsetRoutingAlgorithm::routing(
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
    int MINOutputPort = *(MINIt);
    f64 MINAvailability = 0.0;
    for (u32 vc = baseVc_; vc < baseVc_ + numVcs_;
         vc += 2 * globalDimensions + 3) {
      MINAvailability += router_->congestionStatus(MINOutputPort, vc);
    }

    std::unordered_set<u32> NonMINOutputPorts;
    u32 maxPort = getPortBase(concentration_, localDimWidths_,
                              localDimWeights_);
    // maxPort += GlobalLinksPerRouter_;
    for (u32 localPort = concentration_; localPort < maxPort; localPort++) {
      NonMINOutputPorts.insert(localPort);
    }
    for (auto itr = MINOutputPorts.begin();
         itr != MINOutputPorts.end(); itr++) {
      NonMINOutputPorts.erase(*itr);
    }
    f64 AverageAvailability = 0.0;
    int NonMINOutputSize = NonMINOutputPorts.size();
    assert(NonMINOutputSize > 0);
    for (auto NonMINIt = NonMINOutputPorts.begin(); NonMINIt !=
           NonMINOutputPorts.end(); NonMINIt++) {
      int NonMINOutputPort = *(NonMINIt);
      for (u32 vc = baseVc_; vc < baseVc_ + numVcs_;
           vc += 2 * globalDimensions + 3) {
        AverageAvailability += router_->congestionStatus(NonMINOutputPort, vc);
      }
    }
    AverageAvailability /= NonMINOutputSize;

    std::vector<u32> dstRouterAdd(_destinationAddress);
    dstRouterAdd.erase(dstRouterAdd.begin());
    std::vector<u32> intermediateAdd(_destinationAddress);
    u32 MINPathLen = getHopDistance(routerAddress, dstRouterAdd,
      localDimWidths_, globalDimWidths_, globalDimWeights_);
    setIntermediateAdd(&intermediateAdd);
    intermediateAdd.erase(intermediateAdd.begin());
    u32 NonMINPathLen = 0;
    NonMINPathLen += getHopDistance(routerAddress, intermediateAdd,
      localDimWidths_, globalDimWidths_, globalDimWeights_);
    NonMINPathLen += getHopDistance(intermediateAdd, dstRouterAdd,
    localDimWidths_, globalDimWidths_, globalDimWeights_);
    // UGAL with average queue value and danymic offset
    if (MINAvailability * MINPathLen <=
        AverageAvailability * (NonMINPathLen + 0.01)) {
      outputPorts = MINOutputPorts;
    } else {
      // switch to valiant
      outputPorts = NonMINOutputPorts;
      ri->valiantMode = true;
      // printf("Router address is %s \n",
      //   strop::vecString<u32>(routerAddress).c_str());
      // printf("switched to valiant, MINAvai = %f, NonMINAV = %f \n",
      //       MINAvailability, NonMINAvailability);
    }

  } else {
    // not in first group, just use dimension order
    return DimOrderRoutingAlgorithm::routing(
        _flit, _destinationAddress);
  }
  assert(outputPorts.size() >= 1);
  return outputPorts;
}

u32 ProgressiveAdaptiveAverageOffsetRoutingAlgorithm::setIntermediateAdd(
    std::vector<u32>* re) const {
  const std::vector<u32>& routerAddress = router_->address();

  re->at(0) = U32_MAX;  // dummy
  // random intermediate local address
  for (u32 idx = 0; idx < localDimWidths_.size(); idx++) {
    re->at(idx + 1) = gSim->rnd.nextU64(0, localDimWidths_.at(idx) - 1);
  }
  u32 globalPort = 0;
  u32 globalPortBase = 1;
  for (u32 idx = 0; idx < localDimWidths_.size(); idx++) {
    globalPort += routerAddress.at(idx) * globalPortBase;
    globalPortBase *= localDimWidths_.at(idx);
  }
  std::vector<u32> globalPorts;
  for (u32 i = 0; i < globalLinksPerRouter_; i++) {
    globalPorts.push_back(globalPort + i * globalPortBase);
  }
  u32 rnd = gSim->rnd.nextU64(0, globalPorts.size()-1);
  u32 randomGlobalPort = globalPorts.at(rnd);

  // global source router
  std::vector<u32> srcGlobalAddress(routerAddress.begin()
    + localDimWidths_.size(), routerAddress.end());
  // determine RE global address from random port
  u32 virtualGlobalPortBase = 0;
  bool globalDstSet = false;

  for (u32 globalDim = 0; globalDim < globalDimWidths_.size();
       globalDim++) {
    u32 globalDimWidth = globalDimWidths_.at(globalDim);
    u32 globalDimWeight = globalDimWeights_.at(globalDim);
    std::vector<u32> dstGlobalAddress(srcGlobalAddress);
    for (u32 offset = 1; offset < globalDimWidth; offset++) {
      dstGlobalAddress.at(globalDim) = (srcGlobalAddress.at(globalDim)
                                        + offset) % globalDimWidth;
      for (u32 weight = 0; weight < globalDimWeight; weight++) {
        // determine the vitual port of global router
        u32 virtualGlobalSrcPort = virtualGlobalPortBase
          + ((offset - 1) * globalDimWeight)
          + weight;
        if (virtualGlobalSrcPort == randomGlobalPort) {
          // intermediate global address should be linked to
          // current global link
          for (u32 idx = 0; idx < globalDimWidths_.size(); idx++) {
            re->at(idx + localDimWidths_.size() + 1) =
              dstGlobalAddress.at(idx);
          }
          globalDstSet = true;
        }
      }
    }
    virtualGlobalPortBase += ((globalDimWidth - 1) * globalDimWeight);
  }
  assert(globalDstSet == true);
  return rnd;
}
}  // namespace HierarchicalHyperX
