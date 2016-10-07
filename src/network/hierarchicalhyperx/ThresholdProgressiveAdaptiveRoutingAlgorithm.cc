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
#include "network/hierarchicalhyperx/ThresholdProgressiveAdaptiveRoutingAlgorithm.h"
#include <strop/strop.h>
#include <cassert>

#include <unordered_set>
#include <unordered_map>
#include <set>
#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"

namespace HierarchicalHyperX {

ThresholdProgressiveAdaptiveRoutingAlgorithm::
ThresholdProgressiveAdaptiveRoutingAlgorithm(
    const std::string& _name, const Component* _parent, u64 _latency,
    Router* _router, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter,
    f64 _threshold, bool _randomGroup)
    : ValiantRoutingAlgorithm(_name, _parent,
                              _latency, _router, _numVcs, _globalDimensionWidths,
                              _globalDimensionWeights, _localDimensionWidths, _localDimensionWeights,
                              _concentration, _globalLinksPerRouter, _randomGroup),
      threshold_(_threshold) {
  assert(numVcs_ >= 2 * localDimWidths_.size() + 2 * globalDimWidths_.size());
}

ThresholdProgressiveAdaptiveRoutingAlgorithm::
~ThresholdProgressiveAdaptiveRoutingAlgorithm() {}

void ThresholdProgressiveAdaptiveRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->getPacket()->getMessage()->getDestinationAddress();
  Packet* packet = _flit->getPacket();

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
    outputPorts = ThresholdProgressiveAdaptiveRoutingAlgorithm::routing(_flit,
                                                                        destinationAddress);
    assert(outputPorts.size() >= 1);
  } else {
    // use Valiant routing
    outputPorts = ValiantRoutingAlgorithm::routing(
        _flit, destinationAddress);
    assert(outputPorts.size() >= 1);
  }

  // reset localDst once in a new group
  if (*outputPorts.begin() >= getPortBase(concentration_, localDimWidths_,
                                          localDimWeights_)) {
    ri->globalHopCount++;
    // delete local router
    ri->localDst = nullptr;
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
  bool switchedToValiant = false;
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort < concentration_) {
      for (u32 vc = 0; vc < numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      assert(_response->size() > 0);
      // delete the routing extension
      delete ri;
      packet->setRoutingExtension(nullptr);
    } else {
      // check for congestion
      if (ri->valiantMode == false && ri->globalHopCount == 0) {
        f64 availability = 0.0;
        u32 vcCount = 0;
        for (u32 vc = vcSet; vc < numVcs_; vc += 2 * localDimWidths_.size()
                 + 2 * globalDimWidths_.size()) {
          // u32 vcIdx = router_->vcIndex(outputPort, vc);
          availability += router_->congestionStatus(outputPort, vc);
          vcCount++;
        }
        availability = availability / vcCount;
        if (availability >= threshold_) {
          // reset localdst for valiant
          ri->localDst = nullptr;
          ri->localDstPort = nullptr;
          ri->valiantMode = true;
          packet->setRoutingExtension(ri);
          switchedToValiant = true;
        }
      }
    }
  }
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort >= concentration_) {
      if (switchedToValiant == false) {
        for (u32 vc = vcSet; vc < numVcs_; vc += 2 * localDimWidths_.size()
                 + 2 * globalDimWidths_.size()) {
          _response->add(outputPort, vc);
        }
      }
    }
  }
  if (switchedToValiant == true) {
    outputPorts = ValiantRoutingAlgorithm::routing(
        _flit, destinationAddress);
    assert(outputPorts.size() >= 1);
    // reset localDst once in a new group
    if (*outputPorts.begin() >= getPortBase(concentration_, localDimWidths_,
                                            localDimWeights_)) {
      ri->globalHopCount++;
      // delete local router
      ri->localDst = nullptr;
      ri->localDstPort = nullptr;
      packet->setRoutingExtension(ri);
    }
    if (ri->globalHopCount == 0) {
      vcSet = packet->getHopCount() - 1;
    } else {
      vcSet = 2*localDimWidths_.size() - 1 + ri->globalHopCount;
    }
    for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
      u32 outputPort = *it;
      if (outputPort >= concentration_) {
        for (u32 vc = vcSet; vc < numVcs_; vc += 2 * localDimWidths_.size()
                 + 2 * globalDimWidths_.size()) {
          _response->add(outputPort, vc);
        }
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> ThresholdProgressiveAdaptiveRoutingAlgorithm::routing(
    Flit* _flit, const std::vector<u32>* destinationAddress) {
  // ex: [1,...,m,1,...,n]
  const std::vector<u32>& routerAddress = router_->getAddress();
  Packet* packet = _flit->getPacket();
  u32 globalDimensions = globalDimWidths_.size();
  u32 localDimensions = localDimWidths_.size();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 tmp = 0; tmp < localDimensions; tmp++) {
    numRoutersPerGlobalRouter *= localDimWidths_.at(tmp);
  }

  // determine if already at destination virtual global router
  u32 globalDim;
  u32 globalPortBase = 0;
  for (globalDim = 0; globalDim < globalDimensions; globalDim++) {
    if (routerAddress.at(localDimensions + globalDim)
        != destinationAddress->at(localDimensions + globalDim + 1)) {
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
      std::vector<u32>* diffGlobalDims = new std::vector<u32>;
      diffGlobalDims->push_back(globalDim);
      setLocalDst(diffGlobalDims, destinationAddress, &globalOutputPorts,
                  _flit, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
    }
    const std::vector<u32>* localDst =
        reinterpret_cast<const std::vector<u32>*>(ri->localDst);
    const std::vector<u32>* localDstPort =
        reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);

    // if router has a global link to destination global router
    if (std::equal(localDst->begin(), localDst->end(),
                   routerAddress.begin())) {
      u32 portBase = getPortBase(concentration_, localDimWidths_,
                                 localDimWeights_);
      // find if congested
      for (auto itr = localDstPort->begin();
           itr != localDstPort->end(); itr++) {
        f64 availability = 0.0;
        u32 vcCount = 0;
        for (u32 vc = packet->getHopCount() - 1; vc < numVcs_;
             vc += 2 * localDimensions + 2 * globalDimensions) {
          // u32 vcIdx = router_->vcIndex(portBase + *itr, vc);
          availability += router_->congestionStatus(portBase + *itr, vc);
          vcCount++;
        }
        availability = availability / vcCount;
        // port not congested
        if (availability < threshold_) {
          bool res = outputPorts.insert(portBase + *itr).second;
          (void)res;
          assert(res);
        }
      }
      // congested, go Valiant
      if (outputPorts.size() == 0) {
        ri->valiantMode = true;
        // reset localdst for Valiant
        ri->localDst = nullptr;
        ri->localDstPort = nullptr;
        packet->setRoutingExtension(ri);
        outputPorts = ValiantRoutingAlgorithm::routing(
            _flit, destinationAddress);
      }
      assert(outputPorts.size() >= 1);
    } else {
      // determine the next local dimension to work on
      u32 localDim;
      u32 portBase = concentration_;
      for (localDim = 0; localDim < localDimensions; localDim++) {
        if (routerAddress.at(localDim) != localDst->at(localDim)) {
          break;
        }
        portBase += ((localDimWidths_.at(localDim) - 1)
                     * localDimWeights_.at(localDim));
      }
      // more local router-to-router hops needed
      u32 src = routerAddress.at(localDim);
      u32 dst = localDst->at(localDim);
      if (dst < src) {
        dst += localDimWidths_.at(localDim);
      }
      u32 offset = (dst - src - 1) * localDimWeights_.at(localDim);
      // add all ports where the two routers are connecting
      for (u32 weight = 0; weight < localDimWeights_.at(localDim);
           weight++) {
        bool res = outputPorts.insert(portBase + offset + weight).second;
        (void)res;
        assert(res);
      }
      assert(outputPorts.size() >= 1);
    }
  } else {
    // not in first group, just use dimension order
    return DimOrderRoutingAlgorithm::routing(
        _flit, destinationAddress);
  }
  assert(outputPorts.size() >= 1);
  return outputPorts;
}

}  // namespace HierarchicalHyperX
