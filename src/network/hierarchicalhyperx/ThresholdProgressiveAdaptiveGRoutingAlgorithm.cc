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
#include "network/hierarchicalhyperx/ThresholdProgressiveAdaptiveGRoutingAlgorithm.h"
#include <strop/strop.h>
#include <cassert>

#include <unordered_set>
#include <unordered_map>
#include <set>
#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"


namespace HierarchicalHyperX {

ThresholdProgressiveAdaptiveGRoutingAlgorithm::
ThresholdProgressiveAdaptiveGRoutingAlgorithm(
    const std::string& _name, const Component* _parent, u64 _latency,
    Router* _router, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter,
    f64 _threshold)
  : DimOrderRoutingAlgorithm
    (_name, _parent, _latency, _router, _numVcs, _globalDimensionWidths,
     _globalDimensionWeights, _localDimensionWidths, _localDimensionWeights,
     _concentration, _globalLinksPerRouter), threshold_(_threshold) {
  // every VC per hop
  assert(numVcs_ >= localDimWidths_.size() * (globalDimWidths_.size() + 1) + 1
         + globalDimWidths_.size() + 1 + 1);
}

ThresholdProgressiveAdaptiveGRoutingAlgorithm::
~ThresholdProgressiveAdaptiveGRoutingAlgorithm() {
}

void ThresholdProgressiveAdaptiveGRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->getPacket()->getMessage()->getDestinationAddress();
  const std::vector<u32>& routerAddress = router_->getAddress();
  Packet* packet = _flit->getPacket();
  u32 localDimensions = localDimWidths_.size();

  std::unordered_set<u32> outputPorts;

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

  // reset local dst after switching to Valiant mode in first group
  if (ri->globalHopCount == 0 && ri->valiantMode == true) {
    std::vector<u32>* localRouter = new std::vector<u32>(localDimensions);
    for (u32 localDim = 0; localDim < localDimensions; localDim++) {
      localRouter->at(localDim) = routerAddress.at(localDim);
    }
    ri->localDst = localRouter;
    u32 globalPort =  gSim->rnd.nextU64(0, globalLinksPerRouter_ - 1);
    std::vector<u32>* dstPort = new std::vector<u32>;
    dstPort->push_back(globalPort);
    ri->localDstPort = dstPort;
    packet->setRoutingExtension(ri);

    for (auto itr = dstPort->begin();
         itr != dstPort->end(); itr++) {
      u32 portBase = getPortBase(concentration_, localDimWidths_,
                                 localDimWeights_);
      bool res = outputPorts.insert(portBase + *itr).second;
      (void)res;
      assert(res);
    }
  } else {
    // routing depends on mode
    outputPorts = ThresholdProgressiveAdaptiveGRoutingAlgorithm::routing(
        _flit, destinationAddress);
  }
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

  // figure out which VC set to use
  u32 vcSet;
  vcSet = packet->getHopCount() - 1;

  // format the response
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort < concentration_) {
      for (u32 vc = 0; vc < numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      assert(_response->size() > 0);
      delete ri;
      packet->setRoutingExtension(nullptr);
    } else {
      for (u32 vc = vcSet; vc < numVcs_; vc += localDimWidths_.size()
               * (globalDimWidths_.size() + 1) + 1
               + globalDimWidths_.size() + 1 + 1) {
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> ThresholdProgressiveAdaptiveGRoutingAlgorithm::routing(
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
      // in first group, determine if congested
      if (ri->globalHopCount == 0 && ri->valiantMode == false) {
        for (auto itr = localDstPort->begin();
             itr != localDstPort->end(); itr++) {
          // make sure we are reffering to the right outputPort
          u32 outputPort = *itr + getPortBase(concentration_, localDimWidths_,
                                              localDimWeights_);
          f64 availability = 0.0;
          u32 vcCount = 0;
          for (u32 vc = packet->getHopCount() - 1; vc < numVcs_;
               vc += localDimWidths_.size() * (globalDimWidths_.size() + 1) + 1
                   + globalDimWidths_.size() + 1 + 1) {
            // u32 vcIdx = router_->vcIndex(outputPort, vc);
            f64 vcStatus = router_->congestionStatus(outputPort, vc);
            availability += vcStatus;
            vcCount++;
          }
          availability = availability / vcCount;
          if (availability >= threshold_) {
            // reset localdst for valiant
            ri->localDst = nullptr;
            ri->localDstPort = nullptr;
            ri->valiantMode = true;
            packet->setRoutingExtension(ri);
            // pick a random local port
            u32 localPort = gSim->rnd.nextU64(concentration_, getPortBase(
                concentration_, localDimWidths_, localDimWeights_) - 1);
            // send using the local port
            bool res = outputPorts.insert(localPort).second;
            (void)res;
            // assert(res);
          }
        }
        // not congested
        if (outputPorts.size() == 0) {
          for (auto itr = localDstPort->begin();
               itr != localDstPort->end(); itr++) {
            u32 portBase = getPortBase(concentration_, localDimWidths_,
                                       localDimWeights_);
            bool res = outputPorts.insert(portBase + *itr).second;
            (void)res;
            assert(res);
          }
        }
      } else {
        // set output ports to those links
        for (auto itr = localDstPort->begin();
             itr != localDstPort->end(); itr++) {
          u32 portBase = getPortBase(concentration_, localDimWidths_,
                                     localDimWeights_);
          bool res = outputPorts.insert(portBase + *itr).second;
          (void)res;
          assert(res);
        }
      }
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
