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
#include "network/hierarchicalhyperx/GlobalThresholdProgressiveAdaptiveRoutingAlgorithm.h"

#include <strop/strop.h>
#include <cassert>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"


namespace HierarchicalHyperX {

GlobalThresholdProgressiveAdaptiveRoutingAlgorithm::
  GlobalThresholdProgressiveAdaptiveRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter,
    f64 _threshold)
  : DimOrderRoutingAlgorithm(_name, _parent, _router,
     _latency, _baseVc, _numVcs, _globalDimensionWidths,
     _globalDimensionWeights, _localDimensionWidths, _localDimensionWeights,
     _concentration, _globalLinksPerRouter), threshold_(_threshold) {
  // every VC per hop
  assert(numVcs_ >= localDimWidths_.size() * (globalDimWidths_.size() + 1) + 1
         + globalDimWidths_.size() + 1 + 1);
}

GlobalThresholdProgressiveAdaptiveRoutingAlgorithm::
  ~GlobalThresholdProgressiveAdaptiveRoutingAlgorithm() {
}

void GlobalThresholdProgressiveAdaptiveRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->packet()->message()->getDestinationAddress();
  const std::vector<u32>& routerAddress = router_->address();
  Packet* packet = _flit->packet();
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
    for (u32 localDim = 0; localDim < localDimensions; localDim++) {
      ri->localDst->at(localDim) = routerAddress.at(localDim);
    }
    u32 globalPort =  gSim->rnd.nextU64(0, globalLinksPerRouter_ - 1);
    ri->localDstPort->clear();
    ri->localDstPort->push_back(globalPort);
    packet->setRoutingExtension(ri);

    for (auto itr = ri->localDstPort->begin();
         itr != ri->localDstPort->end(); itr++) {
      u32 portBase = getPortBase(concentration_, localDimWidths_,
                                 localDimWeights_);
      bool res = outputPorts.insert(portBase + *itr).second;
      (void)res;
      assert(res);
    }
  } else {
    // routing depends on mode
    outputPorts = GlobalThresholdProgressiveAdaptiveRoutingAlgorithm::routing(
        _flit, *destinationAddress);
  }
  assert(outputPorts.size() >= 1);

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
  vcSet = packet->getHopCount() - 1;

  // format the response
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort < concentration_) {
      for (u32 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      assert(_response->size() > 0);
      delete ri;
      packet->setRoutingExtension(nullptr);
    } else {
      for (u32 vc = baseVc_ + vcSet; vc < baseVc_ + numVcs_;
           vc += localDimWidths_.size()
               * (globalDimWidths_.size() + 1) + 1
               + globalDimWidths_.size() + 1 + 1) {
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> GlobalThresholdProgressiveAdaptiveRoutingAlgorithm::
  routing(Flit* _flit, const std::vector<u32>& _destinationAddress) const {
  // ex: [1,...,m,1,...,n]
  const std::vector<u32>& routerAddress = router_->address();
  Packet* packet = _flit->packet();
  u32 globalDimensions = globalDimWidths_.size();
  u32 localDimensions = localDimWidths_.size();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 dim = 0; dim < localDimensions; dim++) {
    numRoutersPerGlobalRouter *= localDimWidths_.at(dim);
  }

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
            f64 vcStatus = router_->congestionStatus(outputPort, vc);
            availability += vcStatus;
            vcCount++;
          }
          availability = availability / vcCount;
          // availabitlity = 1 means fully congested
          // this means the global link shouldn't be used
          if (availability >= threshold_) {
            // switch to valiant
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
    return DimOrderRoutingAlgorithm::routing(_flit, _destinationAddress);
  }
  assert(outputPorts.size() >= 1);
  return outputPorts;
}

}  // namespace HierarchicalHyperX
