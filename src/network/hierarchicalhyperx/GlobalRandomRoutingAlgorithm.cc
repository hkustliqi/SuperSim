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
#include "network/hierarchicalhyperx/GlobalRandomRoutingAlgorithm.h"
#include <strop/strop.h>
#include <cassert>

#include <unordered_set>
#include <set>
#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"


namespace HierarchicalHyperX {

GlobalRandomRoutingAlgorithm::GlobalRandomRoutingAlgorithm(
    const std::string& _name, const Component* _parent, u64 _latency,
    Router* _router, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter)
    : RoutingAlgorithm(_name, _parent, _router, _latency),
      numVcs_(_numVcs), globalDimWidths_(_globalDimensionWidths),
      globalDimWeights_(_globalDimensionWeights),
      localDimWidths_(_localDimensionWidths),
      localDimWeights_(_localDimensionWeights),
      concentration_(_concentration),
      globalLinksPerRouter_(_globalLinksPerRouter) {
  assert(numVcs_ >= globalDimWidths_.size() + 1);
}

GlobalRandomRoutingAlgorithm::~GlobalRandomRoutingAlgorithm() {}

void GlobalRandomRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->getPacket()->getMessage()->getDestinationAddress();
  Packet* packet = _flit->getPacket();
  // perform routing
  std::unordered_set<u32> outputPorts = routing(_flit, *destinationAddress);
  assert(outputPorts.size() >= 1);
  if (*outputPorts.begin() >= getPortBase(concentration_, localDimWidths_,
                                          localDimWeights_)) {
    RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
        packet->getRoutingExtension());
    ri->globalHopCount++;
    // delete local router
    ri->localDst = nullptr;
    ri->localDstPort = nullptr;
    packet->setRoutingExtension(ri);
  }
  // figure out which VC set to use
  u32 vcSet = packet->getHopCount() - 1;

  // format the response
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort < concentration_) {
      for (u32 vc = 0; vc < numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      packet->setRoutingExtension(nullptr);
    } else {
      // select VCs in the corresponding set
      for (u32 vc = vcSet; vc < numVcs_; vc += (globalDimWidths_.size() + 1)
               * localDimWidths_.size() + globalDimWidths_.size()) {
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> GlobalRandomRoutingAlgorithm::routing
  (Flit* _flit, const std::vector<u32>& destinationAddress) {
  // ex: [1,...,m,1,...,n]
  const std::vector<u32>& routerAddress = router_->getAddress();
  assert(routerAddress.size() == destinationAddress.size() - 1);

  Packet* packet = _flit->getPacket();

  u32 globalDimensions = globalDimWidths_.size();
  u32 localDimensions = localDimWidths_.size();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 tmp = 0; tmp < localDimensions; tmp++) {
    numRoutersPerGlobalRouter *= localDimWidths_.at(tmp);
  }

  // determine if already at destination virtual global router
  std::vector<u32> diffGlobalDims;
  assert(diffGlobalDims.size() == 0);
  bool atGlobalDst = true;
  for (u32 globalDim = 0; globalDim < globalDimensions; globalDim++) {
    if (routerAddress.at(localDimensions + globalDim)
        != destinationAddress.at(localDimensions + globalDim + 1)) {
      diffGlobalDims.push_back(globalDim);
      atGlobalDst = false;
    }
  }

  // first perform routing at the global level
  std::vector<u32> globalOutputPorts;
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

  // if at different global router
  if (atGlobalDst == false) {
    if (ri->localDst == nullptr) {
      setLocalDst(&diffGlobalDims, destinationAddress, &globalOutputPorts,
                  _flit, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
    }

    const std::vector<u32>* localDst =
        reinterpret_cast<const std::vector<u32>*>(ri->localDst);
    const std::vector<u32>* localDstPort =
        reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);

    u32 portBase = getPortBase(concentration_, localDimWidths_,
                               localDimWeights_);
    // if router has a global link to destination global router
    if (std::equal(localDst->begin(), localDst->end(),
                   routerAddress.begin())) {
      // set output ports to those links
      for (auto itr = localDstPort->begin();
           itr != localDstPort->end(); itr++) {
        bool res = outputPorts.insert(portBase + *itr).second;
        (void)res;
        assert(res);
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
    }
  } else {
    // if at the same global virtual router
    // use the regular dimension order routing of HyperX
    ri->localDst = nullptr;
    ri->localDstPort = nullptr;
    packet->setRoutingExtension(ri);
    // determine the next local dimension to work on
    u32 localDim;
    u32 portBase = concentration_;
    for (localDim = 0; localDim < localDimensions; localDim++) {
      if (routerAddress.at(localDim) != destinationAddress.at(localDim+1)) {
        break;
      }
      portBase += ((localDimWidths_.at(localDim) - 1)
                   * localDimWeights_.at(localDim));
    }
    // test if already at destination local router
    if (localDim == localDimensions) {
      bool res = outputPorts.insert(destinationAddress.at(0)).second;
      (void)res;
      assert(res);
    } else {
      // more local router-to-router hops needed
      u32 src = routerAddress.at(localDim);
      u32 dst = destinationAddress.at(localDim + 1);
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
    }
  }
  return outputPorts;
}

}  // namespace HierarchicalHyperX
