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
#include "network/hierarchicalhyperx/GlobalAndLocalRandomRoutingAlgorithm.h"

#include <strop/strop.h>

#include <cassert>
#include <set>
#include <unordered_set>

#include "network/hierarchicalhyperx/util.h"
#include "types/Message.h"
#include "types/Packet.h"

namespace HierarchicalHyperX {

GlobalAndLocalRandomRoutingAlgorithm::GlobalAndLocalRandomRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter)
  : RoutingAlgorithm(_name, _parent, _router, _latency, _baseVc, _numVcs),
      globalDimWidths_(_globalDimensionWidths),
      globalDimWeights_(_globalDimensionWeights),
      localDimWidths_(_localDimensionWidths),
      localDimWeights_(_localDimensionWeights),
      concentration_(_concentration),
      globalLinksPerRouter_(_globalLinksPerRouter) {
  assert(numVcs_ >= (globalDimWidths_.size() + 1) * localDimWidths_.size()
         + globalDimWidths_.size());
}

GlobalAndLocalRandomRoutingAlgorithm::~GlobalAndLocalRandomRoutingAlgorithm() {}

void GlobalAndLocalRandomRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
      _flit->packet()->message()->getDestinationAddress();
  Packet* packet = _flit->packet();

  // perform routing
  std::unordered_set<u32> outputPorts = routing(_flit, *destinationAddress);
  assert(outputPorts.size() >= 1);

  u32 portBase = getPortBase(concentration_, localDimWidths_, localDimWeights_);
  if (*outputPorts.begin() >= portBase) {
    RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
        packet->getRoutingExtension());
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
  // use a new VCset for every hop
  u32 vcSet = packet->getHopCount() - 1;

  // format the response
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    assert(outputPort < portBase + globalLinksPerRouter_);
    if (outputPort < concentration_) {
      for (u32 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      delete reinterpret_cast<RoutingInfo*>(packet->getRoutingExtension());
      packet->setRoutingExtension(nullptr);
    } else {
      // select VCs in the corresponding set
      for (u32 vc = baseVc_ + vcSet; vc < baseVc_ + numVcs_;
           vc += (globalDimWidths_.size() + 1)
               * localDimWidths_.size() + globalDimWidths_.size()) {
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> GlobalAndLocalRandomRoutingAlgorithm::routing
  (Flit* _flit, const std::vector<u32>& _destinationAddress) const {
  // ex: [1,...,m,1,...,n]
  const std::vector<u32>& routerAddress = router_->address();
  assert(routerAddress.size() == _destinationAddress.size() - 1);

  Packet* packet = _flit->packet();

  u32 globalDimensions = globalDimWidths_.size();
  u32 localDimensions = localDimWidths_.size();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 dim = 0; dim < localDimensions; dim++) {
    numRoutersPerGlobalRouter *= localDimWidths_.at(dim);
  }

  // determine if already at destination virtual global router
  std::vector<u32> diffGlobalDims;
  assert(diffGlobalDims.size() == 0);
  bool atGlobalDst = true;
  for (u32 globalDim = 0; globalDim < globalDimensions; globalDim++) {
    if (routerAddress.at(localDimensions + globalDim)
        != _destinationAddress.at(localDimensions + globalDim + 1)) {
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
      // pick a random global dimension
      setLocalDst(diffGlobalDims, _destinationAddress, &globalOutputPorts,
                  packet, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
    }

    const std::vector<u32>* localDst =
        reinterpret_cast<const std::vector<u32>*>(ri->localDst);
    const std::vector<u32>* localDstPort =
        reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);

    u32 portBase = getPortBase(concentration_,
                               localDimWidths_, localDimWeights_);
    // if router has a global link to destination global router
    if (std::equal(localDst->begin(), localDst->end(),
                   routerAddress.begin())) {
      // set output ports to those links
      for (auto itr = localDstPort->begin();
           itr != localDstPort->end(); itr++) {
        assert(*itr < globalLinksPerRouter_);
        bool res = outputPorts.insert(portBase + *itr).second;
        (void)res;
        assert(res);
      }
    } else {
      std::vector<u32> diffLocalDims;
      for (u32 localDim = 0; localDim < localDimensions; localDim++) {
        if (routerAddress.at(localDim) != localDst->at(localDim)) {
          diffLocalDims.push_back(localDim);
        }
      }
      // pick a random local diff dim
      u32 localDim = diffLocalDims.
        at(gSim->rnd.nextU64(0, diffLocalDims.size() - 1));
      u32 portBase = concentration_;
      for (u32 dim = 0; dim < localDim; dim++) {
        portBase += ((localDimWidths_.at(dim) - 1)
                     * localDimWeights_.at(dim));
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
    // select a random local different dimension to resolve
    if (ri->localDst != nullptr) {
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
    }
    ri->localDst = nullptr;
    if (ri->localDstPort != nullptr) {
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
    }
    ri->localDstPort = nullptr;
    packet->setRoutingExtension(ri);
    // determine the local dimension to work on
    bool atDst = true;
    std::vector<u32> diffLocalDims;
    for (u32 localDim = 0; localDim < localDimensions; localDim++) {
      if (routerAddress.at(localDim) != _destinationAddress.at(localDim+1)) {
        diffLocalDims.push_back(localDim);
        atDst = false;
      }
    }
    // test if already at destination local router
    if (atDst == true) {
      bool res = outputPorts.insert(_destinationAddress.at(0)).second;
      (void)res;
      assert(res);
    } else {
      // pick a random local diff dim
      u32 localDim = diffLocalDims.
        at(gSim->rnd.nextU64(0, diffLocalDims.size() - 1));
      u32 portBase = concentration_;
      for (u32 dim = 0; dim < localDim; dim++) {
        portBase += ((localDimWidths_.at(dim) - 1)
                     * localDimWeights_.at(dim));
      }
      // more local router-to-router hops needed
      u32 src = routerAddress.at(localDim);
      u32 dst = _destinationAddress.at(localDim + 1);
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
