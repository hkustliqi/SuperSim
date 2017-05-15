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
#include "network/hierarchicalhyperx/MinimalAdaptiveRoutingAlgorithm.h"

#include <strop/strop.h>

#include <cassert>
#include <set>
#include <unordered_set>
#include <unordered_map>

#include "types/Message.h"
#include "types/Packet.h"
#include "network/hierarchicalhyperx/util.h"


namespace HierarchicalHyperX {

MinimalAdaptiveRoutingAlgorithm::MinimalAdaptiveRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter,
    f64 _threshold, u32 _localDeroute)
    : RoutingAlgorithm(_name, _parent, _router, _latency, _baseVc, _numVcs),
      globalDimWidths_(_globalDimensionWidths),
      globalDimWeights_(_globalDimensionWeights),
      localDimWidths_(_localDimensionWidths),
      localDimWeights_(_localDimensionWeights),
      concentration_(_concentration),
      globalLinksPerRouter_(_globalLinksPerRouter),
      threshold_(_threshold), localDeroute_(_localDeroute) {
  assert(numVcs_ >= (globalDimWidths_.size() + 1) * localDimWidths_.size()
         *(1 + localDeroute_) + globalDimWidths_.size());
}

MinimalAdaptiveRoutingAlgorithm::~MinimalAdaptiveRoutingAlgorithm() {}

void MinimalAdaptiveRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress =
    _flit->packet()->message()->getDestinationAddress();
  Packet* packet = _flit->packet();

  // perform routing
  std::unordered_set<u32> outputPorts = routing(_flit, *destinationAddress);
  assert(outputPorts.size() >= 1);

  if (*outputPorts.begin() >= getPortBase(concentration_, localDimWidths_,
                                          localDimWeights_)) {
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
    // reset deroute
    ri->localDerouteCount = localDeroute_;
    packet->setRoutingExtension(ri);
  }

  // increment VcSet for each hop
  u32 vcSet = _flit->packet()->getHopCount() - 1;

  // format the response
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    if (outputPort < concentration_) {
      for (u32 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
        _response->add(outputPort, vc);
      }
      RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
       packet->getRoutingExtension());
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
      delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
      delete reinterpret_cast<const std::vector<u32>*>(
        ri->intermediateAddress);
      delete ri;
      packet->setRoutingExtension(nullptr);
    } else {
      // select VCs in the corresponding set
      for (u32 vc = baseVc_ + vcSet; vc < baseVc_ + numVcs_;
           vc += (globalDimWidths_.size() + 1)
               * localDimWidths_.size() * (1 + localDeroute_)
               + globalDimWidths_.size()) {
        _response->add(outputPort, vc);
      }
    }
  }
  assert(_response->size() > 0);
}

std::unordered_set<u32> MinimalAdaptiveRoutingAlgorithm::routing
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
    ri->localDerouteCount = localDeroute_;
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
      setLocalDst(diffGlobalDims, _destinationAddress, &globalOutputPorts,
                  packet, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
    }

    const std::vector<u32>* localDst =
        reinterpret_cast<const std::vector<u32>*>(ri->localDst);

    // if at local dst
    if (std::equal(localDst->begin(), localDst->end(),
                   routerAddress.begin())) {
      ifAtLocalDst(_flit, &outputPorts, &globalOutputPorts, diffGlobalDims);
      assert(outputPorts.size() > 0);
    } else {
      // determine the local dimension to work on
      std::vector<u32> diffLocalDims;
      std::unordered_map<u32, f64> portAvailability;
      for (u32 localDim = 0; localDim < localDimensions; localDim++) {
        if (routerAddress.at(localDim) != localDst->at(localDim)) {
          diffLocalDims.push_back(localDim);
        }
      }
      findPortAvailability(diffLocalDims, &portAvailability, *localDst,
                           _flit);
      // find the port with max availability
      u32 highestPort = findMostAvailablePort(portAvailability);
      bool res = outputPorts.insert(highestPort).second;
      (void)res;
      assert(res);
    }
  } else {
    // if at the same global virtual router
    // find the port with most availability of all offset dimensions

    // determine all local dimensions src and dst are different
    std::vector<u32> diffDims;
    std::unordered_map<u32, f64> portAvailability;
    bool atDst = true;
    for (u32 localDim = 0; localDim < localDimensions; localDim++) {
      if (routerAddress.at(localDim) != _destinationAddress.at(localDim+1)) {
        diffDims.push_back(localDim);
        atDst = false;
      }
    }
    // test if already at destination local router
    if (atDst == true) {
      bool res = outputPorts.insert(_destinationAddress.at(0)).second;
      (void)res;
      assert(res);
    } else {
      // more local router-to-router hops needed
      std::vector<u32> dstRouter(_destinationAddress.begin() + 1,
                                 _destinationAddress.end());
      findPortAvailability(diffDims, &portAvailability, dstRouter, _flit);
      // find the port with max availability
      u32 availablePort = findMostAvailablePort(portAvailability);

      bool res = outputPorts.insert(availablePort).second;
      (void)res;
      assert(res);
    }
    assert(outputPorts.size() > 0);
  }
  return outputPorts;
}

// status: 0 = empty, 1 = fully congested
u32 MinimalAdaptiveRoutingAlgorithm::findMostAvailablePort(
    const std::unordered_map< u32, f64 >& _portAvailability) const {
  assert(_portAvailability.size() >= 1);
  f64 lowest = INT_MAX;
  for (auto const& port : _portAvailability) {
    if (port.second <= lowest) {
      lowest = port.second;
    }
  }
  // break tie with random selection
  std::vector<u32> potentialPorts;
  for (auto const& port : _portAvailability) {
    if (port.second == lowest) {
      potentialPorts.push_back(port.first);
    }
  }
  assert(potentialPorts.size() > 0);
  u32 rnd = gSim->rnd.nextU64(0, potentialPorts.size() - 1);
  return potentialPorts.at(rnd);
}

void MinimalAdaptiveRoutingAlgorithm::findPortAvailability(
    const std::vector<u32>& _diffDims,
    std::unordered_map<u32, f64>* _portAvailability,
    const std::vector<u32>& _destinationAddress, Flit* _flit) const {
  const std::vector<u32>& routerAddress = router_->address();
  for (auto itr = _diffDims.begin(); itr != _diffDims.end(); itr++) {
    u32 src = routerAddress.at(*itr);
    u32 dst = _destinationAddress.at(*itr);
    if (dst < src) {
      dst += localDimWidths_.at(*itr);
    }
    u32 portBase = concentration_;
    for (u32 localDim = 0; localDim < *itr; localDim++) {
      portBase += ((localDimWidths_.at(localDim) - 1)
                   * localDimWeights_.at(localDim));
    }
    u32 offset = (dst - src - 1) * localDimWeights_.at(*itr);
    // all ports where the two routers are connecting
    for (u32 weight = 0; weight < localDimWeights_.at(*itr); weight++) {
      u32 port = portBase + offset + weight;
      f64 availability = 0.0;
      for (u32 vc = _flit->packet()->getHopCount() - 1; vc < numVcs_;
           vc += (globalDimWidths_.size() + 1)
               * localDimWidths_.size() * (1 + localDeroute_)
               + globalDimWidths_.size()) {
        // u32 vcIdx = router_->vcIndex(port, vc);
        availability += router_->congestionStatus(port, vc);
      }
      _portAvailability->insert(std::make_pair(port, availability));
    }
  }
}

void MinimalAdaptiveRoutingAlgorithm::ifAtLocalDst
  (Flit* _flit, std::unordered_set<u32>* _outputPorts,
    std::vector<u32>* _globalOutputPorts,
    const std::vector<u32>& _diffGlobalDims) const {
  Packet* packet = _flit->packet();
  const std::vector<u32>& routerAddress = router_->address();
  const std::vector<u32>* destinationAddress =
      _flit->packet()->message()->getDestinationAddress();
  RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());

  const std::vector<u32>* localDst =
      reinterpret_cast<const std::vector<u32>*>(ri->localDst);
  const std::vector<u32>* localDstPort =
      reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);

  u32 portBase = getPortBase(concentration_, localDimWidths_,
                             localDimWeights_);
  // if at local dst
  if (std::equal(localDst->begin(), localDst->end(),
                 routerAddress.begin())) {
    assert(_outputPorts->size() == 0);
    // all appropriate ports
    for (auto itr = localDstPort->begin();
         itr != localDstPort->end(); itr++) {
      f64 availability = 0.0;
      u32 vcCount = 0;
      for (u32 vc = _flit->packet()->getHopCount(); vc < numVcs_;
           vc += (globalDimWidths_.size() + 1)
               * localDimWidths_.size() * (1 + localDeroute_)
               + globalDimWidths_.size()) {
        // u32 vcIdx = router_->vcIndex(portBase + *itr, vc);
        availability += router_->congestionStatus(portBase + *itr, vc);
        vcCount++;
      }
      availability = availability / vcCount;
      // port not congested or has to take it
      if (availability < threshold_ || ri->localDerouteCount <= 0) {
        bool res = _outputPorts->insert(portBase + *itr).second;
        (void)res;
        assert(res);
      }
    }
    // can deroute and ports all congested
    if (_outputPorts->size() == 0 && ri->localDerouteCount > 0) {
      _globalOutputPorts->clear();
      // delete previous local dst ptr
      if (ri->localDst != nullptr) {
        delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
      }
      ri->localDst = nullptr;
      if (ri->localDstPort != nullptr) {
        delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
      }
      ri->localDstPort = nullptr;
      setLocalDst(_diffGlobalDims, *destinationAddress, _globalOutputPorts,
                  packet, routerAddress, localDimWidths_, globalDimWidths_,
                  globalDimWeights_);
      ri->localDerouteCount--;
      packet->setRoutingExtension(ri);
      ifAtLocalDst(_flit, _outputPorts, _globalOutputPorts, _diffGlobalDims);
      // make sure outputports will be set
      if (_outputPorts->size() == 0) {
        const std::vector<u32>* localDst =
            reinterpret_cast<const std::vector<u32>*>(ri->localDst);
        u32 localDimensions = localDimWidths_.size();
        // determine the local dimension to work on
        std::vector<u32> diffLocalDims;
        std::unordered_map<u32, f64> portAvailability;
        for (u32 localDim = 0; localDim < localDimensions; localDim++) {
          if (routerAddress.at(localDim) != localDst->at(localDim)) {
            diffLocalDims.push_back(localDim);
          }
        }
        findPortAvailability(diffLocalDims, &portAvailability, *localDst,
                             _flit);
        // find the port with max availability
        u32 highestPort = findMostAvailablePort(portAvailability);
        bool res = _outputPorts->insert(highestPort).second;
        (void)res;
        assert(res);
      }
    }
  }
}

}  // namespace HierarchicalHyperX
