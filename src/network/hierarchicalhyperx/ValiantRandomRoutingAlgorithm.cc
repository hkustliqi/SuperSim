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
#include "network/hierarchicalhyperx/ValiantRandomRoutingAlgorithm.h"

#include <strop/strop.h>

#include <cassert>
#include <set>
#include <unordered_set>

#include "types/Packet.h"
#include "types/Message.h"
#include "network/hierarchicalhyperx/util.h"

namespace HierarchicalHyperX {

ValiantRandomRoutingAlgorithm::ValiantRandomRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter, bool _randomGroup)
  : GlobalAndLocalRandomRoutingAlgorithm
    (_name, _parent, _router, _latency, _baseVc, _numVcs,
     _globalDimensionWidths, _globalDimensionWeights, _localDimensionWidths,
     _localDimensionWeights, _concentration, _globalLinksPerRouter),
     randomGroup_(_randomGroup) {
  assert(numVcs_ >= 2 * globalDimWidths_.size() + 2);
}

ValiantRandomRoutingAlgorithm::~ValiantRandomRoutingAlgorithm() {}

void ValiantRandomRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  // ex: [1,...,m,1,...,n]
  const std::vector<u32>& routerAddress = router_->address();
  Packet* packet = _flit->packet();
  Message* message = packet->message();
  // ex: [c,1,...,m,1,...,n]
  const std::vector<u32>* destinationAddress = message->getDestinationAddress();

  // if at destination
  if (std::equal(routerAddress.begin(), routerAddress.end(),
                 destinationAddress->begin() + 1)) {
    u32 outputPort = destinationAddress->at(0);
    // on ejection, any dateline VcSet is ok within any stage VcSet
    for (u32 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
      _response->add(outputPort, vc);
    }
    assert(_response->size() > 0);
    // delete the routing extension
    RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
        packet->getRoutingExtension());
    /* if (ri->localDst != nullptr) {
    delete reinterpret_cast<const std::vector<u32>*>(ri->localDst);
    }
    if (ri->localDstPort != nullptr) {
    delete reinterpret_cast<const std::vector<u32>*>(ri->localDstPort);
    } 
    delete reinterpret_cast<const std::vector<u32>*>(ri->intermediateAddress);*/
    delete ri;
    packet->setRoutingExtension(nullptr);
  } else {
    std::unordered_set<u32> outputPorts = routing(_flit, *destinationAddress,
                                                  randomGroup_);
    assert(outputPorts.size() > 0);
    RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
        packet->getRoutingExtension());
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
    u32 vcSet = ri->globalHopCount;
    assert(vcSet <= 2 * globalDimWidths_.size() + 2);

    // format the response
    for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
      u32 outputPort = *it;
      // select VCs in the corresponding set
      for (u32 vc = baseVc_ + vcSet; vc < baseVc_ + numVcs_;
           vc += 2 * globalDimWidths_.size() + 2) {
        _response->add(outputPort, vc);
      }
    }
    assert(_response->size() > 0);
  }
}

std::unordered_set<u32> ValiantRandomRoutingAlgorithm::routing
  (Flit* _flit, const std::vector<u32>& _destinationAddress,
    bool _randomGroup) const {
  const std::vector<u32>& routerAddress = router_->address();
  Packet* packet = _flit->packet();

  // create the routing extension if needed
  if (packet->getRoutingExtension() == nullptr) {
    // create routing extension header
    //  the extension is a vector with one dummy element then the address of the
    //  intermediate router
    std::vector<u32>* re = new std::vector<u32>(1 + routerAddress.size());
    re->at(0) = U32_MAX;  // dummy

    // random intermediate address
    for (u32 idx = 0; idx < localDimWidths_.size(); idx++) {
      re->at(idx + 1) = gSim->rnd.nextU64(0, localDimWidths_.at(idx) - 1);
    }
    for (u32 idx = 0; idx < globalDimWidths_.size(); idx++) {
      re->at(idx + localDimWidths_.size() + 1) =
          gSim->rnd.nextU64(0, globalDimWidths_.at(idx) - 1);
    }
    RoutingInfo* ri = new RoutingInfo();
    ri->intermediateAddress = re;
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
  if (ri->intermediateAddress == nullptr) {
    std::vector<u32>* re = new std::vector<u32>(1 + routerAddress.size());
    re->at(0) = U32_MAX;  // dummy

    // random intermediate address
    for (u32 idx = 0; idx < localDimWidths_.size(); idx++) {
      re->at(idx + 1) = gSim->rnd.nextU64(0, localDimWidths_.at(idx) - 1);
    }
    for (u32 idx = 0; idx < globalDimWidths_.size(); idx++) {
      re->at(idx + localDimWidths_.size() + 1) =
          gSim->rnd.nextU64(0, globalDimWidths_.at(idx) - 1);
    }
    ri->intermediateAddress = re;
  }
  // intermediate address info
  const std::vector<u32>* intermediateAddress =
      reinterpret_cast<const std::vector<u32>*>(ri->intermediateAddress);
  assert(routerAddress.size() == _destinationAddress.size() - 1);
  assert(routerAddress.size() == intermediateAddress->size() - 1);

  // update intermediate info for Valiant
  if (ri->intermediateDone == false) {
    if (_randomGroup == true) {
      const std::vector<u32> intermediateGroup
        (intermediateAddress->begin() + localDimWidths_.size() + 1,
         intermediateAddress->end());
      if (std::equal(routerAddress.begin() + localDimWidths_.size(),
                     routerAddress.end(), intermediateGroup.begin())) {
        ri->intermediateDone = true;
        packet->setRoutingExtension(ri);
      }
    } else {
      const std::vector<u32> intermediateNode(intermediateAddress->begin() +
                                              + 1, intermediateAddress->end());
      if (std::equal(routerAddress.begin(),
                     routerAddress.end(), intermediateNode.begin())) {
        ri->intermediateDone = true;
        packet->setRoutingExtension(ri);
      }
    }
  }

  std::unordered_set<u32> outputPorts;
  // first stage of valiant
  if (ri->intermediateDone == false) {
    outputPorts = GlobalAndLocalRandomRoutingAlgorithm::routing(
        _flit, *intermediateAddress);
  } else {
    outputPorts = GlobalAndLocalRandomRoutingAlgorithm::routing(
        _flit, _destinationAddress);
  }
  return outputPorts;
}

}  // namespace HierarchicalHyperX
