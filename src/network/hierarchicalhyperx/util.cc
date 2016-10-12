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
#include "network/hierarchicalhyperx/util.h"

#include <cassert>

namespace HierarchicalHyperX {

void globalPortToLocalAddress
  (u32 _globalPort,
    std::vector<u32>* _localAddress, u32* _localPortWithoutBase,
    const std::vector<u32>& _localDimWidths) {
  u32 localDimensions = _localDimWidths.size();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 dim = 0; dim < localDimensions; dim++) {
    numRoutersPerGlobalRouter *= _localDimWidths.at(dim);
  }
  u32 product = 1;
  for (u32 dim = 0; dim < localDimensions - 1; dim++) {
    product *= _localDimWidths.at(dim);
  }
  u32 globalPortCopy = _globalPort;
  for (s32 localDim = localDimensions - 1; localDim >= 0; localDim--) {
    _localAddress->at(localDim) = (globalPortCopy / product)
        % _localDimWidths.at(localDim);
    globalPortCopy %= product;
    if (localDim != 0) {
      product /= _localDimWidths.at(localDim - 1);
    }
  }
  assert(_localAddress->size() == localDimensions);
  *_localPortWithoutBase = _globalPort / numRoutersPerGlobalRouter;
  // assert(*localPortWithoutBase < globalLinksPerRouter_);
}

u32 getPortBase(u32 _concentration, const std::vector<u32>& _localDimWidths,
                const std::vector<u32>& _localDimWeights) {
  u32 localDimensions = _localDimWidths.size();
  u32 portBase = _concentration;
  for (u32 i = 0; i < localDimensions; i++) {
    portBase += ((_localDimWidths.at(i) - 1) * _localDimWeights.at(i));
  }
  return portBase;
}

void setLocalDst(const std::vector<u32>& _diffGlobalDims,
                 const std::vector<u32>& _destinationAddress,
                 std::vector<u32>* _globalOutputPorts, Flit* _flit,
                 const std::vector<u32>& _routerAddress,
                 const std::vector<u32>& _localDimWidths,
                 const std::vector<u32>& _globalDimWidths,
                 const std::vector<u32>& _globalDimWeights) {
  u32 localDimensions = _localDimWidths.size();
  Packet* packet = _flit->getPacket();
  RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());

  // pick a random global dimension
  u32 globalDim = _diffGlobalDims.at
    (gSim->rnd.nextU64(0, _diffGlobalDims.size() - 1));
  u32 globalPortBase = 0;
  for (u32 dim = 0; dim < globalDim; dim++) {
    globalPortBase += ((_globalDimWidths.at(dim) - 1)
                       * _globalDimWeights.at(dim));
  }
  // find the right port of the virtual global router
  u32 src = _routerAddress.at(localDimensions + globalDim);
  u32 dst = _destinationAddress.at(localDimensions + globalDim + 1);
  if (dst < src) {
    dst += _globalDimWidths.at(globalDim);
  }
  u32 offset = (dst - src - 1) * _globalDimWeights.at(globalDim);

  // add all ports where the two global routers are connecting
  for (u32 weight = 0; weight < _globalDimWeights.at(globalDim);
       weight++) {
    u32 globalPort = globalPortBase + offset + weight;
    _globalOutputPorts->push_back(globalPort);
  }
  assert(_globalOutputPorts->size() > 0);

  bool hasGlobalLinkToDst = false;
  std::vector<u32> dstPort;

  // set local dst to self if has global link
  for (auto itr = _globalOutputPorts->begin();
       itr != _globalOutputPorts->end(); itr++) {
    std::vector<u32> localRouter(localDimensions);
    u32 connectedPort;
    globalPortToLocalAddress(*itr, &localRouter, &connectedPort,
                             _localDimWidths);
    if (std::equal(localRouter.begin(), localRouter.end(),
                   _routerAddress.begin())) {
      hasGlobalLinkToDst = true;
      ri->localDst = &localRouter;
      dstPort.push_back(connectedPort);
      ri->localDstPort = &dstPort;
      packet->setRoutingExtension(ri);
    }
  }
  // if no global link, pick a random one
  if (hasGlobalLinkToDst == false) {
    // pick a random global port
    u32 globalPort = _globalOutputPorts->at(gSim->rnd.nextU64(
        0, _globalOutputPorts->size() - 1));

    // translate global router port number to local router
    std::vector<u32> localRouter(localDimensions);
    u32 connectedPort;
    globalPortToLocalAddress(globalPort, &localRouter, &connectedPort,
                             _localDimWidths);
    dstPort.push_back(connectedPort);
    ri->localDst = &localRouter;
    ri->localDstPort = &dstPort;
    packet->setRoutingExtension(ri);
  }
}

}  // namespace HierarchicalHyperX
