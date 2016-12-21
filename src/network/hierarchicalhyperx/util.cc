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

#include <strop/strop.h>

#include <cassert>

namespace HierarchicalHyperX {

void globalPortToLocalAddress
  (u32 _globalPort, std::vector<u32>* _localAddress,
    u32* _localPortWithoutBase, const std::vector<u32>& _localDimWidths) {
  u32 localDimensions = _localDimWidths.size();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 dim = 0; dim < localDimensions; dim++) {
    numRoutersPerGlobalRouter *= _localDimWidths.at(dim);
  }
  u32 numNodesInRemainingDims = 1;
  for (u32 dim = 0; dim < localDimensions - 1; dim++) {
    numNodesInRemainingDims *= _localDimWidths.at(dim);
  }
  u32 globalPortCopy = _globalPort;
  for (s32 localDim = localDimensions - 1; localDim >= 0; localDim--) {
    _localAddress->at(localDim) = (globalPortCopy / numNodesInRemainingDims)
        % _localDimWidths.at(localDim);
    globalPortCopy %= numNodesInRemainingDims;
    if (localDim != 0) {
      numNodesInRemainingDims /= _localDimWidths.at(localDim - 1);
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

// This function selects a random local destination when current node
// and final dst are in different groups
void setLocalDst(const std::vector<u32>& _diffGlobalDims,
                 const std::vector<u32>& _destinationAddress,
                 std::vector<u32>* _globalOutputPorts,
                 Packet* _packet,
                 const std::vector<u32>& _routerAddress,
                 const std::vector<u32>& _localDimWidths,
                 const std::vector<u32>& _globalDimWidths,
                 const std::vector<u32>& _globalDimWeights) {
  u32 localDimensions = _localDimWidths.size();
  RoutingInfo* ri = reinterpret_cast<RoutingInfo*>(
      _packet->getRoutingExtension());
  // pick a random global dimension
  u32 pos = gSim->rnd.nextU64(0, _diffGlobalDims.size() - 1);
  u32 globalDim = _diffGlobalDims.at(pos);
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
  std::vector<u32>* dstPort = new std::vector<u32>;

  // set local dst to self if has global link
  for (auto itr = _globalOutputPorts->begin();
       itr != _globalOutputPorts->end(); itr++) {
    std::vector<u32>* localRouter = new std::vector<u32>(localDimensions);
    u32 connectedPort;
    globalPortToLocalAddress(*itr, localRouter, &connectedPort,
                             _localDimWidths);
    if (std::equal(localRouter->begin(), localRouter->end(),
                   _routerAddress.begin())) {
      hasGlobalLinkToDst = true;
      ri->localDst = localRouter;
      dstPort->push_back(connectedPort);
      ri->localDstPort = dstPort;
      _packet->setRoutingExtension(ri);
    } else {
      delete localRouter;
    }
  }

  // if no global link, pick a random one
  if (hasGlobalLinkToDst == false) {
    // pick a random global port
    u32 globalPort = _globalOutputPorts->at(gSim->rnd.nextU64(
        0, _globalOutputPorts->size() - 1));

    // translate global router port number to local router
    std::vector<u32>* localRouter = new std::vector<u32>(localDimensions);
    u32 connectedPort;
    globalPortToLocalAddress(globalPort, localRouter, &connectedPort,
                             _localDimWidths);
    dstPort->push_back(connectedPort);
    ri->localDst = localRouter;
    ri->localDstPort = dstPort;
    _packet->setRoutingExtension(ri);
  }
}

u32 getHopDistance(const std::vector<u32>& _routerAddress,
                   const std::vector<u32>& _dstAddress,
                   const std::vector<u32>& _localDimWidths,
                   const std::vector<u32>& _globalDimWidths,
                   const std::vector<u32>& _globalDimWeights) {
  if (_routerAddress == _dstAddress) {
    return 0;
  }
  u32 localDims = _localDimWidths.size();
  u32 globalDims = _globalDimWidths.size();
  std::vector<u32> routerLocalAdd(_routerAddress.begin(),
                                  _routerAddress.begin() + localDims);
  std::vector<u32> routerGlobalAdd(_routerAddress.begin() + localDims,
                                   _routerAddress.end());
  std::vector<u32> dstLocalAdd(_dstAddress.begin(),
                               _dstAddress.begin() + localDims);
  std::vector<u32> dstGlobalAdd(_dstAddress.begin() + localDims,
                                _dstAddress.end());
  if (routerGlobalAdd == dstGlobalAdd) {
    u32 diffDims = 0;
    for (u32 i=0; i < localDims; i++) {
      if (routerLocalAdd.at(i) != dstLocalAdd.at(i)) {
        diffDims++;
      }
    }
    return diffDims;
  } else {
    std::vector<u32> nextGroup(routerGlobalAdd);
    printf("nextGroup is %s \n",
         strop::vecString<u32>(nextGroup).c_str());
    u32 diffGlobalDim = 0;
    for (u32 i = 0; i < globalDims; i++) {
      if (dstGlobalAdd.at(i) != routerGlobalAdd.at(i)) {
        nextGroup.at(i) = dstGlobalAdd.at(i);
        diffGlobalDim = i;
        break;
      }
    }
    printf("diffGlobalDim = %u \n", diffGlobalDim);
    u32 srcGroupPort = 0;
    for (u32 globalDim = 0; globalDim < diffGlobalDim; globalDim++) {
      srcGroupPort += (_globalDimWidths.at(globalDim) - 1) *
                      _globalDimWeights.at(globalDim);
    }
    for (u32 offset = 1; offset < abs(nextGroup.at(diffGlobalDim)
         - routerGlobalAdd.at(diffGlobalDim)); offset++) {
      srcGroupPort += _globalDimWeights.at(diffGlobalDim);
    }
    u32 rndWeight = gSim->rnd.nextU64(0,
                      _globalDimWeights.at(diffGlobalDim) - 1);
    for (u32 weight = 0; weight < rndWeight; weight++) {
      srcGroupPort++;
    }
    printf("srcGroupPort = %u \n", srcGroupPort);
    u32 srcGroupPortFinal = 0;
    u32 dstGroupPortFinal = 0;
    // find src and dst group port number
    u32 virtualGlobalPortBase = 0;
    for (u32 globalDim = 0; globalDim < globalDims; globalDim++) {
      u32 globalDimWidth = _globalDimWidths.at(globalDim);
      u32 globalDimWeight = _globalDimWeights.at(globalDim);
      for (u32 offset = 1; offset < globalDimWidth; offset++) {
        for (u32 weight = 0; weight < globalDimWeight; weight++) {
          // determine the vitual port of global router
          u32 virtualGlobalSrcPort = virtualGlobalPortBase
                + ((offset - 1) * globalDimWeight)
                + weight;
          u32 virtualGlobalDstPort = virtualGlobalPortBase
              + ((globalDimWidth - 1 - offset) *
                 globalDimWeight) + weight;
          if (virtualGlobalSrcPort == srcGroupPort) {
            srcGroupPortFinal = virtualGlobalSrcPort;
            dstGroupPortFinal = virtualGlobalDstPort;
          }
        }
      }
      virtualGlobalPortBase += ((globalDimWidth - 1) * globalDimWeight);
    }
    printf("dstGroupPort = %u \n", dstGroupPortFinal);
    std::vector<u32> srcGroupDst(routerLocalAdd);
    u32 srcPort;
    globalPortToLocalAddress(srcGroupPortFinal, &srcGroupDst, &srcPort,
                             _localDimWidths);
    srcGroupDst.insert(srcGroupDst.end(), routerGlobalAdd.begin(),
                       routerGlobalAdd.end());
    printf("srcGroupDst now is %s \n",
         strop::vecString<u32>(srcGroupDst).c_str());
    std::vector<u32> dstGroupDst(dstLocalAdd);
    u32 dstPort;
    globalPortToLocalAddress(dstGroupPortFinal, &dstGroupDst, &dstPort,
                             _localDimWidths);
    dstGroupDst.insert(dstGroupDst.end(), nextGroup.begin(),
                       nextGroup.end());
    printf("dstGroupDst now is %s \n",
         strop::vecString<u32>(dstGroupDst).c_str());
    u32 srcGroupHop = getHopDistance(_routerAddress, srcGroupDst,
                   _localDimWidths, _globalDimWidths, _globalDimWeights);
    u32 dstGroupHop = getHopDistance(dstGroupDst, _dstAddress,
                   _localDimWidths, _globalDimWidths, _globalDimWeights);

    return srcGroupHop + 1 + dstGroupHop;
  }
}
}  // namespace HierarchicalHyperX
