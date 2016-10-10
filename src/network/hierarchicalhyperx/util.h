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
#ifndef NETWORK_HIERARCHICALHYPERX_UTIL_H_
#define NETWORK_HIERARCHICALHYPERX_UTIL_H_

#include <prim/prim.h>
#include <vector>
#include "types/Message.h"
#include "types/Packet.h"
#include "types/Flit.h"
#include "event/Component.h"
#include "network/RoutingAlgorithm.h"
#include "router/Router.h"

namespace HierarchicalHyperX {

struct RoutingInfo {
  void* intermediateAddress;
  void* localDst;
  void* localDstPort;
  u32 localDerouteCount;
  u32 globalHopCount;
  bool intermediateDone;
  bool valiantMode;
};

void globalPortToLocalAddress(
    u32 globalPort, std::vector<u32>* localAddress, u32* localPortWithoutBase,
    const std::vector<u32> localDimWidths_);

u32 getPortBase(u32 concentration_, const std::vector<u32> localDimWidths_,
                const std::vector<u32> localDimWeights_);

void setLocalDst(std::vector<u32>* diffGlobalDims,
                 const std::vector<u32>* destinationAddress,
                 std::vector<u32>* globalOutputPorts, Flit* _flit,
                 const std::vector<u32>& routerAddress,
                 const std::vector<u32> localDimWidths_,
                 const std::vector<u32> globalDimWidths_,
                 const std::vector<u32> globalDimWeights_);
}  // namespace HierarchicalHyperX

#endif  // NETWORK_HIERARCHICALHYPERX_UTIL_H_
