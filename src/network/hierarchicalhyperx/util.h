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
  std::vector<u32>* intermediateAddress;
  std::vector<u32>* localDst;
  std::vector<u32>* localDstPort;
  u32 localDerouteCount;
  u32 globalHopCount;
  bool intermediateDone;
  bool valiantMode;
};

void globalPortToLocalAddress(
    u32 _globalPort, std::vector<u32>* _localAddress,
    u32* _localPortWithoutBase, const std::vector<u32>& _localDimWidths);

u32 getPortBase(u32 _concentration_, const std::vector<u32>& _localDimWidths,
                const std::vector<u32>& _localDimWeights);

void setLocalDst(const std::vector<u32>& _diffGlobalDims,
                 const std::vector<u32>& _destinationAddress,
                 std::vector<u32>* _globalOutputPorts,
                 Packet* _packet,
                 const std::vector<u32>& _routerAddress,
                 const std::vector<u32>& _localDimWidths,
                 const std::vector<u32>& _globalDimWidths,
                 const std::vector<u32>& _globalDimWeights);
}  // namespace HierarchicalHyperX

#endif  // NETWORK_HIERARCHICALHYPERX_UTIL_H_
