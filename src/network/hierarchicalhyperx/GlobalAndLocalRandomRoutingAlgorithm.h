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
#ifndef NETWORK_HIERARCHICALHYPERX_GLOBALANDLOCALRANDOMROUTINGALGORITHM_H_
#define NETWORK_HIERARCHICALHYPERX_GLOBALANDLOCALRANDOMROUTINGALGORITHM_H_

#include <prim/prim.h>

#include <string>
#include <vector>

#include "event/Component.h"
#include "network/RoutingAlgorithm.h"
#include "router/Router.h"

namespace HierarchicalHyperX {

// This routing algorihtm will resolve the different global and local
// dimensions between src and dst in a random fashion, as compared with
// the fixed order in DimOrderRouting

class GlobalAndLocalRandomRoutingAlgorithm : public RoutingAlgorithm {
 public:
  GlobalAndLocalRandomRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _globalDimensionWidths,
    const std::vector<u32>& _globalDimensionWeights,
    const std::vector<u32>& _localDimensionWidths,
    const std::vector<u32>& _localDimensionWeights,
    u32 _concentration, u32 _globalLinksPerRouter);
  ~GlobalAndLocalRandomRoutingAlgorithm();

 protected:
  void processRequest(
      Flit* _flit, RoutingAlgorithm::Response* _response) override;

  std::unordered_set<u32> routing(
      Flit* _flit, const std::vector<u32>& _destinationAddress) const;

  const std::vector<u32> globalDimWidths_;
  const std::vector<u32> globalDimWeights_;
  const std::vector<u32> localDimWidths_;
  const std::vector<u32> localDimWeights_;
  const u32 concentration_;
  const u32 globalLinksPerRouter_;
};

}  // namespace HierarchicalHyperX

#endif  // NETWORK_HIERARCHICALHYPERX_GLOBALANDLOCALRANDOMROUTINGALGORITHM_H_
