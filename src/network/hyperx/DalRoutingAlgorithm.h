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
#ifndef NETWORK_HYPERX_DALROUTINGALGORITHM_H_
#define NETWORK_HYPERX_DALROUTINGALGORITHM_H_

#include <colhash/tuplehash.h>
#include <json/json.h>
#include <prim/prim.h>

#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "event/Component.h"
#include "network/RoutingAlgorithm.h"
#include "network/hyperx/util.h"
#include "router/Router.h"

namespace HyperX {

class DalRoutingAlgorithm : public RoutingAlgorithm {
 public:
  DalRoutingAlgorithm(const std::string& _name, const Component* _parent,
                      Router* _router, u64 _latency, u32 _baseVc, u32 _numVcs,
                      const std::vector<u32>& _dimensionWidths,
                      const std::vector<u32>& _dimensionWeights,
                      u32 _concentration, u32 _inputPort,
                      Json::Value _settings);
  ~DalRoutingAlgorithm();
  void vcScheduled(Flit* _flit, u32 _port, u32 _vc);

 protected:
  void processRequest(Flit* _flit,
                      RoutingAlgorithm::Response* _response) override;

 private:
  const std::vector<u32> dimensionWidths_;
  const std::vector<u32> dimensionWeights_;
  const u32 concentration_;
  const u32 inputPort_;

  u32 numVcSets_;
  u32 maxOutputs_;
  OutputAlg outputAlg_;
  f64 threshold_;
  f64 iBias_;
  f64 cBias_;

  bool outputTypePort_;
  AdaptiveRoutingAlg adaptivityType_;
  DecisionScheme decisionScheme_;
  bool multiDeroute_;
  u32 maxHopsAllowed_;

  std::unordered_set<std::tuple<u32, u32, f64>> outputVcsMin_;
  std::unordered_set<std::tuple<u32, u32, f64>> outputVcsDer_;
  std::unordered_set<std::tuple<u32, u32, f64>> vcPool_;
  std::unordered_set<std::tuple<u32, u32, f64>> outputPorts_;
};

}  // namespace HyperX

#endif  // NETWORK_HYPERX_DALROUTINGALGORITHM_H_
