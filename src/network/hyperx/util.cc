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
#include "network/hyperx/util.h"

#include <colhash/tuplehash.h>

#include <cassert>
#include <set>

#include <iostream>

#include "network/cube/util.h"

namespace HyperX {

/**************************UTILITY FUNCTIONS**********************************/

bool isDestinationRouter(
    Router* _router, const std::vector<u32>* _destinationAddress) {
  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      break;
    }
  }
  if (dim == routerAddress.size()) {
    return true;
  }
  return false;
}

u32 hopsLeft(Router* _router, const std::vector<u32>* _destinationAddress) {
  u32 hops = 0;
  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      hops++;
    }
  }
  return hops;
}

u32 computeInputPortDim(const std::vector<u32>& _dimensionWidths,
                        const std::vector<u32>& _dimensionWeights,
                        u32 _concentration, u32 _inputPort) {
  // determine which network dimension this port is attached to
  if (_inputPort < _concentration) {
    return U32_MAX;  // terminal dimension
  }

  u32 port = _inputPort - _concentration;
  for (u32 dim = 0; dim < _dimensionWeights.size(); dim++) {
    if (port < (_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim)) {
      return dim;
    } else {
      port -= (_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim);
    }
  }

  assert(false);
}

/*******************INTERMEDIATE DESTINATION FOR VALIANTS**********************/

void intNodeReg(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  u32 numIds = _concentration;
  u64 intId = U64_MAX;
  for (u32 dim = 0; dim < _dimensionWidths.size(); dim++) {
    numIds *= _dimensionWidths.at(dim);
  }
  intId = gSim->rnd.nextU64(0, numIds - 1);
  Cube::computeTerminalAddress(intId, _dimensionWidths, _concentration,
                               _address);
  _address->at(0) = 0;
}

void intNodeUnAligned(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  std::unordered_set<u32> nodesUnAligned;

  u32 numIds = _concentration;
  for (u32 dim = 0; dim < _dimensionWidths.size(); dim++) {
    numIds *= _dimensionWidths.at(dim);
  }

  for (u32 nodeId = 0; nodeId < numIds; ++nodeId) {
    std::vector<u32> nodeAddr;
    Cube::computeRouterAddress(nodeId, _dimensionWidths, &nodeAddr);
    bool aligned = true;
    for (u32 dim = 0; dim < _dimensionWidths.size(); dim++) {
      if ((_sourceRouter.at(dim) == _destinationTerminal->at(dim + 1)) &&
          (_sourceRouter.at(dim) != nodeAddr.at(dim))) {
        aligned = false;
      }
    }
    if (aligned) {
      bool res = nodesUnAligned.emplace(nodeId).second;
      (void)res;
      assert(res);
    }
  }

  const u32* it = uSetRandElement(nodesUnAligned);
  std::vector<u32> ancestor;
  Cube::computeRouterAddress(*it, _dimensionWidths, &ancestor);

  for (u32 ind = 0; ind < _sourceRouter.size(); ind++) {
    _address->at(ind + 1) = ancestor.at(ind);
  }
  _address->at(0) = 0;
}

void intNodeSrc(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  std::unordered_set<u32> ancestors;
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  for (u32 dim = 0; dim < dimensions; dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim); idx++) {
      std::vector<u32> tmpVec(_sourceRouter);
      tmpVec.at(dim) = idx;
      u32 ancestorId = Cube::computeRouterId(&tmpVec, _dimensionWidths);
      ancestors.emplace(ancestorId);
    }
  }

  const u32* it = uSetRandElement(ancestors);
  std::vector<u32> ancestor;
  Cube::computeRouterAddress(*it, _dimensionWidths, &ancestor);

  for (u32 ind = 0; ind < _sourceRouter.size(); ind++) {
    _address->at(ind + 1) = ancestor.at(ind);
  }
  _address->at(0) = 0;
}

void intNodeDst(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  std::unordered_set<u32> ancestors;
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  for (u32 dim = 0; dim < dimensions; dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim); idx++) {
      std::vector<u32> tmpVec(dimensions);
      for (u32 ind = 0; ind < dimensions; ind++) {
        tmpVec.at(ind) = _destinationTerminal->at(ind + 1);
      }
      tmpVec.at(dim) = idx;
      u32 ancestorId = Cube::computeRouterId(&tmpVec, _dimensionWidths);
      ancestors.emplace(ancestorId);
    }
  }

  const u32* it = uSetRandElement(ancestors);
  std::vector<u32> ancestor;
  Cube::computeRouterAddress(*it, _dimensionWidths, &ancestor);

  for (u32 ind = 0; ind < _sourceRouter.size(); ind++) {
    _address->at(ind + 1) = ancestor.at(ind);
  }
  _address->at(0) = 0;
}

void intNodeSrcDst(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  std::unordered_set<u32> ancestors;
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  for (u32 dim = 0; dim < dimensions; dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim); idx++) {
      std::vector<u32> tmpVec(_sourceRouter);
      tmpVec.at(dim) = idx;
      u32 ancestorId = Cube::computeRouterId(&tmpVec, _dimensionWidths);
      ancestors.emplace(ancestorId);
    }
  }

  for (u32 dim = 0; dim < dimensions; dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim); idx++) {
      std::vector<u32> tmpVec(dimensions);
      for (u32 ind = 0; ind < dimensions; ind++) {
        tmpVec.at(ind) = _destinationTerminal->at(ind + 1);
      }
      tmpVec.at(dim) = idx;
      u32 ancestorId = Cube::computeRouterId(&tmpVec, _dimensionWidths);
      ancestors.emplace(ancestorId);
    }
  }

  const u32* it = uSetRandElement(ancestors);
  std::vector<u32> ancestor;
  Cube::computeRouterAddress(*it, _dimensionWidths, &ancestor);

  for (u32 ind = 0; ind < _sourceRouter.size(); ind++) {
    _address->at(ind + 1) = ancestor.at(ind);
  }
  _address->at(0) = 0;
}

void intNodeMinV(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  u32 dim;
  u32 portBase = _concentration;
  f64 minCongestion = F64_POS_INF;

  std::unordered_set<u32> ancestors;
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  // determine available dimensions
  for (dim = 0; dim < dimensions; dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim) - 1; idx++) {
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + idx + weight;
        for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
          f64 congestion = _router->congestionStatus(port, vc);
          if (congestion > minCongestion) {
            continue;
          } else if (congestion < minCongestion) {
            minCongestion = congestion;
            ancestors.clear();
          }
          std::vector<u32> tmpVec(_sourceRouter);
          if (idx < _sourceRouter.at(dim)) {
            tmpVec.at(dim) = idx;
          } else {
            tmpVec.at(dim) = idx + 1;
          }
          u32 ancestorId = Cube::computeRouterId(&tmpVec, _dimensionWidths);
          // we don't need to perform check here, because congestion is
          // for port,vc pair, and we're interested only in Router ID (port),
          // so we can have many port,vc pair with the same congestion
          // for the same port
          ancestors.emplace(ancestorId);
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  const u32* it = uSetRandElement(ancestors);
  std::vector<u32> ancestor;
  Cube::computeRouterAddress(*it, _dimensionWidths, &ancestor);

  for (u32 ind = 0; ind < _sourceRouter.size(); ind++) {
    _address->at(ind + 1) = ancestor.at(ind);
  }
  _address->at(0) = 0;
}

void intNodeMinP(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address) {
  u32 dim;
  u32 portBase = _concentration;
  f64 minCongestion = F64_POS_INF;

  std::unordered_set<u32> ancestors;
  u32 dimensions = _dimensionWidths.size();
  _address->resize(1 + dimensions);

  // determine available dimensions
  for (dim = 0; dim < dimensions; dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim) - 1; idx++) {
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + idx + weight;
        f64 congestion = getAveragePortCongestion(_router, port,
                                                  _vcSet, _numVcSets, _numVcs);
        if (congestion > minCongestion) {
          continue;
        } else if (congestion < minCongestion) {
          minCongestion = congestion;
          ancestors.clear();
        }
        for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
          std::vector<u32> tmpVec(_sourceRouter);
          if (idx < _sourceRouter.at(dim)) {
            tmpVec.at(dim) = idx;
          } else {
            tmpVec.at(dim) = idx + 1;
          }
          u32 ancestorId = Cube::computeRouterId(&tmpVec, _dimensionWidths);
          // we don't need to perform check here, because congestion is
          // for port,vc pair, and we're interested only in Router ID (port),
          // so we can have many port,vc pair with the same congestion
          // for the same port
          ancestors.emplace(ancestorId);
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  const u32* it = uSetRandElement(ancestors);
  std::vector<u32> ancestor;
  Cube::computeRouterAddress(*it, _dimensionWidths, &ancestor);

  for (u32 ind = 0; ind < _sourceRouter.size(); ind++) {
    _address->at(ind + 1) = ancestor.at(ind);
  }
  _address->at(0) = 0;
}

/*******************MAX_OUTPUTS HANDLING FOR ROUTING ALGORITHMS***************/
void makeOutputVcSet(
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool,
    u32 _maxOutputs, OutputAlg _outputAlg,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputPorts) {
  _outputPorts->clear();
  if (_vcPool->size() == 0) {
    // No available ports found
    return;
  }

  if (!_maxOutputs) {
    for (auto& it : *_vcPool) {
      bool res = _outputPorts->emplace(it).second;
      (void)res;
      assert(res);
    }
  } else {
    for (u32 i = 0; i < _maxOutputs; ++i) {
      if (_vcPool->size() == 0) {
        return;
      } else {
        const std::tuple<u32, u32, f64>* it;
        if (_outputAlg == OutputAlg::Rand) {
          it = uSetRandElement(*_vcPool);
        } else if (_outputAlg == OutputAlg::Min) {
          it = uSetMinCong(*_vcPool);
        } else {
          fprintf(stderr, "Unknown output algorithm\n");
          assert(false);
        }
        bool res = _outputPorts->emplace(*it).second;
        (void)res;
        assert(res);
        _vcPool->erase(*it);
      }
    }
  }
}

void makeOutputPortSet(
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    u32 _maxOutputs, OutputAlg _outputAlg,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputPorts) {
  _outputPorts->clear();
  if (_vcPool->size() == 0) {
    // No available ports found
    return;
  }

  if (!_maxOutputs) {
    for (auto& it : *_vcPool) {
      u32 port = std::get<0>(it);
      f64 congestion = std::get<2>(it);
      for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
        std::tuple<u32, u32, f64> t(port, vc, congestion);
        bool res = _outputPorts->emplace(t).second;
        (void)res;
        assert(res);
      }
    }
  } else {
    for (u32 i = 0; i < _maxOutputs; ++i) {
      if (_vcPool->size() == 0) {
        return;
      } else {
        const std::tuple<u32, u32, f64>* it;
        if (_outputAlg == OutputAlg::Rand) {
          it = uSetRandElement(*_vcPool);
        } else if (_outputAlg == OutputAlg::Min) {
          it = uSetMinCong(*_vcPool);
        } else {
          fprintf(stderr, "Unknown output algorithm\n");
          assert(false);
        }
        u32 port = std::get<0>(*it);
        f64 congestion = std::get<2>(*it);
        for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
          std::tuple<u32, u32, f64> t(port, vc, congestion);
          bool res = _outputPorts->emplace(t).second;
          (void)res;
          assert(res);
        }
        _vcPool->erase(*it);
      }
    }
  }
}

f64 getAveragePortCongestion(Router* _router, u32 _port,
                             u32 _vcSet, u32 _numVcSets, u32 _numVcs) {
  u32 _numVcsInSet = 0;
  f64 congestion = 0;
  for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
    congestion += _router->congestionStatus(_port, vc);
    _numVcsInSet += 1;
  }
  congestion /= _numVcsInSet;  // average
  return congestion;
}

/*******************DIMENSION ORDERED ROUTING ALGORITHM***********************/

void dimOrderVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      break;
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dim != routerAddress.size()) {
    // more router-to-router hops needed
    u32 src = routerAddress.at(dim);
    u32 dst = _destinationAddress->at(dim+1);
    if (dst < src) {
      dst += _dimensionWidths.at(dim);
    }
    u32 offset = (dst - src - 1) * _dimensionWeights.at(dim);

    // add all ports where the two routers are connecting
    for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
      u32 port = portBase + offset + weight;
      for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
        // The code below returns congestion status
        // This information is not needed for DOR routing, which is completely
        // oblivious, it might be helpful for another routing algorithms that
        // use DOR as base routing in adaptive phases
        f64 congestion = _router->congestionStatus(port, vc);

        std::tuple<u32, u32, f64> t(port, vc, congestion);
        bool res = _vcPool->emplace(t).second;
        (void)res;
        assert(res);
      }
    }
  }
}

void dimOrderPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      break;
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dim != routerAddress.size()) {
    // more router-to-router hops needed
    u32 src = routerAddress.at(dim);
    u32 dst = _destinationAddress->at(dim+1);
    if (dst < src) {
      dst += _dimensionWidths.at(dim);
    }
    u32 offset = (dst - src - 1) * _dimensionWeights.at(dim);

    // add all ports where the two routers are connecting
    for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
      u32 port = portBase + offset + weight;
      // The code below returns congestion status
      // This information is not needed for DOR routing, which is completely
      // oblivious, it might be helpful for another routing algorithms that
      // use DOR as base routing in adaptive phases
      f64 congestion = getAveragePortCongestion(_router, port,
                                                _vcSet, _numVcSets, _numVcs);

      std::tuple<u32, u32, f64> t(port, 0, congestion);
      bool res = _vcPool->emplace(t).second;
      (void)res;
      assert(res);
    }
  }
}

/*******************RANDOM MINIMAL ROUTING ALGORITHMS ************************/

void randMinVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offset = (dst - src - 1) * _dimensionWeights.at(dim);

      // add all ports where the two routers are connecting
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + offset + weight;
        for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
          f64 congestion = _router->congestionStatus(port, vc);
          std::tuple<u32, u32, f64> t(port, vc, congestion);
          bool res = _vcPool->emplace(t).second;
          (void)res;
          assert(res);
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

void randMinPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;

  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offset = (dst - src - 1) * _dimensionWeights.at(dim);

      // add all ports where the two routers are connecting
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + offset + weight;
        f64 congestion = getAveragePortCongestion(_router, port,
                                                  _vcSet, _numVcSets, _numVcs);
        std::tuple<u32, u32, f64> t(port, 0, congestion);
        bool res = _vcPool->emplace(t).second;
        (void)res;
        assert(res);
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

/*******************ADAPTIVE MINIMAL ROUTING ALGORITHMS **********************/

void adaptiveMinVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  f64 minCongestion = F64_POS_INF;

  // determine available dimensions
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offset = (dst - src - 1) * _dimensionWeights.at(dim);

      // add all ports where the two routers are connecting
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + offset + weight;
        for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
          f64 congestion = _router->congestionStatus(port, vc);
          if (congestion > minCongestion) {
            continue;
          } else if (congestion < minCongestion) {
            minCongestion = congestion;
            _vcPool->clear();
          }
          std::tuple<u32, u32, f64> t(port, vc, congestion);
          bool res = _vcPool->emplace(t).second;
          (void)res;
          assert(res);
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

void adaptiveMinPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  f64 minCongestion = F64_POS_INF;

  // determine available dimensions
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offset = (dst - src - 1) * _dimensionWeights.at(dim);

      // add all ports where the two routers are connecting
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + offset + weight;
        f64 congestion = getAveragePortCongestion(_router, port,
                                                  _vcSet, _numVcSets, _numVcs);
        if (congestion > minCongestion) {
          continue;
        } else if (congestion < minCongestion) {
          minCongestion = congestion;
          _vcPool->clear();
        }
        std::tuple<u32, u32, f64> t(port, 0, congestion);
        bool res = _vcPool->emplace(t).second;
        (void)res;
        assert(res);
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

/*******************VALIANTS ROUTING ALGORITHMS *******************************/

void valiantsRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    IntNodeAlg _intNodeAlg, BaseRoutingAlg _routingAlg, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  Packet* packet = _flit->packet();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  // determine which stage we are in based on routing extension
  //  if routing extension is empty, it's stage 1
  u32 stage;
  if (packet->getHopCount() == 1) {
    stage = 0;
  } else {
    stage = (packet->getRoutingExtension() != nullptr) ? 0 : 1;
  }

  if (_shortCut) {
    // If source == destination, don't pick intermediate address
    if (isDestinationRouter(_router, _destinationAddress)) {
      delete reinterpret_cast<const std::vector<u32>*>
          (packet->getRoutingExtension());
      packet->setRoutingExtension(nullptr);
      return;
    }
  }

  // create the routing extension if needed
  if (packet->getHopCount() == 1) {
    // should be first router encountered
    assert(packet->getRoutingExtension() == nullptr);

    // create routing extension header
    //  the extension is a vector with one dummy element then the address of the
    //  intermediate router
    std::vector<u32>* intAddr = new std::vector<u32>(1 + routerAddress.size());

    IntNodeAlgFunc intNodeAlgFunc;
    switch (_intNodeAlg) {
      case IntNodeAlg::REG:
        intNodeAlgFunc = &intNodeReg;
        break;
      case IntNodeAlg::SRC:
        intNodeAlgFunc = &intNodeSrc;
        break;
      case IntNodeAlg::DST:
        intNodeAlgFunc = &intNodeDst;
        break;
      case IntNodeAlg::SRCDST:
        intNodeAlgFunc = &intNodeSrcDst;
        break;
      case IntNodeAlg::UNALIGNED:
        intNodeAlgFunc = &intNodeUnAligned;
        break;
      case IntNodeAlg::MINV:
        intNodeAlgFunc = &intNodeMinV;
        break;
      case IntNodeAlg::MINP:
        intNodeAlgFunc = &intNodeMinP;
        break;
      default:
        fprintf(stderr, "Unknown intermediate node algorithm\n");
        assert(false);
    }
    intNodeAlgFunc(_router, routerAddress, _destinationAddress,
                   _dimensionWidths, _dimensionWeights, _concentration,
                   _vcSet, _numVcSets, _numVcs, intAddr);

    intAddr->at(0) = U32_MAX;  // dummy
    packet->setRoutingExtension(intAddr);
  }

  // get a const pointer to the address (with leading dummy)
  const std::vector<u32>* intermediateAddress =
      reinterpret_cast<const std::vector<u32>*>(packet->getRoutingExtension());

  MinRoutingAlgFunc routingAlgFunc;
  switch (_routingAlg) {
    case BaseRoutingAlg::DORV   : {
      routingAlgFunc = &dimOrderVcRoutingOutput;
      break;
    }
    case BaseRoutingAlg::DORP   : {
      routingAlgFunc = &dimOrderPortRoutingOutput;
      break;
    }
    case BaseRoutingAlg::RMINV : {
      routingAlgFunc = &randMinVcRoutingOutput;
      break;
    }
    case BaseRoutingAlg::RMINP : {
      routingAlgFunc = &randMinPortRoutingOutput;
      break;
    }
    case BaseRoutingAlg::AMINV : {
      routingAlgFunc = &adaptiveMinVcRoutingOutput;
      break;
    }
    case BaseRoutingAlg::AMINP : {
      routingAlgFunc = &adaptiveMinPortRoutingOutput;
      break;
    }
    default                    : {
      fprintf(stderr, "Unknown routing algorithm\n");
      assert(false);
    }
  }

  if (stage == 0) {
    assert(packet->getRoutingExtension() != nullptr);

    routingAlgFunc(_router, _dimensionWidths, _dimensionWeights,
                   _concentration, intermediateAddress,
                   _vcSet, _numVcSets, _numVcs, _vcPool);
    if (_vcPool->empty()) {
      delete reinterpret_cast<const std::vector<u32>*>
          (packet->getRoutingExtension());
      packet->setRoutingExtension(nullptr);
      stage = 1;
      u32 vcSet = _vcSet;
      if ((_routingAlg == BaseRoutingAlg::DORP) ||
          (_routingAlg == BaseRoutingAlg::DORV)) {
        vcSet++;
      }
      routingAlgFunc(_router, _dimensionWidths, _dimensionWeights,
                     _concentration, _destinationAddress,
                     vcSet, _numVcSets, _numVcs, _vcPool);
      return;
    }
    assert(!_vcPool->empty());
  }

  if (stage == 1) {
    routingAlgFunc(_router, _dimensionWidths, _dimensionWeights,
                   _concentration, _destinationAddress,
                   _vcSet, _numVcSets, _numVcs, _vcPool);
  }
}

void ugalRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    bool _shortCut, IntNodeAlg _intNodeAlg,
    BaseRoutingAlg _routingAlg, NonMinRoutingAlg _nonMinimalAlg,
    f64 _bias, Flit* _flit, f64* _weightReg, f64* _weightVal,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPoolReg,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPoolVal) {
  _vcPoolReg->clear();
  _vcPoolVal->clear();

  const std::vector<u32>* destAddress =
      _flit->packet()->message()->getDestinationAddress();

  Packet* packet = _flit->packet();

  MinRoutingAlgFunc routingAlgFunc;
  switch (_routingAlg) {
    case BaseRoutingAlg::DORV   : {
      routingAlgFunc = &dimOrderVcRoutingOutput;
      break;
    }
    case BaseRoutingAlg::DORP   : {
      routingAlgFunc = &dimOrderPortRoutingOutput;
      break;
    }
    case BaseRoutingAlg::RMINV : {
      routingAlgFunc = &randMinVcRoutingOutput;
      break;
    }
    case BaseRoutingAlg::RMINP : {
      routingAlgFunc = &randMinPortRoutingOutput;
      break;
    }
    case BaseRoutingAlg::AMINV : {
      routingAlgFunc = &adaptiveMinVcRoutingOutput;
      break;
    }
    case BaseRoutingAlg::AMINP : {
      routingAlgFunc = &adaptiveMinPortRoutingOutput;
      break;
    }
    default                    : {
      fprintf(stderr, "Unknown routing algorithm\n");
      assert(false);
    }
  }

  if ((packet->getHopCount() == 1) &&
      (_nonMinimalAlg == NonMinRoutingAlg::LCQP)) {
    lcqPortRoutingOutput(
        _router, _dimensionWidths, _dimensionWeights, _concentration,
        destAddress, _vcSet, _numVcSets, _numVcs, _shortCut, _vcPoolVal);
  } else if ((packet->getHopCount() == 1) &&
             (_nonMinimalAlg == NonMinRoutingAlg::LCQV)) {
    lcqVcRoutingOutput(
        _router, _dimensionWidths, _dimensionWeights, _concentration,
        destAddress, _vcSet, _numVcSets, _numVcs, _shortCut, _vcPoolVal);
  } else {
    valiantsRoutingOutput(
        _router, _dimensionWidths, _dimensionWeights, _concentration,
        destAddress, _vcSet, _numVcSets, _numVcs, _shortCut, _intNodeAlg,
        _routingAlg, _flit, _vcPoolVal);
  }

  if (packet->getHopCount() == 1) {
    u32 hopsReg = 0;
    u32 hopsVal = 0;

    routingAlgFunc(_router, _dimensionWidths,
                   _dimensionWeights, _concentration, destAddress,
                   _vcSet, _numVcSets, _numVcs, _vcPoolReg);

    // ex: [x,y,z] for router, [c,x,y,z] for destination
    const std::vector<u32>& routerAddress = _router->address();

    // get a const pointer to the address (with leading dummy)
    const std::vector<u32>* intermediateAddress =
        reinterpret_cast<const std::vector<u32>*>(
            packet->getRoutingExtension());

    if (_vcPoolReg->empty()) {
      // Source and destination are the same, so UGAL should eject packet
      return;
    }

    if (intermediateAddress == nullptr) {
      // Intermediate address in VAL was equal to SRC
      // Routing converges to base_algorithm
      hopsVal = U32_MAX;
    } else {
      assert(routerAddress.size() == (destAddress->size() - 1));
      for (u32 dim = 0; dim < routerAddress.size(); dim++) {
        if (routerAddress.at(dim) != destAddress->at(dim + 1)) {
          hopsReg++;
        }
        assert(intermediateAddress);
        if (intermediateAddress->at(dim + 1) != destAddress->at(dim + 1)) {
          hopsVal++;
        }
        if (routerAddress.at(dim) != intermediateAddress->at(dim + 1)) {
          hopsVal++;
        }
      }
    }

    if ((_nonMinimalAlg == NonMinRoutingAlg::LCQV) ||
        (_nonMinimalAlg == NonMinRoutingAlg::LCQP)) {
      // For LCQ we do not have intermediateAddress (it is always null)
      // but we take extra one hop
      hopsVal = hopsReg + 1;
    }

    if (_vcPoolVal->empty()) {
      // assert(_vcPoolReg->empty());
      return;
    }

    if ( (packet->getHopCount() > 1) && (_routingAlg != BaseRoutingAlg::DORP) &&
         (_routingAlg != BaseRoutingAlg::DORV)) {
      assert(_vcSet > 0);
    }

    for (auto& it : *_vcPoolReg) {
      f64 congestion = std::get<2>(it);
      *_weightReg += hopsReg * congestion;
    }
    *_weightReg /= _vcPoolReg->size();

    for (auto& it : *_vcPoolVal) {
      f64 congestion = std::get<2>(it);
      *_weightVal += hopsVal * congestion + _bias;
    }
    *_weightVal /= _vcPoolVal->size();
  } else {
    *_weightReg = F64_MAX;
  }
}

/*******************LEAST CONGESTED QUEUE ROUTING ALGORITHMS *****************/
void lcqVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  u32 dim;
  u32 portBase = _concentration;
  f64 minCongestion = F64_POS_INF;

  if (_shortCut) {
    // If source == destination, don't pick intermediate address
    if (isDestinationRouter(_router, _destinationAddress)) {
      return;
    }
  }

  // determine available dimensions
  for (dim = 0; dim < _dimensionWidths.size(); dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim) - 1; idx++) {
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + (idx * _dimensionWeights.at(dim)) + weight;
        for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
          f64 congestion = _router->congestionStatus(port, vc);
          if (congestion > minCongestion) {
            continue;
          } else if (congestion < minCongestion) {
            minCongestion = congestion;
            _vcPool->clear();
          }
          std::tuple<u32, u32, f64> t(port, vc, congestion);
          bool res = _vcPool->emplace(t).second;
          (void)res;
          assert(res);
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

void lcqPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  u32 dim;
  u32 portBase = _concentration;
  f64 minCongestion = F64_POS_INF;

  if (_shortCut) {
    // If source == destination, don't pick intermediate address
    if (isDestinationRouter(_router, _destinationAddress)) {
      return;
    }
  }

  // determine available dimensions
  for (dim = 0; dim < _dimensionWidths.size(); dim++) {
    for (u32 idx = 0; idx < _dimensionWidths.at(dim) - 1; idx++) {
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + (idx * _dimensionWeights.at(dim)) + weight;
        f64 congestion = getAveragePortCongestion(_router, port,
                                                  _vcSet, _numVcSets, _numVcs);
        if (congestion > minCongestion) {
          continue;
        } else if (congestion < minCongestion) {
          minCongestion = congestion;
          _vcPool->clear();
        }
        std::tuple<u32, u32, f64> t(port, 0, congestion);
        bool res = _vcPool->emplace(t).second;
        (void)res;
        assert(res);
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

/********************DAL ROUTING ALGORITHMS **********************************/
void doalPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer) {
  _outputVcsMin->clear();
  _outputVcsDer->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  u32 derouted = (_vcSet - _baseVc) % 2;

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      break;
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dim != routerAddress.size()) {
    // more router-to-router hops needed
    u32 src = routerAddress.at(dim);
    u32 dst = _destinationAddress->at(dim+1);
    if (dst < src) {
      dst += _dimensionWidths.at(dim);
    }
    u32 offsetMin = (dst - src - 1) * _dimensionWeights.at(dim);

    // add all ports where the two routers are connecting to outputPortsMin
    // add all other ports to outputPortsDer
    for (u32 offset = 0; offset < _dimensionWidths.at(dim) - 1; offset++) {
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + offset + weight;

        // first look for minimal routes with VCset == 0
        if (offset == offsetMin) {
          f64 congestion = getAveragePortCongestion(
              _router, port, _vcSet, _numVcSets, _numVcs);
          std::tuple<u32, u32, f64> t(port, 0, congestion);
          bool res = _outputVcsMin->emplace(t).second;
          (void)res;
          assert(res);
        }

        // then look at derouting from minimal with VCset == 1
        // we can do that if we didn't deroute previously
        if (offset != offsetMin) {
          if (derouted == 0) {
            f64 congestion = getAveragePortCongestion(
                _router, port, _vcSet, _numVcSets, _numVcs);
            std::tuple<u32, u32, f64> t(port, 0, congestion);
            bool res = _outputVcsDer->emplace(t).second;
            (void)res;
            assert(res);
          }
        }
      }
    }
  }
}

void doalVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer) {
  _outputVcsMin->clear();
  _outputVcsDer->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  u32 derouted = (_vcSet - _baseVc) % 2;

  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      break;
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dim != routerAddress.size()) {
    // more router-to-router hops needed
    u32 src = routerAddress.at(dim);
    u32 dst = _destinationAddress->at(dim+1);
    if (dst < src) {
      dst += _dimensionWidths.at(dim);
    }
    u32 offsetMin = (dst - src - 1) * _dimensionWeights.at(dim);

    // add all ports where the two routers are connecting to outputPortsMin
    // add all other ports to outputPortsDer
    for (u32 offset = 0; offset < _dimensionWidths.at(dim) - 1; offset++) {
      for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
        u32 port = portBase + offset + weight;
        // first look for minimal routes with VCset == 0
        if (offset == offsetMin) {
          for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
            f64 congestion = _router->congestionStatus(port, vc);
            std::tuple<u32, u32, f64> t(port, vc, congestion);
            bool res = _outputVcsMin->emplace(t).second;
            (void)res;
            assert(res);
          }
        }

        // then look at derouting from minimal with VCset == 1
        // we can do that if we didn't deroute previously
        if (offset != offsetMin) {
          for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
            if (derouted == 0) {
              f64 congestion = _router->congestionStatus(port, vc);
              std::tuple<u32, u32, f64> t(port, vc, congestion);
              bool res = _outputVcsDer->emplace(t).second;
              (void)res;
              assert(res);
            }
          }
        }
      }
    }
  }
}

void ddalPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer) {
  _outputVcsMin->clear();
  _outputVcsDer->clear();
  Packet* packet = _flit->packet();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  if (packet->getHopCount() == 1) {
    assert(packet->getRoutingExtension() == nullptr);
    std::vector<u32>* re = new std::vector<u32>(_dimensionWidths.size(), 0);
    packet->setRoutingExtension(re);
  }

  std::set<u32> dimensions;
  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      dimensions.emplace(dim);
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dimensions.empty()) {
    return;
  }

  portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (dimensions.find(dim) != dimensions.end()) {
      // more router-to-router hops needed
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offsetMin = (dst - src - 1) * _dimensionWeights.at(dim);
      // get a const pointer to the address (with leading dummy)
      std::vector<u32>* deroutedDims =
          reinterpret_cast<std::vector<u32>*>(packet->getRoutingExtension());
      u32 derouted = deroutedDims->at(dim);

      // add all ports where the two routers are connecting to outputPortsMin
      // add all other ports to outputPortsDer
      for (u32 offset = 0; offset < _dimensionWidths.at(dim) - 1; offset++) {
        for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
          u32 port = portBase + offset + weight;
          if ((offset != offsetMin) && (derouted == 0)) {
            f64 congestion = getAveragePortCongestion(
                _router, port, _vcSet, _numVcSets, _numVcs);
            std::tuple<u32, u32, f64> t(port, 0, congestion);
            bool res = _outputVcsDer->emplace(t).second;
            (void)res;
            assert(res);
          }
          if ((offset == offsetMin) && (derouted >  0)) {
            f64 congestion = getAveragePortCongestion(
                _router, port, _vcSet, _numVcSets, _numVcs);
            assert(derouted == 1);
            std::tuple<u32, u32, f64> t(port, 0, congestion);
            bool res = _outputVcsMin->emplace(t).second;
            (void)res;
            assert(res);
          }
          if ((offset == offsetMin) && (derouted == 0)) {
            f64 congestion = getAveragePortCongestion(
                _router, port, _vcSet, _numVcSets, _numVcs);
            std::tuple<u32, u32, f64> t(port, 0, congestion);
            bool res = _outputVcsMin->emplace(t).second;
            (void)res;
            assert(res);
          }
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

void ddalVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer) {
  _outputVcsMin->clear();
  _outputVcsDer->clear();
  Packet* packet = _flit->packet();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  if (packet->getHopCount() == 1) {
    assert(packet->getRoutingExtension() == nullptr);
    std::vector<u32>* re = new std::vector<u32>(_dimensionWidths.size(), 0);
    packet->setRoutingExtension(re);
  }

  std::set<u32> dimensions;
  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      dimensions.emplace(dim);
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dimensions.empty()) {
    return;
  }

  portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (dimensions.find(dim) != dimensions.end()) {
      // more router-to-router hops needed
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offsetMin = (dst - src - 1) * _dimensionWeights.at(dim);
      // get a const pointer to the address (with leading dummy)
      std::vector<u32>* deroutedDims =
          reinterpret_cast<std::vector<u32>*>(packet->getRoutingExtension());
      u32 derouted = deroutedDims->at(dim);

      // add all ports where the two routers are connecting to outputPortsMin
      // add all other ports to outputPortsDer
      for (u32 offset = 0; offset < _dimensionWidths.at(dim) - 1; offset++) {
        for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
          u32 port = portBase + offset + weight;
          for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
            if ((offset != offsetMin) && (derouted == 0)) {
              // We can mark dim as derouted here, and that's truly sad
              // deroutedDims->at(dim)++;
              f64 congestion = _router->congestionStatus(port, vc);
              std::tuple<u32, u32, f64> t(port, vc, congestion);
              bool res = _outputVcsDer->emplace(t).second;
              (void)res;
              assert(res);
            }
            if ((offset == offsetMin) && (derouted >  0)) {
              assert(derouted == 1);
              f64 congestion = _router->congestionStatus(port, vc);
              std::tuple<u32, u32, f64> t(port, vc, congestion);
              bool res = _outputVcsMin->emplace(t).second;
              (void)res;
              assert(res);
            }
          }
          for (u32 vc = _vcSet + 1; vc < _numVcs; vc += _numVcSets) {
            if ((offset == offsetMin) && (derouted == 0)) {
              f64 congestion = _router->congestionStatus(port, vc);
              std::tuple<u32, u32, f64> t(port, vc, congestion);
              bool res = _outputVcsMin->emplace(t).second;
              (void)res;
              assert(res);
            }
          }
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

void vdalPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress, u32 _inputPort, u32 _baseVc,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit, bool _multiDeroute,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer) {
  _outputVcsMin->clear();
  _outputVcsDer->clear();
  Packet* packet = _flit->packet();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  u32 hops = packet->getHopCount();
  u32 hopsleft = hopsLeft(_router, _destinationAddress);
  assert((hops - _vcSet + _baseVc) == 1);
  assert(hopsleft == 0 || hopsleft <= (_numVcSets - _vcSet + _baseVc));

  u32 prevDim = computeInputPortDim(_dimensionWidths, _dimensionWeights,
                                    _concentration, _inputPort);

  bool allowDeroutes = (hopsleft + _vcSet - _baseVc) > _numVcSets ? true
                                                                  : false;

  std::set<u32> dimensions;
  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      dimensions.emplace(dim);
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dimensions.empty()) {
    return;
  }

  portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (dimensions.find(dim) != dimensions.end()) {
      // more router-to-router hops needed
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offsetMin = (dst - src - 1) * _dimensionWeights.at(dim);
      if ((dim == prevDim) && !_multiDeroute) {
        allowDeroutes = false;
      }

      // add all ports where the two routers are connecting to outputPortsMin
      // add all other ports to outputPortsDer
      for (u32 offset = 0; offset < _dimensionWidths.at(dim) - 1; offset++) {
        for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
          u32 port = portBase + offset + weight;
          if ((offset != offsetMin) && allowDeroutes) {
            f64 congestion = getAveragePortCongestion(
                _router, port, _vcSet, _numVcSets, _numVcs);
            std::tuple<u32, u32, f64> t(port, 0, congestion);
            bool res = _outputVcsDer->emplace(t).second;
            (void)res;
            assert(res);
          }
          if (offset == offsetMin) {
            f64 congestion = getAveragePortCongestion(
                _router, port, _vcSet, _numVcSets, _numVcs);
            std::tuple<u32, u32, f64> t(port, 0, congestion);
            bool res = _outputVcsMin->emplace(t).second;
            (void)res;
            assert(res);
          }
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

void vdalVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress, u32 _inputPort, u32 _baseVc,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit, bool _multiDeroute,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer) {
  _outputVcsMin->clear();
  _outputVcsDer->clear();
  Packet* packet = _flit->packet();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  u32 hops = packet->getHopCount();
  u32 hopsleft = hopsLeft(_router, _destinationAddress);
  assert((hops - _vcSet + _baseVc) == 1);
  assert(hopsleft == 0 || hopsleft <= (_numVcSets - _vcSet + _baseVc));

  u32 prevDim = computeInputPortDim(_dimensionWidths, _dimensionWeights,
                                    _concentration, _inputPort);

  bool allowDeroutes = (hopsleft + _vcSet - _baseVc) > _numVcSets ? true
                                                                  : false;

  std::set<u32> dimensions;
  // determine the next dimension to work on
  u32 dim;
  u32 portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (routerAddress.at(dim) != _destinationAddress->at(dim+1)) {
      dimensions.emplace(dim);
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }

  // test if already at destination router
  if (dimensions.empty()) {
    return;
  }

  portBase = _concentration;
  for (dim = 0; dim < routerAddress.size(); dim++) {
    if (dimensions.find(dim) != dimensions.end()) {
      // more router-to-router hops needed
      u32 src = routerAddress.at(dim);
      u32 dst = _destinationAddress->at(dim+1);
      if (dst < src) {
        dst += _dimensionWidths.at(dim);
      }
      u32 offsetMin = (dst - src - 1) * _dimensionWeights.at(dim);
      if ((dim == prevDim) && !_multiDeroute) {
        allowDeroutes = false;
      }

      // add all ports where the two routers are connecting to outputPortsMin
      // add all other ports to outputPortsDer
      for (u32 offset = 0; offset < _dimensionWidths.at(dim) - 1; offset++) {
        for (u32 weight = 0; weight < _dimensionWeights.at(dim); weight++) {
          u32 port = portBase + offset + weight;
          for (u32 vc = _vcSet; vc < _numVcs; vc += _numVcSets) {
            if ((offset != offsetMin) && allowDeroutes) {
              f64 congestion = _router->congestionStatus(port, vc);
              std::tuple<u32, u32, f64> t(port, vc, congestion);
              bool res = _outputVcsDer->emplace(t).second;
              (void)res;
              assert(res);
            }
            if (offset == offsetMin) {
              f64 congestion = _router->congestionStatus(port, vc);
              std::tuple<u32, u32, f64> t(port, vc, congestion);
              bool res = _outputVcsMin->emplace(t).second;
              (void)res;
              assert(res);
            }
          }
        }
      }
    }
    portBase += ((_dimensionWidths.at(dim) - 1) * _dimensionWeights.at(dim));
  }
}

/********************SKIPPING DIMENSIONALLY ORDERED ROUTING ALGORITHMS********/

void skippingDimOrderRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _startingDim, u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    Flit* _flit, f64 _iBias, f64 _cBias, f64 _step, f64 _threshold,
    SkippingRoutingAlg _routingAlg, DecisionScheme _decisionScheme,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs1,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs2,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs3,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _outputVcs1->clear();
  _outputVcs2->clear();
  _outputVcs3->clear();
  _vcPool->clear();

  // ex: [x,y,z] for router, [c,x,y,z] for destination
  const std::vector<u32>& routerAddress = _router->address();
  assert(routerAddress.size() == (_destinationAddress->size() - 1));

  std::vector<u32> fakeDestinationAddress(*_destinationAddress);
  for (u32 dim = 0; dim < _startingDim; dim++) {
    fakeDestinationAddress.at(dim+1) = routerAddress.at(dim);
  }

  u32 hops = hopsLeft(_router, _destinationAddress);

  // Put into outputVcs1 routing options without skipping dimension
  if (_routingAlg == SkippingRoutingAlg::DORV) {
    dimOrderVcRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                            _concentration, &fakeDestinationAddress,
                            _vcSet, _numVcSets, _numVcs, _outputVcs1);
  } else if (_routingAlg == SkippingRoutingAlg::DORP) {
    dimOrderPortRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                              _concentration, &fakeDestinationAddress,
                              _vcSet, _numVcSets, _numVcs, _outputVcs1);
  } else {
    if (_routingAlg == SkippingRoutingAlg::DOALV) {
      doalVcRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                          _concentration, &fakeDestinationAddress,
                          _baseVc, _vcSet, _numVcSets, _numVcs, _flit,
                          _outputVcs1, _outputVcs2);
    } else if (_routingAlg == SkippingRoutingAlg::DOALP) {
      doalPortRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                            _concentration, &fakeDestinationAddress,
                            _baseVc, _vcSet, _numVcSets, _numVcs, _flit,
                            _outputVcs1, _outputVcs2);
    } else {
      fprintf(stderr, "Inappropriate skipping routing algorithm\n");
      assert(false);
    }

    if (_decisionScheme == DecisionScheme::W) {
      weightedDecisionScheme(*_outputVcs1, *_outputVcs2,
                             _vcSet, _numVcSets, _numVcs, hops, 1, _iBias,
                             _cBias, false, _vcPool);
      _vcPool->swap(*_outputVcs1);
      _vcPool->clear();
    } else if (_decisionScheme == DecisionScheme::T) {
      thresholdDecisionScheme(*_outputVcs1, *_outputVcs2,
                              _vcSet, _numVcSets, _numVcs, _threshold,
                              _iBias, _vcPool);
      _vcPool->swap(*_outputVcs1);
      _vcPool->clear();
    } else {
      fprintf(stderr, "Inappropriate decision scheme\n");
      assert(false);
    }
  }

  for (u32 dim = _startingDim; dim < _dimensionWidths.size(); dim++) {
    fakeDestinationAddress.at(dim+1) = routerAddress.at(dim);
    u32 skippedDims = dim - _startingDim + 1;

    // Put into outputVcs2 routing options with the next dimension skipped
    if (_routingAlg == SkippingRoutingAlg::DORV) {
      dimOrderVcRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                              _concentration, &fakeDestinationAddress,
                              _vcSet, _numVcSets, _numVcs, _outputVcs2);
    } else if (_routingAlg == SkippingRoutingAlg::DORP) {
      dimOrderPortRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                                _concentration, &fakeDestinationAddress,
                                _vcSet, _numVcSets, _numVcs, _outputVcs2);
    } else {
      if (_routingAlg == SkippingRoutingAlg::DOALV) {
        doalVcRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                            _concentration, &fakeDestinationAddress,
                            _baseVc, _vcSet, _numVcSets, _numVcs, _flit,
                            _outputVcs2, _outputVcs3);
      } else if (_routingAlg == SkippingRoutingAlg::DOALP) {
        doalPortRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                              _concentration, &fakeDestinationAddress,
                              _baseVc, _vcSet, _numVcSets, _numVcs, _flit,
                              _outputVcs2, _outputVcs3);
      } else {
        fprintf(stderr, "Inappropriate skipping routing algorithm\n");
        assert(false);
      }

      if (_decisionScheme == DecisionScheme::W) {
        weightedDecisionScheme(*_outputVcs2, *_outputVcs3,
                               _vcSet, _numVcSets, _numVcs, hops, 1, _iBias,
                               _cBias, false, _vcPool);
        _outputVcs2->clear();
        _vcPool->swap(*_outputVcs2);
      } else if (_decisionScheme == DecisionScheme::T) {
        thresholdDecisionScheme(*_outputVcs2, *_outputVcs3,
                                _vcSet, _numVcSets, _numVcs, _threshold,
                                _iBias, _vcPool);
        _outputVcs2->clear();
        _vcPool->swap(*_outputVcs2);
      } else {
        fprintf(stderr, "Inappropriate decision scheme\n");
        assert(false);
      }
    }

    // Make a choice between previous iteration (outputVcs1) and current
    // dimension skipped (outputVcs2), put result in outputVc1
    if (_decisionScheme == DecisionScheme::W) {
      weightedDecisionScheme(*_outputVcs1, *_outputVcs2,
                             _vcSet, _numVcSets, _numVcs, 1, 0, 0,
                             _step * skippedDims, true, _vcPool);
    } else if (_decisionScheme == DecisionScheme::T) {
      thresholdDecisionScheme(*_outputVcs1, *_outputVcs2,
                              _vcSet, _numVcSets, _numVcs, _threshold,
                              _step * skippedDims, _vcPool);
      if (!_vcPool->empty()) {
        break;
      }
    } else {
      fprintf(stderr, "Inappropriate decision scheme\n");
      assert(false);
    }
    _outputVcs1->swap(*_vcPool);
  }
  _vcPool->swap(*_outputVcs1);
}

void finishingDimOrderRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    Flit* _flit, f64 _iBias, f64 _cBias, f64 _threshold,
    SkippingRoutingAlg _routingAlg, DecisionScheme _decisionScheme,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs1,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs2,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _outputVcs1->clear();
  _outputVcs2->clear();
  _vcPool->clear();

  u32 hops = hopsLeft(_router, _destinationAddress);

  if (_routingAlg == SkippingRoutingAlg::DORV) {
    dimOrderVcRoutingOutput(
        _router, _dimensionWidths, _dimensionWeights, _concentration,
        _destinationAddress, _vcSet, _numVcs, _numVcSets,
        _vcPool);
  } else if (_routingAlg == SkippingRoutingAlg::DORP) {
    dimOrderPortRoutingOutput(
        _router, _dimensionWidths, _dimensionWeights, _concentration,
        _destinationAddress, _vcSet, _numVcs, _numVcSets,
        _vcPool);
  } else {
    if (_routingAlg == SkippingRoutingAlg::DOALV) {
      doalVcRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                          _concentration, _destinationAddress,
                          _baseVc, _vcSet, _numVcs, _numVcSets, _flit,
                          _outputVcs1, _outputVcs2);
    } else if (_routingAlg == SkippingRoutingAlg::DOALP) {
      doalPortRoutingOutput(_router, _dimensionWidths, _dimensionWeights,
                            _concentration, _destinationAddress,
                            _baseVc, _vcSet, _numVcs, _numVcSets, _flit,
                            _outputVcs1, _outputVcs2);
    } else {
      fprintf(stderr, "Inappropriate skipping routing algorithm\n");
      assert(false);
    }

    if (_decisionScheme == DecisionScheme::W) {
      weightedDecisionScheme(*_outputVcs1, *_outputVcs2,
                             _vcSet, _numVcs, _numVcSets, hops, 1, _iBias,
                             _cBias, true, _vcPool);
    } else if (_decisionScheme == DecisionScheme::T) {
      thresholdDalDecisionScheme(*_outputVcs1, *_outputVcs2,
                                 _vcSet, _numVcs, _numVcSets, _threshold,
                                 _iBias, _vcPool);
    } else {
      fprintf(stderr, "Inappropriate decision scheme\n");
      assert(false);
    }
  }
}

/********************DECISION SCHEMES ****************************************/

void thresholdDecisionScheme(
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsMin,
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsDer,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, f64 _threshold, f64 _bias,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  for (auto& it : _outputVcsMin) {
    f64 congestion = std::get<2>(it);
    if (congestion < _threshold) {
      _vcPool->emplace(it);
    }
  }

  if (_vcPool->empty() && !_outputVcsDer.empty()) {
    for (auto& it : _outputVcsDer) {
      f64 congestion = std::get<2>(it);
      if (congestion < (_threshold - _bias)) {
        _vcPool->emplace(it);
      }
    }
  }
}

void thresholdDalDecisionScheme(
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsMin,
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsDer,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, f64 _threshold, f64 _bias,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  for (auto& it : _outputVcsMin) {
    f64 congestion = std::get<2>(it);
    if (congestion < _threshold) {
      _vcPool->emplace(it);
    }
  }

  if (_vcPool->empty() && !_outputVcsDer.empty()) {
    for (auto& it : _outputVcsDer) {
      f64 congestion = std::get<2>(it);
      if (congestion < (_threshold - _bias)) {
        _vcPool->emplace(it);
      }
    }
  }

  if (_vcPool->empty()) {
    for (auto& it : _outputVcsMin) {
      _vcPool->emplace(it);
    }
  }
}

void weightedDecisionScheme(
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsMin,
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsDer,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, f64 _hopsLeft, f64 _hopsIncr,
    f64 _iBias, f64 _cBias, bool _conservative,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool) {
  _vcPool->clear();

  f64 weightMin = F64_MAX;

  for (auto& it : _outputVcsMin) {
    f64 congestion = std::get<2>(it);
    f64 weight = _hopsLeft * congestion;
    if (weight < weightMin) {
      weightMin = weight;
      _vcPool->clear();
      _vcPool->emplace(it);
    } else if ((weight == weightMin) && !_conservative) {
      _vcPool->emplace(it);
    }
  }

  for (auto& it : _outputVcsDer) {
    f64 congestion = std::get<2>(it);
    f64 weight = (_hopsLeft + _hopsIncr) * (congestion + _cBias) + _iBias;
    if (weight < weightMin) {
      // std::cout << "\n !!! Taking deroute !!!\n";
      weightMin = weight;
      _vcPool->clear();
      _vcPool->emplace(it);
    } else if ((weight == weightMin) && !_conservative) {
      _vcPool->emplace(it);
    }
  }
}

}  // namespace HyperX
