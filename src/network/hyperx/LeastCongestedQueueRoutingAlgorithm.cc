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
#include "network/hyperx/LeastCongestedQueueRoutingAlgorithm.h"

#include <cassert>

#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

LeastCongestedQueueRoutingAlgorithm::LeastCongestedQueueRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    Json::Value _settings)
    : RoutingAlgorithm(_name, _parent, _router, _latency, _baseVc, _numVcs),
      dimensionWidths_(_dimensionWidths),
      dimensionWeights_(_dimensionWeights),
      concentration_(_concentration) {
  // VC set mapping:
  //  0 = injection from terminal port, to intermediate destination
  //  1 = first hop from intermediate to final destination
  //  ...
  //  N + 1 = last hop to dimension N
  //  we can eject flit to destination terminal using any VC

  assert(_settings.isMember("minimal") && _settings["minimal"].isString());
  assert(_settings.isMember("output_type") &&
         _settings["output_type"].isString());
  assert(_settings.isMember("max_outputs") &&
         _settings["max_outputs"].isUInt());

  maxOutputs_ = _settings["max_outputs"].asUInt();

  assert(_settings.isMember("output_algorithm") &&
         _settings["output_algorithm"].isString());
  if (_settings["output_algorithm"].asString() == "random") {
    outputAlg_ = OutputAlg::Rand;
  } else if (_settings["output_algorithm"].asString() == "minimal") {
    outputAlg_ = OutputAlg::Min;
  } else {
    fprintf(stderr, "Unknown output algorithm:");
    fprintf(stderr, " '%s'\n",
            _settings["output_algorithm"].asString().c_str());
    assert(false);
  }

  std::string minimalType = _settings["minimal"].asString();
  std::string outputType = _settings["output_type"].asString();
  if (outputType == "port") {
    outputTypePort_ = true;
  } else if (outputType == "vc") {
    outputTypePort_ = false;
  } else {
    fprintf(stderr, "Unknown output type:");
    fprintf(stderr, " '%s'\n", outputType.c_str());
    assert(false);
  }

  shortCut_ = _settings["short_cut"].asBool();

  if (minimalType == "dimension_order") {
    if (outputType == "vc") {
      routingAlg_ = BaseRoutingAlg::DORV;
    } else if (outputType == "port") {
      routingAlg_ = BaseRoutingAlg::DORP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else if (minimalType == "random") {
    if (outputType == "vc") {
      routingAlg_ = BaseRoutingAlg::RMINV;
    } else if (outputType == "port") {
      routingAlg_ = BaseRoutingAlg::RMINP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else if (minimalType == "adaptive") {
    if (outputType == "vc") {
      routingAlg_ = BaseRoutingAlg::AMINV;
    } else if (outputType == "port") {
      routingAlg_ = BaseRoutingAlg::AMINP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else {
    fprintf(stderr, "Unknown inter minimal algorithm:");
    fprintf(stderr, " '%s'\n", minimalType.c_str());
    assert(false);
  }

  if (minimalType == "dimension_order") {
    assert(_numVcs >= 2);
  } else {
    assert(_numVcs >= _dimensionWidths.size() + 1);
  }
}

LeastCongestedQueueRoutingAlgorithm::~LeastCongestedQueueRoutingAlgorithm() {}

void LeastCongestedQueueRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  const std::vector<u32>* destinationAddress =
      _flit->packet()->message()->getDestinationAddress();

  Packet* packet = _flit->packet();
  u32 vcSet = U32_MAX;
  u32 numVcSets = (routingAlg_ == BaseRoutingAlg::DORP) ||
                          (routingAlg_ == BaseRoutingAlg::DORV)
                      ? 2
                      : dimensionWidths_.size() + 1;

  if (packet->getHopCount() == 1) {
    vcSet = baseVc_;
  } else if (packet->getHopCount() == 2) {
    vcSet = baseVc_ + 1;
  } else {
    if ((routingAlg_ == BaseRoutingAlg::DORP) ||
        (routingAlg_ == BaseRoutingAlg::DORV)) {
      vcSet = baseVc_ + (_flit->getVc() - baseVc_) % numVcSets;
    } else {
      vcSet = baseVc_ + (_flit->getVc() - baseVc_ + 1) % numVcSets;
    }
  }

  if (packet->getHopCount() == 1) {
    if (outputTypePort_) {
      lcqPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                           concentration_, destinationAddress, vcSet, numVcSets,
                           baseVc_ + numVcs_, shortCut_, &vcPool_);
      makeOutputPortSet(&vcPool_, vcSet, numVcSets, baseVc_ + numVcs_,
                        maxOutputs_, outputAlg_, &outputPorts_);
    } else {
      lcqVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                         concentration_, destinationAddress, vcSet, numVcSets,
                         baseVc_ + numVcs_, shortCut_, &vcPool_);
      makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
    }
  } else {
    switch (routingAlg_) {
      case BaseRoutingAlg::DORV: {
        dimOrderVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                                concentration_, destinationAddress, vcSet,
                                numVcSets, baseVc_ + numVcs_, &vcPool_);
        makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
        break;
      }
      case BaseRoutingAlg::DORP: {
        dimOrderPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                                  concentration_, destinationAddress, vcSet,
                                  numVcSets, baseVc_ + numVcs_, &vcPool_);
        makeOutputPortSet(&vcPool_, vcSet, numVcSets, baseVc_ + numVcs_,
                          maxOutputs_, outputAlg_, &outputPorts_);
        break;
      }
      case BaseRoutingAlg::RMINV: {
        randMinVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                               concentration_, destinationAddress, vcSet,
                               numVcSets, baseVc_ + numVcs_, &vcPool_);
        makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
        break;
      }
      case BaseRoutingAlg::RMINP: {
        randMinPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                                 concentration_, destinationAddress, vcSet,
                                 numVcSets, baseVc_ + numVcs_, &vcPool_);
        makeOutputPortSet(&vcPool_, vcSet, numVcSets, baseVc_ + numVcs_,
                          maxOutputs_, outputAlg_, &outputPorts_);
        break;
      }
      case BaseRoutingAlg::AMINV: {
        adaptiveMinVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                                   concentration_, destinationAddress, vcSet,
                                   numVcSets, baseVc_ + numVcs_, &vcPool_);
        makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
        break;
      }
      case BaseRoutingAlg::AMINP: {
        adaptiveMinPortRoutingOutput(
            router_, dimensionWidths_, dimensionWeights_, concentration_,
            destinationAddress, vcSet, numVcSets, baseVc_ + numVcs_, &vcPool_);
        makeOutputPortSet(&vcPool_, vcSet, numVcSets, baseVc_ + numVcs_,
                          maxOutputs_, outputAlg_, &outputPorts_);
        break;
      }
      default: {
        fprintf(stderr, "Unknown routing algorithm\n");
        assert(false);
      }
    }
  }

  if (outputPorts_.empty()) {
    // we can use any VC to eject packet
    for (u64 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
      _response->add(destinationAddress->at(0), vc);
    }
    return;
  }

  for (auto it : outputPorts_) {
    _response->add(std::get<0>(it), std::get<1>(it));
  }
}

}  // namespace HyperX
