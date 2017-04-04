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
#include "network/hyperx/DalRoutingAlgorithm.h"

#include <cassert>

#include <iostream>

#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

DalRoutingAlgorithm::DalRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights,
    u32 _concentration, u32 _inputPort, Json::Value _settings)
    : RoutingAlgorithm(_name, _parent, _router, _latency, _baseVc, _numVcs),
      dimensionWidths_(_dimensionWidths),
      dimensionWeights_(_dimensionWeights),
      concentration_(_concentration),
      inputPort_(_inputPort) {
  assert(_settings.isMember("adaptivity_type") &&
         _settings["adaptivity_type"].isString());
  assert(_settings.isMember("output_type") &&
         _settings["output_type"].isString());
  assert(_settings.isMember("decision_scheme") &&
         _settings["decision_scheme"].isString());

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

  assert(_settings.isMember("threshold"));
  threshold_ = _settings["threshold"].asFloat();
  assert(_settings.isMember("independent_bias"));
  iBias_ = _settings["independent_bias"].asFloat();

  multiDeroute_ = _settings.isMember("multi_deroute")
                      ? _settings["multi_deroute"].asBool()
                      : false;

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

  if (_settings["adaptivity_type"].asString() == "dimension_adaptive") {
    if (outputType == "vc") {
      adaptivityType_ = AdaptiveRoutingAlg::DDALV;
    } else if (outputType == "port") {
      adaptivityType_ = AdaptiveRoutingAlg::DDALP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else if (_settings["adaptivity_type"].asString() == "dimension_order") {
    if (outputType == "vc") {
      adaptivityType_ = AdaptiveRoutingAlg::DOALV;
    } else if (outputType == "port") {
      adaptivityType_ = AdaptiveRoutingAlg::DOALP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else if (_settings["adaptivity_type"].asString() == "variable") {
    assert(_settings.isMember("max_hops"));
    maxHopsAllowed_ = _settings["max_hops"].asUInt();
    if (outputType == "vc") {
      adaptivityType_ = AdaptiveRoutingAlg::VDALV;
    } else if (outputType == "port") {
      adaptivityType_ = AdaptiveRoutingAlg::VDALP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else {
    fprintf(stderr, "Unknown adaptive algorithm:");
    fprintf(stderr, " '%s'\n", _settings["adaptivity_type"].asString().c_str());
    assert(false);
  }

  if (_settings["decision_scheme"].asString() == "weighted") {
    decisionScheme_ = DecisionScheme::W;
    assert(_settings.isMember("congestion_bias"));
    cBias_ = _settings["congestion_bias"].asFloat();
  } else if (_settings["decision_scheme"].asString() == "threshold") {
    decisionScheme_ = DecisionScheme::T;
    assert(_settings.isMember("threshold"));
  } else {
    fprintf(stderr, "Unknown decision scheme:");
    fprintf(stderr, " '%s'\n", _settings["decision_scheme"].asString().c_str());
    assert(false);
  }

  if ((adaptivityType_ == AdaptiveRoutingAlg::DOALV) ||
      (adaptivityType_ == AdaptiveRoutingAlg::DOALP)) {
    assert(numVcs_ >= 2);
  } else if ((adaptivityType_ == AdaptiveRoutingAlg::DDALV) ||
             (adaptivityType_ == AdaptiveRoutingAlg::DDALP)) {
    assert(numVcs_ >= dimensionWidths_.size() * 2);
  } else if ((adaptivityType_ == AdaptiveRoutingAlg::VDALV) ||
             (adaptivityType_ == AdaptiveRoutingAlg::VDALP)) {
    assert(numVcs_ >= maxHopsAllowed_);
    assert(maxHopsAllowed_ >= dimensionWidths_.size());
  }

  if ((adaptivityType_ == AdaptiveRoutingAlg::DOALP) ||
      (adaptivityType_ == AdaptiveRoutingAlg::DOALV)) {
    numVcSets_ = 2;
  } else if ((adaptivityType_ == AdaptiveRoutingAlg::DDALP) ||
             (adaptivityType_ == AdaptiveRoutingAlg::DDALV)) {
    numVcSets_ = 2 * dimensionWidths_.size();
  } else if ((adaptivityType_ == AdaptiveRoutingAlg::VDALP) ||
             (adaptivityType_ == AdaptiveRoutingAlg::VDALV)) {
    numVcSets_ = maxHopsAllowed_;
  }
}

DalRoutingAlgorithm::~DalRoutingAlgorithm() {}

void DalRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  Packet* packet = _flit->packet();
  const std::vector<u32>* destinationAddress =
      packet->message()->getDestinationAddress();
  const std::vector<u32>& routerAddress = router_->address();

  u32 vcSet = U32_MAX;
  if ((adaptivityType_ == AdaptiveRoutingAlg::DOALP) ||
      (adaptivityType_ == AdaptiveRoutingAlg::DOALV)) {
    // first hop in dimension is always VC = 0
    // if incoming dimension is un aligned, hence it is a deroute, than VC = 1
    u32 inDim = computeInputPortDim(dimensionWidths_, dimensionWeights_,
                                    concentration_, inputPort_);
    if ((inDim == U32_MAX) ||
        (routerAddress.at(inDim) == destinationAddress->at(inDim + 1))) {
      vcSet = baseVc_ + 0;
    } else {
      vcSet = baseVc_ + 1;
    }
  } else if ((adaptivityType_ == AdaptiveRoutingAlg::DDALP) ||
             (adaptivityType_ == AdaptiveRoutingAlg::DDALV)) {
    if (packet->getHopCount() == 1) {
      vcSet = 0;
    } else {
      vcSet = baseVc_ + (_flit->getVc() - baseVc_ + 1) % numVcSets_;
    }
  } else if ((adaptivityType_ == AdaptiveRoutingAlg::VDALP) ||
             (adaptivityType_ == AdaptiveRoutingAlg::VDALV)) {
    if (packet->getHopCount() == 1) {
      vcSet = 0;
    } else {
      vcSet = baseVc_ + (_flit->getVc() - baseVc_ + 1) % numVcSets_;
    }
  }

  if (adaptivityType_ == AdaptiveRoutingAlg::DOALP) {
    doalPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                          concentration_, destinationAddress, baseVc_, vcSet,
                          numVcSets_, baseVc_ + numVcs_, _flit, &outputVcsMin_,
                          &outputVcsDer_);
  } else if (adaptivityType_ == AdaptiveRoutingAlg::DOALV) {
    doalVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                        concentration_, destinationAddress, baseVc_, vcSet,
                        numVcSets_, baseVc_ + numVcs_, _flit, &outputVcsMin_,
                        &outputVcsDer_);
  } else if (adaptivityType_ == AdaptiveRoutingAlg::DDALP) {
    ddalPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                          concentration_, destinationAddress, vcSet, numVcSets_,
                          baseVc_ + numVcs_, _flit, &outputVcsMin_,
                          &outputVcsDer_);
  } else if (adaptivityType_ == AdaptiveRoutingAlg::DDALV) {
    ddalVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                        concentration_, destinationAddress, vcSet, numVcSets_,
                        baseVc_ + numVcs_, _flit, &outputVcsMin_,
                        &outputVcsDer_);
  } else if (adaptivityType_ == AdaptiveRoutingAlg::VDALP) {
    vdalPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                          concentration_, destinationAddress, inputPort_,
                          baseVc_, vcSet, numVcSets_, baseVc_ + numVcs_,
                          _flit, multiDeroute_, &outputVcsMin_, &outputVcsDer_);
  } else if (adaptivityType_ == AdaptiveRoutingAlg::VDALV) {
    vdalVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                        concentration_, destinationAddress, inputPort_,
                        baseVc_, vcSet, numVcSets_, baseVc_ + numVcs_,
                        _flit, multiDeroute_, &outputVcsMin_, &outputVcsDer_);
  } else {
    fprintf(stderr, "Unknown adaptive algorithm\n");
    assert(false);
  }

  u32 hops = hopsLeft(router_, destinationAddress);
  if (decisionScheme_ == DecisionScheme::W) {
    weightedDecisionScheme(outputVcsMin_, outputVcsDer_, vcSet, numVcSets_,
                           baseVc_ + numVcs_, hops, 1, iBias_, cBias_, true,
                           &vcPool_);
    if (outputTypePort_) {
      makeOutputPortSet(&vcPool_, vcSet, numVcSets_, baseVc_ + numVcs_,
                        maxOutputs_, outputAlg_, &outputPorts_);
    } else {
      makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
    }
  } else if (decisionScheme_ == DecisionScheme::T) {
    thresholdDalDecisionScheme(outputVcsMin_, outputVcsDer_, vcSet, numVcSets_,
                               baseVc_ + numVcs_, threshold_, iBias_, &vcPool_);
    if (outputTypePort_) {
      makeOutputPortSet(&vcPool_, vcSet, numVcSets_, baseVc_ + numVcs_,
                        maxOutputs_, outputAlg_, &outputPorts_);
    } else {
      makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
    }
  } else {
    fprintf(stderr, "Unknown decision scheme\n");
    assert(false);
  }

  if (outputPorts_.empty()) {
    for (u64 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
      _response->add(destinationAddress->at(0), vc);
      if ((adaptivityType_ == AdaptiveRoutingAlg::DDALP) ||
          (adaptivityType_ == AdaptiveRoutingAlg::DDALV)) {
        delete reinterpret_cast<const std::vector<u32>*>(
            packet->getRoutingExtension());
        packet->setRoutingExtension(nullptr);
      }
    }
  } else {
  // std::cout << "\nNumber of options " << outputPorts_.size() << std::endl;
    for (auto& it : outputPorts_) {
      // std::cout << std::get<2>(it) << ", ";
      _response->add(std::get<0>(it), std::get<1>(it));
    }
    // std::cout << std::endl;
  }
}

void DalRoutingAlgorithm::vcScheduled(Flit* _flit, u32 _port, u32 _vc) {
  Packet* packet = _flit->packet();
  const std::vector<u32>* destinationAddress =
      packet->message()->getDestinationAddress();
  const std::vector<u32>& routerAddress = router_->address();

  if ((adaptivityType_ != AdaptiveRoutingAlg::DDALP) &&
      (adaptivityType_ != AdaptiveRoutingAlg::DDALV)) {
    return;
  }

  bool derouted = true;
  if (_port < concentration_) {
    return;
  }

  // u32 portBase = concentration_;
  u32 port = _port - concentration_;
  u32 offset, dim;
  for (dim = 0; dim < dimensionWeights_.size(); dim++) {
    u32 src = routerAddress.at(dim);
    u32 dst = destinationAddress->at(dim + 1);
    if (dst < src) {
      dst += dimensionWidths_.at(dim);
    }
    offset = (dst - src - 1) * dimensionWeights_.at(dim);
    if (port < (dimensionWidths_.at(dim) - 1) * dimensionWeights_.at(dim)) {
      break;
    } else {
      port -= (dimensionWidths_.at(dim) - 1) * dimensionWeights_.at(dim);
      // portBase += (dimensionWidths_.at(dim) - 1) * dimensionWeights_.at(dim);
    }
  }

  for (u32 weight = 0; weight < dimensionWeights_.at(dim); weight++) {
    u32 offsetPort = offset + weight;
    if (offsetPort == port) {
      derouted = false;
    }
  }

  // printf("input port %u, output port %u, dim %u\n", _port, dim);
  std::vector<u32>* deroutedDims =
      reinterpret_cast<std::vector<u32>*>(packet->getRoutingExtension());
  if (derouted) {
    assert(deroutedDims->at(dim) == 0);
    deroutedDims->at(dim)++;
  }
}

}  // namespace HyperX
