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
#include "network/hyperx/UgalRoutingAlgorithm.h"

#include <cassert>

#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

UgalRoutingAlgorithm::UgalRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights,
    u32 _concentration, Json::Value _settings)
    : RoutingAlgorithm(_name, _parent, _router, _latency, _baseVc, _numVcs),
      dimensionWidths_(_dimensionWidths),
      dimensionWeights_(_dimensionWeights),
      concentration_(_concentration) {
  // VC set mapping:
  //  0 = injection from terminal port, to intermediate destination
  //  1 = switching dimension increments VC count
  //  ...
  //  N = last hop to dimension N to intermediate destination
  //  N + 1 = first hop from intermediate to final destination
  //  ...
  //  2N = last hop to dimension N
  //  we can eject flit to destination terminal using any VC

  assert(_settings.isMember("minimal") && _settings["minimal"].isString());
  assert(_settings.isMember("non_minimal") &&
         _settings["non_minimal"].isString());
  assert(_settings.isMember("output_type") &&
         _settings["output_type"].isString());
  assert(_settings.isMember("max_outputs") &&
         _settings["max_outputs"].isUInt());

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

  maxOutputs_ = _settings["max_outputs"].asUInt();

  std::string minimalType = _settings["minimal"].asString();
  std::string outputType = _settings["output_type"].asString();
  std::string nonMinimalType = _settings["non_minimal"].asString();

  firstHopMultiPort_ = _settings["first_hop_multi_port"].asBool();
  shortCut_ = _settings["short_cut"].asBool();

  assert(_settings.isMember("bias"));
  bias_ = _settings["bias"].asFloat();

  if (nonMinimalType == "valiants") {
    nonMinimalAlg_ = NonMinRoutingAlg::VAL;
    assert(_settings.isMember("intermediate_node") &&
           _settings["intermediate_node"].isString());
    std::string intermediateNode = _settings["intermediate_node"].asString();
    if (intermediateNode == "regular") {
      intNodeAlg_ = IntNodeAlg::REG;
    } else if (intermediateNode == "source") {
      intNodeAlg_ = IntNodeAlg::SRC;
    } else if (intermediateNode == "dest") {
      intNodeAlg_ = IntNodeAlg::DST;
    } else if (intermediateNode == "source_dest") {
      intNodeAlg_ = IntNodeAlg::SRCDST;
    } else if (intermediateNode == "unaligned") {
      intNodeAlg_ = IntNodeAlg::UNALIGNED;
    } else if (intermediateNode == "minimal_vc") {
      intNodeAlg_ = IntNodeAlg::MINV;
    } else if (intermediateNode == "minimal_port") {
      intNodeAlg_ = IntNodeAlg::MINP;
    } else {
      fprintf(stderr, "Unknown inter node algorithm:");
      fprintf(stderr, " '%s'\n", intermediateNode.c_str());
      assert(false);
    }
  } else if (nonMinimalType == "least_congested_queue") {
    if (outputType == "vc") {
      nonMinimalAlg_ = NonMinRoutingAlg::LCQV;
      intNodeAlg_ = IntNodeAlg::NONE;
    } else if (outputType == "port") {
      nonMinimalAlg_ = NonMinRoutingAlg::LCQP;
      intNodeAlg_ = IntNodeAlg::NONE;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else {
    fprintf(stderr, "Unknown non-minimal algorithm:");
    fprintf(stderr, " '%s'\n", nonMinimalType.c_str());
    assert(false);
  }

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
    fprintf(stderr, "Unknown minimal algorithm:");
    fprintf(stderr, " '%s'\n", minimalType.c_str());
    assert(false);
  }

  if ((routingAlg_ == BaseRoutingAlg::DORV) ||
      (routingAlg_ == BaseRoutingAlg::DORP)) {
    assert(_numVcs >= 2);
  } else if ((intNodeAlg_ == IntNodeAlg::REG) ||
             (intNodeAlg_ == IntNodeAlg::UNALIGNED)) {
    assert(_numVcs >= _dimensionWidths.size() * 2);
  } else {
    assert(_numVcs >= _dimensionWidths.size() + 1);
  }
}

UgalRoutingAlgorithm::~UgalRoutingAlgorithm() {}

void UgalRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  const std::vector<u32>* destAddress =
      _flit->packet()->message()->getDestinationAddress();
  Packet* packet = _flit->packet();
  f64 weightReg = 0.0, weightVal = 0.0;

  u32 vcSet = U32_MAX;
  u32 numVcSets = (routingAlg_ == BaseRoutingAlg::DORP) ||
                          (routingAlg_ == BaseRoutingAlg::DORV)
                      ? 1
                      : dimensionWidths_.size();
  if ((intNodeAlg_ == IntNodeAlg::REG) ||
      (intNodeAlg_ == IntNodeAlg::UNALIGNED)) {
    numVcSets *= 2;
  } else {
    numVcSets += 1;
  }

  if (packet->getHopCount() == 1) {
    vcSet = baseVc_;
  } else {
    if ((routingAlg_ == BaseRoutingAlg::DORP) ||
        (routingAlg_ == BaseRoutingAlg::DORV)) {
      vcSet = baseVc_ + (_flit->getVc() - baseVc_) % numVcSets;
    } else {
      vcSet = baseVc_ + (_flit->getVc() - baseVc_ + 1) % numVcSets;
    }
  }

  ugalRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                    concentration_, vcSet, numVcSets, baseVc_ + numVcs_,
                    shortCut_, intNodeAlg_, routingAlg_, nonMinimalAlg_, bias_,
                    _flit, &weightReg, &weightVal, &vcPoolReg_, &vcPoolVal_);

  if ((routingAlg_ == BaseRoutingAlg::DORP) ||
      (routingAlg_ == BaseRoutingAlg::RMINP) ||
      (routingAlg_ == BaseRoutingAlg::AMINP)) {
    makeOutputPortSet(&vcPoolReg_, vcSet, numVcSets, baseVc_ + numVcs_,
                      maxOutputs_, outputAlg_, &outputPortsReg_);
    makeOutputPortSet(&vcPoolVal_, vcSet, numVcSets, baseVc_ + numVcs_,
                      maxOutputs_, outputAlg_, &outputPortsVal_);
  } else if ((routingAlg_ == BaseRoutingAlg::DORV) ||
             (routingAlg_ == BaseRoutingAlg::RMINV) ||
             (routingAlg_ == BaseRoutingAlg::AMINV)) {
    makeOutputVcSet(&vcPoolReg_, maxOutputs_, outputAlg_, &outputPortsReg_);
    makeOutputVcSet(&vcPoolVal_, maxOutputs_, outputAlg_, &outputPortsVal_);
  } else {
    fprintf(stderr, "Unknown routing algorithm\n");
    assert(false);
  }

  if (((outputPortsReg_.empty()) && packet->getHopCount() == 1) ||
      ((outputPortsVal_.empty()) && packet->getHopCount() != 1)) {
    delete reinterpret_cast<const std::vector<u32>*>(
        packet->getRoutingExtension());
    packet->setRoutingExtension(nullptr);
    // we can use any VC to eject packet
    for (u64 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
      _response->add(destAddress->at(0), vc);
    }
    return;
  }

  if (weightVal < weightReg) {
    for (auto& it : outputPortsVal_) {
      _response->add(std::get<0>(it), std::get<1>(it));
    }
  } else {
    delete reinterpret_cast<const std::vector<u32>*>(
        packet->getRoutingExtension());
    packet->setRoutingExtension(nullptr);
    for (auto& it : outputPortsReg_) {
      _response->add(std::get<0>(it), std::get<1>(it));
    }
  }
}

}  // namespace HyperX
