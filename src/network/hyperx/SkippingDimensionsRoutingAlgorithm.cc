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
#include "network/hyperx/SkippingDimensionsRoutingAlgorithm.h"

#include <cassert>
#include <iostream>

#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

SkippingDimensionsRoutingAlgorithm::SkippingDimensionsRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _baseVc, u32 _numVcs,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _inputPort, Json::Value _settings)
    : RoutingAlgorithm(_name, _parent, _router, _latency, _baseVc, _numVcs),
      dimensionWidths_(_dimensionWidths),
      dimensionWeights_(_dimensionWeights),
      concentration_(_concentration),
      inputPort_(_inputPort) {
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

  assert(_settings.isMember("num_skip_rounds") &&
         _settings["num_skip_rounds"].isUInt());
  numRounds_ = _settings["num_skip_rounds"].asUInt();

  assert(_settings.isMember("threshold"));
  threshold_ = _settings["threshold"].asFloat();
  assert(_settings.isMember("independent_bias"));
  iBias_ = _settings["independent_bias"].asFloat();
  assert(_settings.isMember("step"));
  step_ = _settings["step"].asFloat();

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

  if (_settings["skipping_algorithm"].asString() == "dimension_adaptive") {
    if (outputType == "vc") {
      skippingType_ = SkippingRoutingAlg::DOALV;
    } else if (outputType == "port") {
      skippingType_ = SkippingRoutingAlg::DOALP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else if (_settings["skipping_algorithm"].asString() == "dimension_order") {
    if (outputType == "vc") {
      skippingType_ = SkippingRoutingAlg::DORV;
    } else if (outputType == "port") {
      skippingType_ = SkippingRoutingAlg::DORP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else {
    fprintf(stderr, "Unknown adaptive algorithm:");
    fprintf(stderr, " '%s'\n",
            _settings["skipping_algorithm"].asString().c_str());
    assert(false);
  }

  if (_settings["finishing_algorithm"].asString() == "dimension_adaptive") {
    if (outputType == "vc") {
      finishingType_ = SkippingRoutingAlg::DOALV;
    } else if (outputType == "port") {
      finishingType_ = SkippingRoutingAlg::DOALP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else if (_settings["finishing_algorithm"].asString() == "dimension_order") {
    if (outputType == "vc") {
      finishingType_ = SkippingRoutingAlg::DORV;
    } else if (outputType == "port") {
      finishingType_ = SkippingRoutingAlg::DORP;
    } else {
      fprintf(stderr, "Unknown output type:");
      fprintf(stderr, " '%s'\n", outputType.c_str());
      assert(false);
    }
  } else {
    fprintf(stderr, "Unknown adaptive algorithm:");
    fprintf(stderr, " '%s'\n",
            _settings["finishing_algorithm"].asString().c_str());
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

  u32 neededVcs = 0;
  u32 fneededVcs = 0;
  if ((skippingType_ == SkippingRoutingAlg::DORV) ||
      (skippingType_ == SkippingRoutingAlg::DORP)) {
    neededVcs = numRounds_ - 1;
  } else if ((skippingType_ == SkippingRoutingAlg::DOALV) ||
             (skippingType_ == SkippingRoutingAlg::DOALP)) {
    neededVcs = 2 * numRounds_ - 2;
  }
  if ((finishingType_ == SkippingRoutingAlg::DORV) ||
      (finishingType_ == SkippingRoutingAlg::DORP)) {
    fneededVcs = 1;
  } else if ((finishingType_ == SkippingRoutingAlg::DOALV) ||
             (finishingType_ == SkippingRoutingAlg::DOALP)) {
    fneededVcs = 2;
  }

  assert(_numVcs >= (numRounds_ - 1) * neededVcs + fneededVcs);

  if ((skippingType_ == SkippingRoutingAlg::DORV) ||
      (skippingType_ == SkippingRoutingAlg::DORP)) {
    numVcSets_ = numRounds_ - 1;
  } else if ((skippingType_ == SkippingRoutingAlg::DOALV) ||
             (skippingType_ == SkippingRoutingAlg::DOALP)) {
    numVcSets_ = 2 * numRounds_ - 2;
  }

  if ((skippingType_ == SkippingRoutingAlg::DORV) ||
      (skippingType_ == SkippingRoutingAlg::DORP)) {
    numVcSets_ += 1;
  } else if ((skippingType_ == SkippingRoutingAlg::DOALV) ||
             (skippingType_ == SkippingRoutingAlg::DOALP)) {
    numVcSets_ += 2;
  }
}

SkippingDimensionsRoutingAlgorithm::~SkippingDimensionsRoutingAlgorithm() {}

void SkippingDimensionsRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  Packet* packet = _flit->packet();
  const std::vector<u32>* destinationAddress =
      packet->message()->getDestinationAddress();
  const std::vector<u32>& routerAddress = router_->address();

  u32 numRound = 0;
  if (packet->getHopCount() > 1) {
    numRound = (_flit->getVc() - baseVc_) % numVcSets_;
  }
  if ((skippingType_ == SkippingRoutingAlg::DOALV) ||
      (skippingType_ == SkippingRoutingAlg::DOALP)) {
    numRound /= 2;
  }

  u32 inDim = computeInputPortDim(dimensionWidths_, dimensionWeights_,
                                  concentration_, inputPort_);
  if (inDim == U32_MAX) {
    inDim = 0;
  } else {
    inDim += 1;
  }

  // std::cout << "Round is " << numRound << " out of " << numRounds_ << "\n";
  u32 vcSet = U32_MAX;
  if (numRound < (numRounds_ - 1)) {
    // Skipping dimensions round
    if ((skippingType_ == SkippingRoutingAlg::DOALV) ||
        (skippingType_ == SkippingRoutingAlg::DOALP)) {
      // first hop in dimension is always VC = 0
      // if incoming dimension is un aligned, hence it is a deroute, than VC = 1
      if ((inDim == 0) ||
          (routerAddress.at(inDim - 1) == destinationAddress->at(inDim))) {
        vcSet = baseVc_ + numRound * 2;
        skippingDimOrderRoutingOutput(
            router_, dimensionWidths_, dimensionWeights_, concentration_,
            destinationAddress, inDim, baseVc_, vcSet, numVcSets_,
            baseVc_ + numVcs_, _flit, iBias_, cBias_, step_, threshold_,
            skippingType_, decisionScheme_, &outputVcs1_, &outputVcs2_,
            &outputVcs3_, &vcPool_);
      } else {
        vcSet = baseVc_ + 1 + numRound * 2;
        if (skippingType_ == SkippingRoutingAlg::DOALV) {
          doalVcRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                              concentration_, destinationAddress, baseVc_,
                              vcSet, numVcSets_, baseVc_ + numVcs_, _flit,
                              &outputVcs1_, &outputVcs2_);
        } else if (skippingType_ == SkippingRoutingAlg::DOALP) {
          doalPortRoutingOutput(router_, dimensionWidths_, dimensionWeights_,
                                concentration_, destinationAddress, baseVc_,
                                vcSet, numVcSets_, baseVc_ + numVcs_, _flit,
                                &outputVcs1_, &outputVcs2_);
        } else {
          fprintf(stderr, "Inappropriate skipping routing algorithm\n");
          assert(false);
        }

        if (decisionScheme_ == DecisionScheme::W) {
          weightedDecisionScheme(outputVcs1_, outputVcs2_, vcSet, numVcSets_,
                                 baseVc_ + numVcs_, 1, 1, iBias_, cBias_, true,
                                 &vcPool_);
        } else if (decisionScheme_ == DecisionScheme::T) {
          thresholdDecisionScheme(outputVcs1_, outputVcs2_, vcSet, numVcSets_,
                                  baseVc_ + numVcs_, threshold_, iBias_,
                                  &vcPool_);
        } else {
          fprintf(stderr, "Inappropriate decision scheme\n");
          assert(false);
        }
      }
    } else if ((skippingType_ == SkippingRoutingAlg::DORV) ||
               (skippingType_ == SkippingRoutingAlg::DORP)) {
      vcSet = baseVc_ + numRound;
      skippingDimOrderRoutingOutput(
          router_, dimensionWidths_, dimensionWeights_, concentration_,
          destinationAddress, inDim, baseVc_, vcSet, numVcSets_,
          baseVc_ + numVcs_, _flit, iBias_, cBias_, step_, threshold_,
          skippingType_, decisionScheme_, &outputVcs1_, &outputVcs2_,
          &outputVcs3_, &vcPool_);
    }
  } else {
    // Finishing round
    if ((finishingType_ == SkippingRoutingAlg::DORV) ||
        (finishingType_ == SkippingRoutingAlg::DORP)) {
      vcSet = baseVc_ + numRound;
    } else {
      if ((inDim == 0) ||
          (routerAddress.at(inDim - 1) == destinationAddress->at(inDim))) {
        vcSet = baseVc_ + numRound * 2;
      } else {
        vcSet = baseVc_ + 1 + numRound * 2;
      }
    }

    finishingDimOrderRoutingOutput(
        router_, dimensionWidths_, dimensionWeights_, concentration_,
        destinationAddress, baseVc_, vcSet, numVcSets_, baseVc_ + numVcs_,
        _flit, iBias_, cBias_, threshold_, finishingType_, decisionScheme_,
        &outputVcs1_, &outputVcs2_, &vcPool_);
  }

  // Round increment if out of dimensions in this round
  if (vcPool_.empty() && !isDestinationRouter(router_, destinationAddress)) {
    // std::cout << "Round is " << numRound << " out of " << numRounds_ << "\n";
    // std::cout << "Src is " << strop::vecString(routerAddress) << std::endl;
    // std::cout << "Dst is " << strop::vecString(*destinationAddress) << "\n";
    numRound += 1;
    if ((finishingType_ == SkippingRoutingAlg::DORV) ||
        (finishingType_ == SkippingRoutingAlg::DORP)) {
      vcSet = baseVc_ + numRound;
    } else {
      if ((inDim == 0) ||
          (routerAddress.at(inDim - 1) == destinationAddress->at(inDim))) {
        vcSet = baseVc_ + numRound * 2;
      } else {
        vcSet = baseVc_ + 1 + numRound * 2;
      }
    }
    if (numRound < (numRounds_ - 1)) {
      skippingDimOrderRoutingOutput(
          router_, dimensionWidths_, dimensionWeights_, concentration_,
          destinationAddress, 0, baseVc_, vcSet, numVcSets_, baseVc_ + numVcs_,
          _flit, iBias_, cBias_, step_, threshold_, skippingType_,
          decisionScheme_, &outputVcs1_, &outputVcs2_, &outputVcs3_, &vcPool_);
      if ((decisionScheme_ == DecisionScheme::T) && vcPool_.empty()) {
        thresholdDalDecisionScheme(outputVcs1_, outputVcs2_, vcSet, numVcSets_,
                                   baseVc_ + numVcs_, threshold_, iBias_,
                                   &vcPool_);
      }
    } else {
      finishingDimOrderRoutingOutput(
          router_, dimensionWidths_, dimensionWeights_, concentration_,
          destinationAddress, baseVc_, vcSet, numVcSets_, baseVc_ + numVcs_,
          _flit, iBias_, cBias_, threshold_, finishingType_, decisionScheme_,
          &outputVcs1_, &outputVcs2_, &vcPool_);
    }
    // std::cout << "input dimension is " << inDim << " src:dst coordinates ";
    // std::cout << routerAddress.at(inDim - 1) << ":" <<
    // destinationAddress->at(inDim) << "\n";
    assert(((decisionScheme_ == DecisionScheme::T) && (inDim == 0)) ||
           ((inDim != 0) &&
            (routerAddress.at(inDim - 1) == destinationAddress->at(inDim))));
  }

  if ((finishingType_ == SkippingRoutingAlg::DOALP) ||
      (finishingType_ == SkippingRoutingAlg::DORP)) {
    makeOutputPortSet(&vcPool_, vcSet, numVcs_, baseVc_ + numVcs_, maxOutputs_,
                      outputAlg_, &outputPorts_);
  } else if ((finishingType_ == SkippingRoutingAlg::DOALV) ||
             (finishingType_ == SkippingRoutingAlg::DORV)) {
    makeOutputVcSet(&vcPool_, maxOutputs_, outputAlg_, &outputPorts_);
  } else {
    fprintf(stderr, "Unknown routing algorithm\n");
    assert(false);
  }

  if (outputPorts_.empty()) {
    // we can use any VC to eject packet
    for (u64 vc = baseVc_; vc < baseVc_ + numVcs_; vc++) {
      _response->add(destinationAddress->at(0), vc);
    }
    return;
  }

  for (auto& it : outputPorts_) {
    _response->add(std::get<0>(it), std::get<1>(it));
  }
}

}  // namespace HyperX
