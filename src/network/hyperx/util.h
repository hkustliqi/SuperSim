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
#ifndef NETWORK_HYPERX_UTIL_H_
#define NETWORK_HYPERX_UTIL_H_

#include <prim/prim.h>

#include <unordered_set>
#include <vector>
#include <tuple>        // std::tuple

#include "router/Router.h"
#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

enum class MinRoutingAlg : u8 {RMINV, RMINP, AMINV, AMINP};
enum class IntNodeAlg : u8 {REG, SRC, DST, SRCDST, UNALIGNED, MINV, MINP, NONE};
enum class BaseRoutingAlg : u8 {DORV, DORP, RMINV, RMINP, AMINV, AMINP};
enum class NonMinRoutingAlg : u8 {VAL, LCQV, LCQP};
enum class AdaptiveRoutingAlg : u8 {DDALV, DDALP, DOALV, DOALP, VDALV, VDALP};
enum class SkippingRoutingAlg : u8 {DORV, DORP, DOALV, DOALP};
enum class DecisionScheme : u8 {W, T, WT};
enum class OutputAlg : u8 {Rand, Min};

typedef void (*IntNodeAlgFunc)(
    Router*, const std::vector<u32>&, const std::vector<u32>*,
    const std::vector<u32>&, const std::vector<u32>&,
    u32, u32, u32, u32, std::vector<u32>*);

typedef void(*MinRoutingAlgFunc)(
    Router*, const std::vector<u32>&, const std::vector<u32>&, u32,
    const std::vector<u32>*, u32, u32, u32,
    std::unordered_set< std::tuple<u32, u32, f64> >*);

typedef void(*FirstHopRoutingAlgFunc)(
    Router*, const std::vector<u32>&, const std::vector<u32>&, const u32,
    const std::vector<u32>*, u32, u32, u32, bool,
    std::unordered_set< std::tuple<u32, u32, f64> >*);

bool isDestinationRouter(
    Router* _router, const std::vector<u32>* _destinationAddress);

u32 hopsLeft(
    Router* _router, const std::vector<u32>* _destinationAddress);

u32 computeInputPortDim(const std::vector<u32>& _dimensionWidths,
                        const std::vector<u32>& _dimensionWeights,
                        u32 _concentration, u32 _inputPort);
void intNodeReg(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void intNodeUnAligned(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void intNodeSrc(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void intNodeDst(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void intNodeSrcDst(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void intNodeMinV(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void intNodeMinP(
    Router* _router, const std::vector<u32>& _sourceRouter,
    const std::vector<u32>* _destinationTerminal,
    const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::vector<u32>* _address);

void makeOutputVcSet(
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool,
    u32 _maxOutputs, OutputAlg _outputAlg,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputPorts);

void makeOutputPortSet(
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    u32 _maxOutputs, OutputAlg _outputAlg,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputPorts);

f64 getAveragePortCongestion(Router* _router, u32 _port,
                             u32 _vcSet, u32 _numVcSets, u32 _numVcs);

void dimOrderVcRoutingOutput(
    Router* router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void dimOrderPortRoutingOutput(
    Router* router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void randMinVcRoutingOutput(
    Router* router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void randMinPortRoutingOutput(
    Router* router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void adaptiveMinVcRoutingOutput(
    Router* router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void adaptiveMinPortRoutingOutput(
    Router* router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void valiantsRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    IntNodeAlg _intNodeAlg, BaseRoutingAlg _routingAlg, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void ugalRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    IntNodeAlg _intNodeAlg, BaseRoutingAlg _routingAlg,
    NonMinRoutingAlg _nonMinimalAlg,
    f64 _bias, Flit* _flit, f64* _weightReg, f64* _weightVal,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputPortsReg,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputPortsVal);

void lcqVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void lcqPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, bool _shortCut,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void doalPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer);

void doalVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer);

void ddalPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer);

void ddalVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer);

void vdalPortRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress, u32 _inputPort, u32 _baseVc,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit, bool _multiDeroute,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer);

void vdalVcRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress, u32 _inputPort, u32 _baseVc,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, Flit* _flit, bool _multiDeroute,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsMin,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcsDer);

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
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void finishingDimOrderRoutingOutput(
    Router* _router, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration,
    const std::vector<u32>* _destinationAddress,
    u32 _baseVc, u32 _vcSet, u32 _numVcSets, u32 _numVcs,
    Flit* _flit, f64 _iBias, f64 _cBias, f64 _threshold,
    SkippingRoutingAlg _routingAlg, DecisionScheme _decisionScheme,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs1,
    std::unordered_set< std::tuple<u32, u32, f64> >* _outputVcs2,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void thresholdDecisionScheme(
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsMin,
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsDer,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, f64 _threshold, f64 _bias,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void thresholdDalDecisionScheme(
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsMin,
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsDer,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, f64 _threshold, f64 _bias,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);

void weightedDecisionScheme(
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsMin,
    const std::unordered_set< std::tuple<u32, u32, f64> >& _outputVcsDer,
    u32 _vcSet, u32 _numVcSets, u32 _numVcs, f64 _hopsLeft, f64 _hopsIncr,
    f64 _iBias, f64 _cBias, bool _conservative,
    std::unordered_set< std::tuple<u32, u32, f64> >* _vcPool);
}  // namespace HyperX

#include "network/hyperx/util.tcc"
#endif  // NETWORK_HYPERX_UTIL_H_
