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
#include "network/hierarchicalhyperx/Network.h"

#include <strop/strop.h>

#include <cassert>
#include <cmath>

#include <tuple>

#include "interface/InterfaceFactory.h"
#include "network/hierarchicalhyperx/RoutingAlgorithmFactory.h"
#include "router/RouterFactory.h"
#include "util/DimensionIterator.h"

namespace HierarchicalHyperX {

Network::Network(const std::string& _name, const Component* _parent,
                 MetadataHandler* _metadataHandler, Json::Value _settings)
    : ::Network(_name, _parent, _metadataHandler, _settings) {
  // global dimension
  assert(_settings["global_dimension_widths"].isArray());
  globalDimensions_ = _settings["global_dimension_widths"].size();
  assert(_settings["global_dimension_weights"].size() == globalDimensions_);
  globalDimensionWidths_.resize(globalDimensions_);
  for (u32 i = 0; i < globalDimensions_; i++) {
    globalDimensionWidths_.at(i) =
        _settings["global_dimension_widths"][i].asUInt();
    assert(globalDimensionWidths_.at(i) >= 2);
  }
  globalDimensionWeights_.resize(globalDimensions_);
  for (u32 i = 0; i < globalDimensions_; i++) {
    globalDimensionWeights_.at(i) =
        _settings["global_dimension_weights"][i].asUInt();
    assert(globalDimensionWeights_.at(i) >= 1);
  }
  // local dimension
  assert(_settings["local_dimension_widths"].isArray());
  localDimensions_ = _settings["local_dimension_widths"].size();
  assert(_settings["local_dimension_weights"].size() == localDimensions_);
  localDimensionWidths_.resize(localDimensions_);
  for (u32 i = 0; i < localDimensions_; i++) {
    localDimensionWidths_.at(i) =
        _settings["local_dimension_widths"][i].asUInt();
    assert(localDimensionWidths_.at(i) >= 2);
  }
  localDimensionWeights_.resize(localDimensions_);
  for (u32 i = 0; i < localDimensions_; i++) {
    localDimensionWeights_.at(i) =
        _settings["local_dimension_weights"][i].asUInt();
    assert(localDimensionWeights_.at(i) >= 1);
  }
  concentration_ = _settings["concentration"].asUInt();
  assert(concentration_ > 0);

  // router radix
  assert(_settings["router"].isMember("num_ports") == false);
  assert(_settings["router"].isMember("num_vcs") == false);
  assert(_settings["interface"].isMember("num_vcs") == false);
  u32 routerRadix = concentration_;
  for (u32 i = 0; i < localDimensions_; i++) {
    routerRadix += ((localDimensionWidths_.at(i) - 1) *
                    localDimensionWeights_.at(i));
  }
  globalLinksPerRouter_ = _settings["global_links_per_router"].asUInt();
  assert(globalLinksPerRouter_ > 0);
  routerRadix += globalLinksPerRouter_;
  _settings["router"]["num_ports"] = Json::Value(routerRadix);
  _settings["router"]["concentration"] = Json::Value(concentration_);
  _settings["router"]["num_vcs"] = Json::Value(numVcs_);
  _settings["interface"]["num_vcs"] = Json::Value(numVcs_);

  // parse the traffic classes description
  std::vector<std::tuple<u32, u32> > trafficClassVcs;
  std::vector<::RoutingAlgorithmFactory*> routingAlgorithmFactories;
  for (u32 idx = 0; idx < _settings["traffic_classes"].size(); idx++) {
    u32 numVcs = _settings["traffic_classes"][idx]["num_vcs"].asUInt();
    u32 baseVc = routingAlgorithmFactories.size();
    trafficClassVcs.push_back(std::make_tuple(baseVc, numVcs));
    for (u32 vc = 0; vc < numVcs; vc++) {
      routingAlgorithmFactories.push_back(
          new RoutingAlgorithmFactory(
              baseVc, numVcs, globalDimensionWidths_, globalDimensionWeights_,
              localDimensionWidths_, localDimensionWeights_,
              globalLinksPerRouter_, concentration_,
              _settings["traffic_classes"][idx]["routing"]));
    }
  }

  // create a vector of local and global dimension widths
  std::vector<u32> allDimensionWidths(localDimensionWidths_);
  allDimensionWidths.insert(allDimensionWidths.end(),
                            globalDimensionWidths_.cbegin(),
                            globalDimensionWidths_.cend());

  // setup a router iterator for looping over the router dimensions
  DimensionIterator routerIterator(allDimensionWidths);
  std::vector<u32> routerAddress(allDimensionWidths.size());

  // create the routers (loop over allDimensionWidths)
  routerIterator.reset();
  routers_.setSize(allDimensionWidths);
  while (routerIterator.next(&routerAddress)) {
    std::string routerName = "Router_" + strop::vecString<u32>(routerAddress);

    // use the router factory to create a router
    u32 routerId = translateRouterAddressToId(&routerAddress);
    routers_.at(routerAddress) = RouterFactory::createRouter(
        routerName, this, routerId, routerAddress, routerRadix, numVcs_,
        _metadataHandler, &routingAlgorithmFactories, _settings["router"]);
    // set the router's address
    // routers_.at(routerAddress)->setAddress(routerAddress);
  }

  // we don't need the routing algorithm factories anymore
  for (::RoutingAlgorithmFactory* raf : routingAlgorithmFactories) {
    delete raf;
  }
  routingAlgorithmFactories.clear();

  // link routers via local channels (loop over localDimensionWidths_)
  routerIterator.reset();
  while (routerIterator.next(&routerAddress)) {
    u32 portBase = concentration_;
    for (u32 localDim = 0; localDim < localDimensions_; localDim++) {
      u32 localDimWidth = localDimensionWidths_.at(localDim);
      u32 localDimWeight = localDimensionWeights_.at(localDim);
      dbgprintf("localDim=%u width=%u weight=%u\n",
                localDim, localDimWidth, localDimWeight);

      for (u32 offset = 1; offset < localDimWidth; offset++) {
        // determine the source router
        std::vector<u32> sourceAddress(routerAddress);

        // determine the destination router
        std::vector<u32> destinationAddress(sourceAddress);
        destinationAddress.at(localDim) = (sourceAddress.at(localDim)
                                           + offset) % localDimWidth;

        for (u32 weight = 0; weight < localDimWeight; weight++) {
          // create the channel
          std::string channelName = "LocalChannel_" +
              strop::vecString<u32>(routerAddress) + "-to-" +
              strop::vecString<u32>(destinationAddress) +
              "-" + std::to_string(weight);
          Channel* channel = new Channel(channelName, this,
                                         _settings["internal_local_channel"]);
          localChannels_.push_back(channel);

          // determine the port numbers
          u32 sourcePort = portBase + ((offset - 1) * localDimWeight) + weight;
          u32 destinationPort = portBase + ((localDimWidth - 1 - offset) *
                                            localDimWeight) + weight;

          // link the routers from source to destination
          routers_.at(sourceAddress)->setOutputChannel(sourcePort, channel);
          routers_.at(destinationAddress)->setInputChannel(destinationPort,
                                                           channel);
        }
      }
      portBase += ((localDimWidth - 1) * localDimWeight);
    }
  }

  // link global channels, global router is Virtual
  DimensionIterator globalRouterIterator(globalDimensionWidths_);
  std::vector<u32> globalRouterAddress(globalDimensionWidths_.size());
  globalRouterIterator.reset();
  u32 numRoutersPerGlobalRouter = 1;
  for (u32 tmp = 0; tmp < localDimensions_; tmp++) {
    numRoutersPerGlobalRouter *= localDimensionWidths_.at(tmp);
  }
  while (globalRouterIterator.next(&globalRouterAddress)) {
    u32 virtualGlobalPortBase = 0;
    for (u32 globalDim = 0; globalDim < globalDimensions_; globalDim++) {
      u32 globalDimWidth = globalDimensionWidths_.at(globalDim);
      u32 globalDimWeight = globalDimensionWeights_.at(globalDim);

      for (u32 offset = 1; offset < globalDimWidth; offset++) {
        // global source router
        std::vector<u32> srcGlobalAddress(globalRouterAddress);
        // determine global destination router
        std::vector<u32> dstGlobalAddress(srcGlobalAddress);
        dstGlobalAddress.at(globalDim) = (srcGlobalAddress.at(globalDim)
                                          + offset) % globalDimWidth;

        for (u32 weight = 0; weight < globalDimWeight; weight++) {
          // determine the vitual port of global router
          u32 virtualGlobalSrcPort = virtualGlobalPortBase
              + ((offset - 1) * globalDimWeight)
              + weight;
          u32 virtualGlobalDstPort = virtualGlobalPortBase
              + ((globalDimWidth - 1 - offset) *
                 globalDimWeight) + weight;

          // translate the virtual port to the actual router
          std::vector<u32> srcLocalAddress(localDimensions_);
          u32 product = 1;
          for (u32 tmp = 0; tmp < localDimensions_ - 1; tmp++) {
            product *= localDimensionWidths_.at(tmp);
          }
          u32 srcPortCopy = virtualGlobalSrcPort;
          for (s32 localDim = localDimensions_ - 1; localDim >= 0; localDim--) {
            srcLocalAddress.at(localDim) = (srcPortCopy / product)
                % localDimensionWidths_.at(localDim);
            srcPortCopy %= product;
            if (localDim != 0) {
              product /= localDimensionWidths_.at(localDim - 1);
            }
          }
          assert(srcLocalAddress.size() == localDimensions_);
          u32 srcPort = virtualGlobalSrcPort / numRoutersPerGlobalRouter;
          assert(srcPort < globalLinksPerRouter_);

          std::vector<u32> dstLocalAddress(localDimensions_);
          product = 1;
          for (u32 tmp = 0; tmp < localDimensions_ - 1; tmp++) {
            product *= localDimensionWidths_.at(tmp);
          }
          u32 dstPortCopy = virtualGlobalDstPort;
          for (s32 localDim = localDimensions_ - 1; localDim >= 0; localDim--) {
            dstLocalAddress.at(localDim) = (dstPortCopy / product)
                % localDimensionWidths_.at(localDim);
            dstPortCopy %= product;
            if (localDim != 0) {
              product /= localDimensionWidths_.at(localDim - 1);
            }
          }
          assert(dstLocalAddress.size() == localDimensions_);
          u32 dstPort = virtualGlobalDstPort / numRoutersPerGlobalRouter;
          assert(dstPort < globalLinksPerRouter_);

          // src and dst router address
          std::vector<u32> srcAddress(srcLocalAddress);
          srcAddress.insert(srcAddress.end(), srcGlobalAddress.cbegin(),
                            srcGlobalAddress.cend());
          assert(srcAddress.size() == localDimensions_ + globalDimensions_);
          std::vector<u32> dstAddress(dstLocalAddress);
          dstAddress.insert(dstAddress.end(), dstGlobalAddress.cbegin(),
                            dstGlobalAddress.cend());
          assert(dstAddress.size() == localDimensions_ + globalDimensions_);

          // create the channel
          std::string channelName = "GlobalChannel_" +
              strop::vecString<u32>(srcAddress) + "-to-" +
              strop::vecString<u32>(dstAddress) +
              "-" + std::to_string(weight);
          Channel* channel = new Channel(channelName, this,
                                         _settings["internal_global_channel"]);
          globalChannels_.push_back(channel);

          srcPort += routerRadix - globalLinksPerRouter_;
          dstPort += routerRadix - globalLinksPerRouter_;

          // link routers from source to destination
          routers_.at(srcAddress)->setOutputChannel(srcPort, channel);
          routers_.at(dstAddress)->setInputChannel(dstPort, channel);
        }
      }
      virtualGlobalPortBase += ((globalDimWidth - 1) * globalDimWeight);
    }
  }

  // create a vector of dimension widths that contains the concentration
  std::vector<u32> fullDimensionWidths(1);
  fullDimensionWidths.at(0) = concentration_;
  fullDimensionWidths.insert(fullDimensionWidths.begin() + 1,
                             allDimensionWidths.cbegin(),
                             allDimensionWidths.cend());

  // create interfaces and link them with the routers
  interfaces_.setSize(fullDimensionWidths);
  // u32 interfaceId = 0;
  routerIterator.reset();
  while (routerIterator.next(&routerAddress)) {
    // get the router now, for later linking with terminals
    Router* router = routers_.at(routerAddress);

    // loop over concentration
    for (u32 conc = 0; conc < concentration_; conc++) {
      // create a vector for the Interface address
      std::vector<u32> interfaceAddress(1);
      interfaceAddress.at(0) = conc;
      interfaceAddress.insert(interfaceAddress.begin() + 1,
                              routerAddress.cbegin(), routerAddress.cend());

      // create an interface name
      std::string interfaceName = "Interface_" +
          strop::vecString<u32>(interfaceAddress);

      // create the interface
      u32 interfaceId = translateInterfaceAddressToId(&interfaceAddress);
      Interface* interface = InterfaceFactory::createInterface(
          interfaceName, this, interfaceId, interfaceAddress, numVcs_,
          trafficClassVcs, _settings["interface"]);
      interfaces_.at(interfaceAddress) = interface;
      // interfaceId++;

      // create I/O channels
      std::string inChannelName = "Channel_" +
          strop::vecString<u32>(interfaceAddress) + "-to-" +
          strop::vecString<u32>(routerAddress);
      std::string outChannelName = "Channel_" +
          strop::vecString<u32>(routerAddress) + "-to-" +
          strop::vecString<u32>(interfaceAddress);
      Channel* inChannel = new Channel(inChannelName, this,
                                       _settings["external_channel"]);
      Channel* outChannel = new Channel(outChannelName, this,
                                        _settings["external_channel"]);
      externalChannels_.push_back(inChannel);
      externalChannels_.push_back(outChannel);

      // link with router
      router->setInputChannel(conc, inChannel);
      interface->setOutputChannel(0, inChannel);
      router->setOutputChannel(conc, outChannel);
      interface->setInputChannel(0, outChannel);
    }
  }
}

Network::~Network() {
  // delete routers
  for (u32 id = 0; id < routers_.size(); id++) {
    delete routers_.at(id);
  }

  // delete interfaces
  for (u32 id = 0; id < interfaces_.size(); id++) {
    delete interfaces_.at(id);
  }

  // delete channels
  for (auto it = localChannels_.begin();
       it != localChannels_.end(); ++it) {
    delete *it;
  }
  for (auto it = globalChannels_.begin();
       it != globalChannels_.end(); ++it) {
    delete *it;
  }
  for (auto it = externalChannels_.begin();
       it != externalChannels_.end(); ++it) {
    delete *it;
  }
}

u32 Network::numRouters() const {
  return routers_.size();
}

u32 Network::numInterfaces() const {
  return interfaces_.size();
}

Router* Network::getRouter(u32 _id) const {
  return routers_.at(_id);
}

Interface* Network::getInterface(u32 _id) const {
  return interfaces_.at(_id);
}

void Network::translateInterfaceIdToAddress(u32 _id,
                                           std::vector<u32>* _address) const {
  _address->resize(localDimensions_ + globalDimensions_ + 1);
  // addresses are in little endian format
  u32 mod, div;
  mod = _id % concentration_;
  div = _id / concentration_;
  _address->at(0) = mod;
  for (u32 localDim = 0; localDim < localDimensions_; localDim++) {
    u32 localDimWidth = localDimensionWidths_.at(localDim);
    mod = div % localDimWidth;
    div = div / localDimWidth;
    _address->at(localDim + 1) = mod;
  }
  for (u32 globalDim = 0; globalDim < globalDimensions_; globalDim++) {
    u32 globalDimWidth = globalDimensionWidths_.at(globalDim);
    mod = div % globalDimWidth;
    div = div / globalDimWidth;
    _address->at(globalDim + localDimensions_ + 1) = mod;
  }
}

u32 Network::translateInterfaceAddressToId(
    const std::vector<u32>* _address) const {
  std::vector<u32> coeff(localDimensions_ + globalDimensions_ + 1);

  // compute coefficients
  u32 prod = 1;
  for (u32 idx = 0; idx < localDimensions_ + globalDimensions_ + 1; idx++) {
    coeff.at(idx) = prod;
    if (idx == 0) {
      assert(_address->at(idx) < concentration_);
      prod *= concentration_;
    } else if (idx < localDimensions_ + 1) {
      assert(_address->at(idx) < localDimensionWidths_.at(idx - 1));
      prod *= localDimensionWidths_.at(idx - 1);
    } else {
      assert(_address->at(idx) <
             globalDimensionWidths_.at(idx - localDimensions_ - 1));
      prod *= globalDimensionWidths_.at(idx - localDimensions_ - 1);
    }
  }

  // compute dot product
  u32 sum = 0;
  for (u32 idx = 0; idx < localDimensions_ + globalDimensions_ + 1; idx++) {
    sum += coeff.at(idx) * _address->at(idx);
  }

  return sum;
}

void Network::translateRouterIdToAddress(
    u32 _id, std::vector<u32>* _address) const {
  _address->resize(localDimensions_ + globalDimensions_);

  // addresses are in little endian format
  u32 mod, div;
  div = _id;
  for (u32 localDim = 0; localDim < localDimensions_; localDim++) {
    u32 localDimWidth = localDimensionWidths_.at(localDim);
    mod = div % localDimWidth;
    div = div / localDimWidth;
    _address->at(localDim) = mod;
  }
  for (u32 globalDim = 0; globalDim < globalDimensions_; globalDim++) {
    u32 globalDimWidth = globalDimensionWidths_.at(globalDim);
    mod = div % globalDimWidth;
    div = div / globalDimWidth;
    _address->at(globalDim + localDimensions_) = mod;
  }
}

u32 Network::translateRouterAddressToId(
    const std::vector<u32>* _address) const {
  std::vector<u32> coeff(localDimensions_ + globalDimensions_);

  // compute coefficients
  u32 prod = 1;
  for (u32 idx = 0; idx < localDimensions_ + globalDimensions_; idx++) {
    coeff.at(idx) = prod;
    if (idx < localDimensions_) {
      assert(_address->at(idx) < localDimensionWidths_.at(idx));
      prod *= localDimensionWidths_.at(idx);
    } else {
      assert(_address->at(idx) <
             globalDimensionWidths_.at(idx - localDimensions_));
      prod *= globalDimensionWidths_.at(idx - localDimensions_);
    }
  }

  // compute dot product
  u32 sum = 0;
  for (u32 idx = 0; idx < localDimensions_ + globalDimensions_; idx++) {
    sum += coeff.at(idx) * _address->at(idx);
  }

  return sum;
}

void Network::collectChannels(std::vector<Channel*>* _channels) {
  for (auto it = externalChannels_.begin();
       it != externalChannels_.end(); ++it) {
    _channels->push_back(*it);
  }
  for (auto it = localChannels_.begin();
       it != localChannels_.end(); ++it) {
    _channels->push_back(*it);
  }
  for (auto it = globalChannels_.begin();
       it != globalChannels_.end(); ++it) {
    _channels->push_back(*it);
  }
}

}  // namespace HierarchicalHyperX
