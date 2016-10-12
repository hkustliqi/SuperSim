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
#ifndef NETWORK_HIERARCHICALHYPERX_NETWORK_H_
#define NETWORK_HIERARCHICALHYPERX_NETWORK_H_

#include <json/json.h>
#include <prim/prim.h>

#include <string>
#include <vector>

#include "event/Component.h"
#include "interface/Interface.h"
#include "network/Network.h"
#include "network/Channel.h"
#include "router/Router.h"
#include "util/DimensionalArray.h"

namespace HierarchicalHyperX {

class Network : public ::Network {
 public:
  Network(const std::string& _name, const Component* _parent,
          MetadataHandler* _metadataHandler, Json::Value _settings);
  ~Network();

  // Network
  u32 numRouters() const override;
  u32 numInterfaces() const override;
  Router* getRouter(u32 _id) const override;
  Interface* getInterface(u32 _id) const override;
  void translateTerminalIdToAddress(u32 _id,
                                    std::vector<u32>* _address) const override;
  u32 translateTerminalAddressToId(
      const std::vector<u32>* _address) const override;
  void translateRouterIdToAddress(
      u32 _id, std::vector<u32>* _address) const override;
  u32 translateRouterAddressToId(
      const std::vector<u32>* _address) const override;

 protected:
  void collectChannels(std::vector<Channel*>* _channels) override;

 private:
  // global hyperX parameters
  u32 globalDimensions_;
  std::vector<u32> globalDimensionWidths_;
  std::vector<u32> globalDimensionWeights_;
  // local hyperX parameters
  u32 localDimensions_;
  std::vector<u32> localDimensionWidths_;
  std::vector<u32> localDimensionWeights_;
  u32 concentration_;
  // user specifed number of global links per router
  u32 globalLinksPerRouter_;

  DimensionalArray<Router*> routers_;
  DimensionalArray<Interface*> interfaces_;
  std::vector<Channel*> localChannels_;
  std::vector<Channel*> globalChannels_;
  std::vector<Channel*> externalChannels_;
};

}  // namespace HierarchicalHyperX

#endif  // NETWORK_HIERARCHICALHYPERX_NETWORK_H_
