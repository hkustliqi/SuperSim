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
#include "network/hierarchicalhyperx/util.h"

#include <gtest/gtest.h>
#include <json/json.h>
#include <prim/prim.h>
#include <strop/strop.h>

#include <unordered_set>
#include <vector>
#include <iostream>

#include "event/Component.h"
#include "network/cube/util.h"
#include "router/Router.h"
#include "test/TestSetup_TEST.h"
#include "types/Packet.h"

struct RoutingInfo {
  std::vector<u32>* intermediateAddress;
  std::vector<u32>* localDst;
  std::vector<u32>* localDstPort;
  u32 localDerouteCount;
  u32 globalHopCount;
  bool intermediateDone;
  bool valiantMode;
};

Json::Value makeJSON(u32 _numPorts, u32 _numVcs) {
  Json::Value settings;
  settings["num_ports"] = _numPorts;
  settings["num_vcs"] = _numVcs;
  return settings;
}

class TestRouter : public Router {
 public:
  TestRouter(const std::vector<u32>& _address, u32 _id, u32 _numPorts,
             u32 _numVcs, const std::unordered_map<u32, f64>& _congStatus)
      : Router("TestRouter " + strop::vecString<u32>(_address), nullptr,
               _id, _address, _numPorts, _numVcs, nullptr,
               makeJSON(_numPorts, _numVcs)) {
    congStatus_ = _congStatus;
  }
  ~TestRouter() {}
  f64 congestionStatus(u32 _port, u32 _vc) const override {
    u32 vcIdx = vcIndex(_port, _vc);
    std::unordered_map<u32, f64>::const_iterator got = congStatus_.find(vcIdx);
    if (got == congStatus_.end()) {
      return 1.0;
    }
    return congStatus_.at(vcIdx);
  }

 private:
  std::unordered_map<u32, f64> congStatus_;
};




TEST(HierarchicalHyperXUtil, globalPortToLocalAddress) {
  u32 globalPort;
  u32 localPort;
  std::vector<u32> localAdd = {0, 0};
  std::vector<u32> localDimWidth = {2, 2};

  globalPort = 0;
  std::vector<u32> exp = {0, 0};
  u32 localPortResult = 0;
  HierarchicalHyperX::globalPortToLocalAddress(globalPort, &localAdd,
                                               &localPort, localDimWidth);
  ASSERT_EQ(localAdd, exp);
  ASSERT_EQ(localPort, localPortResult);

  globalPort = 1;
  exp = {1, 0};
  localPortResult = 0;
  HierarchicalHyperX::globalPortToLocalAddress(globalPort, &localAdd,
                                               &localPort, localDimWidth);
  ASSERT_EQ(localAdd, exp);
  ASSERT_EQ(localPort, localPortResult);

  globalPort = 2;
  exp = {0, 1};
  localPortResult = 0;
  HierarchicalHyperX::globalPortToLocalAddress(globalPort, &localAdd,
                                               &localPort, localDimWidth);
  ASSERT_EQ(localAdd, exp);
  ASSERT_EQ(localPort, localPortResult);

  globalPort = 3;
  exp = {1, 1};
  localPortResult = 0;
  HierarchicalHyperX::globalPortToLocalAddress(globalPort, &localAdd,
                                               &localPort, localDimWidth);
  ASSERT_EQ(localAdd, exp);
  ASSERT_EQ(localPort, localPortResult);

  globalPort = 4;
  exp = {0, 0};
  localPortResult = 1;
  HierarchicalHyperX::globalPortToLocalAddress(globalPort, &localAdd,
                                               &localPort, localDimWidth);
  ASSERT_EQ(localAdd, exp);
  ASSERT_EQ(localPort, localPortResult);
}

TEST(HierarchicalHyperXUtil, getPortBase) {
  u32 portBase;
  u32 concentration;
  std::vector<u32> localDimWidth;
  std::vector<u32> localDimWeight;

  concentration = 3;
  localDimWidth = {2, 3};
  localDimWeight = {2, 1};
  portBase = 7;
  ASSERT_EQ(HierarchicalHyperX::getPortBase(concentration, localDimWidth,
                                            localDimWeight), portBase);

  concentration = 2;
  localDimWidth = {3, 2};
  localDimWeight = {2, 1};
  portBase = 7;
  ASSERT_EQ(HierarchicalHyperX::getPortBase(concentration, localDimWidth,
                                            localDimWeight), portBase);
}


TEST(HierarchicalHyperXUtil, test) {
  TestSetup(1, 1, 123);
  std::vector<u32> test = {0, 1};
  printf("bef0re\n");
  u32 num = test.at(gSim->rnd.nextU64(0, test.size() - 1));
  printf("pos = %u\n", num);
}
/*
TEST(HierarchicalHyperXUtil, setLocalDst) {
  std::vector<u32> diffGlobalDims;
  std::vector<u32> dstAdd;
  std::vector<u32> globalOutputPorts;
  std::vector<u32> routerAdd;
  std::vector<u32> localDimWidth;
  std::vector<u32> globalDimWidth;
  std::vector<u32> globalDimWeight;
  void* data = new int(2);
  Message* message = new Message(1, data);
  Packet* packet = new Packet(1, 1, message);
  std::vector<u32> localDst;
  std::vector<u32> localDstPort;
  RoutingInfo* ri = new RoutingInfo();
  ri->intermediateAddress = nullptr;
  ri->localDst = nullptr;
  ri->localDstPort = nullptr;
  ri->localDerouteCount = 0;
  ri->globalHopCount = 0;
  ri->intermediateDone = false;
  ri->valiantMode = false;
  packet->setRoutingExtension(ri);

  diffGlobalDims = {0};
  dstAdd = {0, 0, 0, 1, 0};
  routerAdd = {0, 0, 0, 0};
  localDimWidth = {3, 2};
  globalDimWidth = {6, 2};
  globalDimWeight = {1, 1};
  localDst = {0, 0};
  localDstPort = {0};
  HierarchicalHyperX::setLocalDst(diffGlobalDims, dstAdd, &globalOutputPorts,
                                  packet, routerAdd, localDimWidth,
                                  globalDimWidth, globalDimWeight);
  RoutingInfo* result = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());
  ASSERT_EQ(*(result->localDst), localDst);
  ASSERT_EQ(*(result->localDstPort), localDstPort);

  globalOutputPorts.clear();
  delete reinterpret_cast<const std::vector<u32>*>(result->localDst);
  result->localDst = nullptr;
  delete reinterpret_cast<const std::vector<u32>*>(result->localDstPort);
  result->localDstPort = nullptr;
  diffGlobalDims = {0};
  dstAdd = {0, 0, 0, 2, 0};
  routerAdd = {0, 0, 0, 0};
  localDimWidth = {3, 2};
  globalDimWidth = {6, 2};
  globalDimWeight = {1, 1};
  localDst = {1, 0};
  localDstPort = {0};
  HierarchicalHyperX::setLocalDst(diffGlobalDims, dstAdd, &globalOutputPorts,
                                  packet, routerAdd, localDimWidth,
                                  globalDimWidth, globalDimWeight);
  result = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());
  ASSERT_EQ(*(result->localDst), localDst);
  ASSERT_EQ(*(result->localDstPort), localDstPort);

  globalOutputPorts.clear();
  delete reinterpret_cast<const std::vector<u32>*>(result->localDst);
  result->localDst = nullptr;
  delete reinterpret_cast<const std::vector<u32>*>(result->localDstPort);
  result->localDstPort = nullptr;
  diffGlobalDims = {1};
  dstAdd = {0, 0, 0, 0, 1};
  routerAdd = {0, 0, 0, 0};
  localDimWidth = {3, 2};
  globalDimWidth = {6, 2};
  globalDimWeight = {1, 1};
  localDst = {2, 1};
  localDstPort = {0};
  HierarchicalHyperX::setLocalDst(diffGlobalDims, dstAdd, &globalOutputPorts,
                                  packet, routerAdd, localDimWidth,
                                  globalDimWidth, globalDimWeight);
  result = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());
  ASSERT_EQ(*(result->localDst), localDst);
  ASSERT_EQ(*(result->localDstPort), localDstPort);

  printf("\n\n start \n");
  globalOutputPorts.clear();
  delete reinterpret_cast<const std::vector<u32>*>(result->localDst);
  result->localDst = nullptr;
  delete reinterpret_cast<const std::vector<u32>*>(result->localDstPort);
  result->localDstPort = nullptr;
  diffGlobalDims = {0, 1};
  dstAdd = {0, 0, 0, 2, 1};
  routerAdd = {0, 0, 0, 0};
  localDimWidth = {3, 2};
  globalDimWidth = {6, 2};
  globalDimWeight = {1, 1};
  localDst = {2, 1};
  localDstPort = {0};
  printf("diffDimSize = %lu \n", diffGlobalDims.size());
  u32 pos = gSim->rnd.nextU64(0, diffGlobalDims.size() - 1);
  printf("pos = %u\n", pos);
  HierarchicalHyperX::setLocalDst(diffGlobalDims, dstAdd, &globalOutputPorts,
                                  packet, routerAdd, localDimWidth,
                                  globalDimWidth, globalDimWeight);
  result = reinterpret_cast<RoutingInfo*>(
      packet->getRoutingExtension());
  ASSERT_EQ(*(result->localDst), localDst);
  ASSERT_EQ(*(result->localDstPort), localDstPort);
  }*/
