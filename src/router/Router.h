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
#ifndef ROUTER_ROUTER_H_
#define ROUTER_ROUTER_H_

#include <json/json.h>
#include <prim/prim.h>

#include <string>
#include <vector>

#include "architecture/PortedDevice.h"
#include "event/Component.h"
#include "metadata/MetadataHandler.h"
#include "network/Channel.h"
#include "types/CreditReceiver.h"
#include "types/CreditSender.h"
#include "types/FlitReceiver.h"
#include "types/FlitSender.h"

class Router : public Component, public PortedDevice, public FlitSender,
               public FlitReceiver, public CreditSender, public CreditReceiver {
 public:
  Router(const std::string& _name, const Component* _parent, u32 _id,
         const std::vector<u32>& _address, u32 _numPorts, u32 _numVcs,
         MetadataHandler* _metadataHandler, Json::Value _settings);
  virtual ~Router();

  // this must be called by all subclasses when a packet's head flit arrives
  //  on an input port.
  void packetArrival(Packet* _packet) const;

  virtual f64 congestionStatus(u32 _port, u32 _vc) const = 0;

 protected:
  MetadataHandler* metadataHandler_;
};

#endif  // ROUTER_ROUTER_H_
