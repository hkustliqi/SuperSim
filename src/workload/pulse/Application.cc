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
#include "workload/pulse/Application.h"

#include <cassert>

#include <vector>

#include "workload/pulse/PulseTerminal.h"
#include "event/Simulator.h"
#include "network/Network.h"

namespace Pulse {

Application::Application(
    const std::string& _name, const Component* _parent, u32 _id,
    Workload* _workload, MetadataHandler* _metadataHandler,
    Json::Value _settings)
    : ::Application(_name, _parent, _id, _workload, _metadataHandler,
                    _settings) {
  // all terminals are the same
  activeTerminals_ = numTerminals();
  for (u32 t = 0; t < numTerminals(); t++) {
    std::string tname = "PulseTerminal_" + std::to_string(t);
    std::vector<u32> address;
    gSim->getNetwork()->translateInterfaceIdToAddress(t, &address);
    PulseTerminal* terminal = new PulseTerminal(tname, this, t, address, this,
                                                _settings["pulse_terminal"]);
    setTerminal(t, terminal);

    // remove terminals with no injection
    if (terminal->requestInjectionRate() == 0.0) {
      activeTerminals_--;
    }
  }

  // initialize counters
  completedTerminals_ = 0;
  doneTerminals_ = 0;

  // this application is immediately ready
  addEvent(0, 0, nullptr, 0);
}

Application::~Application() {}

f64 Application::percentComplete() const {
  f64 percentSum = 0.0;
  for (u32 idx = 0; idx < numTerminals(); idx++) {
    PulseTerminal* t = reinterpret_cast<PulseTerminal*>(getTerminal(idx));
    percentSum += t->percentComplete();
  }
  return percentSum / activeTerminals_;
}

void Application::start() {
  for (u32 idx = 0; idx < numTerminals(); idx++) {
    PulseTerminal* t = reinterpret_cast<PulseTerminal*>(getTerminal(idx));
    t->start();
  }
}

void Application::stop() {
  // this application is done
  workload_->applicationDone(id_);
}

void Application::kill() {}

void Application::terminalComplete(u32 _id) {
  completedTerminals_++;
  assert(completedTerminals_ <= activeTerminals_);
  if (completedTerminals_ == activeTerminals_) {
    dbgprintf("all terminals are done");
    workload_->applicationComplete(id_);
  }
}

void Application::processEvent(void* _event, s32 _type) {
  dbgprintf("application ready");
  workload_->applicationReady(id_);
}

}  // namespace Pulse
