/*
 * This file is part of Behaviour Controller.
 *
 * Behaviour Controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Behaviour Controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Behaviour Controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Controller.h"

namespace modules {
    namespace behaviour {
        
        using messages::behaviour::RegisterAction;
        using messages::behaviour::ActionPriorites;
        using messages::behaviour::ServoCommand;
        using messages::behaviour::LimbID;
        
        // So we don't need a huge long type declaration everywhere...
        using iterators = std::pair<std::vector<std::reference_wrapper<RequestItem>>::iterator, std::vector<std::reference_wrapper<RequestItem>>::iterator>;
        
        Controller::Controller(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            
            on<Trigger<RegisterAction>>("Action Registration", [this] (const RegisterAction& action) {
                
                // Make our request object
                auto request = std::make_unique<Request>(action.id, action.start, action.kill);
                
                // Make our request items
                for(auto& set : action.limbSet) {
                    RequestItem item(*request, request->items.size(), set.first, set.second);
                    request->items.push_back(std::move(item));
                    
                    // Put our request in the correct queue
                    for(auto& l : set.second) {
                        actions[uint(l)].push_back(std::ref(request->items.back()));
                    }
                }
                
                auto maxRequest = std::max_element(std::begin(request->items), std::end(request->items));
                
                // Set the main element to this one
                request->mainElement = maxRequest->index;
                request->maxPriority = maxRequest->priority;
                
                // Insert this into our requests list
                requests.insert(std::make_pair(request->id, std::move(request)));
            });
            
            on<Trigger<Startup>>("Initial Action Selection", [this] (const Startup&) {
                
                // Pick our first action to take
                selectAction();
                
            });
            
            on<Trigger<ActionPriorites>>("Action Priority Update", [this] (const ActionPriorites& update) {
                
                auto& request = requests[update.id];
                
                // Unless we need to, try not to run the expensive subsumption algorithm
                bool reselect = false;
                
                // Perform our update
                for(uint i = 0; i < request->items.size(); ++i) {
                    
                    bool main = request->mainElement == i;
                    bool higherThenMain = request->maxPriority < update.priorities[i];
                    
                    bool priorityUp = request->items[i].priority < update.priorities[i];
                    bool priorityDown = request->items[i].priority > update.priorities[i];
                    bool active = request->items[i].active;
                    
                    
                    // If this is the main item, we need to do a complete search to see if another priority is higher
                    // Otherwise we just need to check if we are higher then the main priority
                    
                    
                    // TODO reselect if item is active and priority is going down
                    // TODO reselect if item is inactive and priority is going up
                    // TODO no reselect if item is main and is less then highest current priority
                    // TODO no reselect
                    
                    // Update our priority
                    //request->items[i]->priority = update.priorities[i];
                }
                
                // Short circuit logic, if the update is for our current action, and the priority went up then don't bother selecting a new action
                
                // Update the priorities of the actions
                
                if(reselect) {
                    
                    // Select our new action
                    selectAction();
                }
                
            });
            
            on<Trigger<ServoCommand>>("Command Filter", [this] (const ServoCommand& command) {
                
                // Find out what servos this supplier is authorized to use
                
                // Emit those to the motion manager
                
                // If if we change permissions we need to purge future commands from other things
                
            });
        }
        
        bool hasLimbs(const std::set<LimbID>& limbRequest, const std::map<LimbID, iterators>& limbAvailable) {
            
            // Get the available limbs and the requested limbs
            auto available = std::begin(limbAvailable);
            auto request   = std::begin(limbRequest);
            
            // Check that every limb in request is available
            for (; available != std::end(limbAvailable); ++available)
            {
                // If we reach the end of the request, or we don't have the limb return false
                if (request == std::end(limbRequest) || available->first < *request) {
                    return false;
                }
                
                // If our element is after our requested element check the next limb
                if (*request >= available->first) {
                    ++available;
                }
            }
            
            return true;
        }
        
        void Controller::selectAction() {
            
            // Sort each of the lists to choose a new item
            for(auto& l : actions) {
                std::sort(std::begin(l), std::end(l), [] (const RequestItem& a, const RequestItem& b) {
                    return a < b;
                });
            }
            
            // Set the active flags on the current actions to false
            for (auto& action : currentActions) {
                action.get().active = false;
                action.get().group.active = false;
            }
            
            // Get iterators to each of the actions on the limbs
            std::map<LimbID, iterators> limbs =
            {
                std::make_pair(LimbID::LEFT_LEG    , std::make_pair(std::begin(actions[uint(LimbID::LEFT_LEG)])  , std::end(actions[uint(LimbID::LEFT_LEG)])))
                , std::make_pair(LimbID::RIGHT_LEG , std::make_pair(std::begin(actions[uint(LimbID::RIGHT_LEG)]) , std::end(actions[uint(LimbID::RIGHT_LEG)])))
                , std::make_pair(LimbID::LEFT_ARM  , std::make_pair(std::begin(actions[uint(LimbID::LEFT_ARM)])  , std::end(actions[uint(LimbID::LEFT_ARM)])))
                , std::make_pair(LimbID::RIGHT_ARM , std::make_pair(std::begin(actions[uint(LimbID::RIGHT_ARM)]) , std::end(actions[uint(LimbID::RIGHT_ARM)])))
                , std::make_pair(LimbID::HEAD      , std::make_pair(std::begin(actions[uint(LimbID::HEAD)])      , std::end(actions[uint(LimbID::HEAD)])))
            };
            
            // Our new actions
            std::vector<std::reference_wrapper<RequestItem>> newActions;
            
            // We keep adding actions while we have limbs to add
            while (!limbs.empty()) {
                
                // Find the largest iterator task from all the limbs
                auto maxIt = std::max_element(std::begin(limbs), std::end(limbs),
                                              [](const std::pair<LimbID, iterators>& a,
                                                 const std::pair<LimbID, iterators>& b) {
                                                  
                                                  // Empty lists go to the end
                                                  if(a.second.first == a.second.second) {
                                                      return true;
                                                  }
                                                  else if(b.second.first == b.second.second) {
                                                      return false;
                                                  }
                                                  
                                                  // The smaller priority loses
                                                  return a.second.first->get().priority < b.second.first->get().priority;
                                              });
                
                // If we ran out of possible actions for this limb remove it
                if(maxIt->second.first == maxIt->second.second) {
                    limbs.erase(maxIt);
                }
                
                // Otherwise this is a candidate for selection
                else {
                    // This is the iterator to our action
                    auto& action = maxIt->second.first->get();
                    
                    // Are we already active (from previous main activation)
                    // Are we the main action?
                    if(((action.index == action.group.mainElement) || action.group.active)
                       // Do we have the needed limbs
                       && hasLimbs(action.limbSet, limbs)) {
                        
                        // Activate this group and item
                        action.active = true;
                        action.group.active = true;
                        
                        // Push this action onto our list of actions
                        newActions.push_back(std::ref(action));
                        
                        // Remove the limbs that we have just allocated
                        for(auto& limb : action.limbSet) {
                            limbs.erase(limbs.find(limb));
                        }
                        
                    }
                    // This request isn't suitable, move to the next one
                    else {
                        ++(maxIt->second.first);
                    }
                }
            }
            
            // Reset the limb access
            for (auto& l : limbAccess) {
                l = 0;
            }
            
            // Set the permissions for a limb according to our allocations
            for (auto& command : newActions) {
                for(auto& l : command.get().limbSet) {
                    limbAccess[uint(l)] = command.get().group.id;
                }
            }
            
            // This comparator will sort our list so we can compare it with the current actions
            std::function<bool (const RequestItem&, const RequestItem&)> comp =
            [] (const RequestItem& a, const RequestItem& b) {
                if(a.group.id < b.group.id) {
                    return true;
                }
                else if(a.group.id == b.group.id) {
                    return a.index < b.index;
                }
                else {
                    return false;
                }
            };
            
            // Sort our list
            std::sort(std::begin(newActions), std::end(newActions), comp);
            
            // These are used to work out the difference
            std::vector<std::reference_wrapper<RequestItem>> start;
            std::map<size_t, std::set<LimbID>> startMap;
            
            std::vector<std::reference_wrapper<RequestItem>> kill;
            std::map<size_t, std::set<LimbID>> killMap;
            
            // We will never have more then 5
            start.reserve(5);
            kill.reserve(5);

            // Find newly added actions
            std::set_difference(std::begin(newActions), std::end(newActions),
                                std::begin(currentActions), std::end(currentActions),
                                std::back_inserter(start), comp);
            
            // Find now deleted actions
            std::set_difference(std::begin(currentActions), std::end(currentActions),
                                std::begin(newActions), std::end(newActions),
                                std::back_inserter(kill), comp);
            
            // Fill up our map with a list of limbs to kill (and the controllers for it)
            for (const auto& k : kill) {
                for (const auto& l : k.get().limbSet) {
                    killMap[k.get().group.id].insert(l);
                }
            }
            
            // Fill up our map with a list of limbs to start (and the controllers for it)
            for (const auto& s : start) {
                for (const auto& l : s.get().limbSet) {
                    killMap[s.get().group.id].insert(l);
                }
            }
            
            // Execute all our kill commands
            for (const auto& k : killMap) {
                requests[k.first]->kill(k.second);
            }
            
            // Execute our start commands
            for (const auto& s : startMap) {
                requests[s.first]->start(s.second);
            }
            
            // Our actions are now these new actions
            currentActions = std::move(newActions);
        }
        
    }  // behaviours
}  // modules
