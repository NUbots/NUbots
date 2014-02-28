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
        
        Controller::Controller(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            
            on<Trigger<RegisterAction>>("Action Registration", [this] (const RegisterAction& action) {
                
                // Register the action in the sorted lists
                
                // Each set gets it's own place in the list
                
            });
            
            on<Trigger<Startup>>("Initial Action Selection", [this] (const Startup&) {
                
                // Pick our first action to take
                selectAction();
                
            });
            
            on<Trigger<ActionPriorites>>("Action Priority Update", [this] (const ActionPriorites& update) {
                
                // Short circuit logic, if the update is for our current action, and the priority went up then don't bother selecting a new action
                
                // Update the priorities of the actions
                
                // Sort the list
                
                // run selectAction()
                
            });
            
            on<Trigger<ServoCommand>>("Command Filter", [this] (const ServoCommand& command) {
                
                // Find out what servos this supplier is authorized to use
                
                // Emit those to the motion manager
                
                // If if we change permissions we need to purge future commands from other things
                
            });
        }
        
        void Controller::selectAction() {
            
            using iterators = std::pair<std::set<RequestItem>::iterator, std::set<RequestItem>::iterator>;
            
            // Set all the running flags on the current actions to false
            
            
            
            
            // Get iterators to each of the actions on the limbs
            std::map<LimbID, iterators> limbs =
            {
                std::make_pair(LimbID::LEFT_LEG  , std::make_pair(std::begin(actions[uint(LimbID::LEFT_LEG)])  , std::end(actions[uint(LimbID::LEFT_LEG)])))
                , std::make_pair(LimbID::RIGHT_LEG , std::make_pair(std::begin(actions[uint(LimbID::RIGHT_LEG)]) , std::end(actions[uint(LimbID::RIGHT_LEG)])))
                , std::make_pair(LimbID::LEFT_ARM  , std::make_pair(std::begin(actions[uint(LimbID::LEFT_ARM)])  , std::end(actions[uint(LimbID::LEFT_ARM)])))
                , std::make_pair(LimbID::RIGHT_ARM , std::make_pair(std::begin(actions[uint(LimbID::RIGHT_ARM)]) , std::end(actions[uint(LimbID::RIGHT_ARM)])))
                , std::make_pair(LimbID::HEAD      , std::make_pair(std::begin(actions[uint(LimbID::HEAD)])      , std::end(actions[uint(LimbID::HEAD)])))
            };
            
            // Our new actions
            std::vector<RequestItem> newActions;
            
            // We keep adding actions while we have limbs to add
            while (!limbs.empty()) {
                
                // Find the largest iterator priority value
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
                                                  return a.second.first->priority < b.second.first->priority;
                                              });
                
                // If we ran out of possible actions for this limb remove it
                if(maxIt->second.first == maxIt->second.second) {
                    limbs.erase(maxIt);
                }
                
                // Otherwise this is a candidate for selection
                else {
                    
                    // These two must pass to be granted the limb control
                    bool mainCheck, hasLimbs = true;
                    
                    // Get the available limbs and the requested limbs
                    auto available = std::begin(limbs);
                    auto request   = std::begin(maxIt->second.first->limbSet);
                    
                    // Check that every limb in request is available
                    while(available != std::end(limbs) && request != std::end(maxIt->second.first->limbSet)) {
                        
                        // If our element is before our requested element check the next limb
                        if(available->first < *request) {
                            ++available;
                        }
                        // If we have the limb requested then move along
                        else if(available->first == *request) {
                            ++available;
                            ++request;
                        }
                        // Otherwise we don't have the needed limb
                        else {
                            hasLimbs = false;
                            break;
                        }
                    }
                    
                    // Are we already active (from previous main activation)
                    // Are we the highest priority action
                    mainCheck = maxIt->second.first->parent.active || maxIt->second.first->mainElement;
                    
                    if(hasLimbs && mainCheck) {
                        
                        // Add this command to our list of commands
                        
                        // Remove the requested limbs from our list of limbs
                        
                        // find each limb from the limbset and remove it
                        
                    }
                    // This request isn't suitable, move to the next one
                    else {
                        ++(maxIt->second.first);
                    }
                }
            }
            
            // Compare our current actions to our new actions
            
            // Find the differences (added and removed)
            
            // Change the control gates on our motors
            
            // Run the kill and start commands for these
            
        }
        
    }  // behaviours
}  // modules
