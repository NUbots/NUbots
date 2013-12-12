/*
 * This file is part of LUTClassifier.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "LUTClassifier.h"

namespace modules {
    namespace vision {

		using messages::input::Image;
		using messages::vision::ColourSegment;
		using messages::support::Configuration;
		using utility::configuration::ConfigurationNode;
        
        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), greenHorizon(), scanLines() { 
			current_LUT_index = 0;
			
            on<Trigger<Configuration<VisionConstants>>>([this](const Configuration<VisionConstants>& constants) {
           		
            });

			//Load LUTs
			on<Trigger<Configuration<LUTLocations>>>([this](const Configuration<LUTLocations>& locations) {
				std::vector<std::string> locat = locations.config;

				for(auto location : locat) {
					LookUpTable LUT;
					bool loaded = LUT.loadLUTFromFile(location);

					if(loaded) {
						LUTs.push_back(LUT);
					}

					else {
						log<NUClear::ERROR>("LUT ", location, " has not loaded successfully." );
					}
				}
			});

			//Load in greenhorizon parameters
			on<Trigger<Configuration<GreenHorizonConfig>>>([this](const Configuration<GreenHorizonConfig>& constants) {
				greenHorizon.setParameters(constants.config["GREEN_HORIZON_SCAN_SPACING"],
											constants.config["GREEN_HORIZON_MIN_GREEN_PIXELS"],
											constants.config["GREEN_HORIZON_UPPER_THRESHOLD_MULT"]);
			});

			//Load in scanline parameters
			on<Trigger<Configuration<ScanLinesConfig>>>([this](const Configuration<ScanLinesConfig>& constants) {
				scanLines.setParameters(constants.config["HORIZONTAL_SCANLINE_SPACING"],
										 constants.config["VERTICAL_SCANLINE_SPACING"]);
			});
			

			on<Trigger<Configuration<RulesConfig>>>([this](const Configuration<RulesConfig>& rules) {
				segmentFilter.clearRules();
				// std::vector< WHAT?!?!?! > rules = rules.config["REPLACEMENT_RULES"];
				std::map<std::string, ConfigurationNode> replacement_rules = rules.config["REPLACEMENT_RULES"];
				std::map<std::string, ConfigurationNode> transition_rules = rules.config["TRANSITION_RULES"];

				for(auto rule : replacement_rules) {
					std::cout << "Loading Replacement rule : " << rule.first << std::endl;

					
					ColourReplacementRule r;

					vector<unint_32> before = rule.second["before"]["vec"];
					vector<unint_32> middle = rule.second["middle"]["vec"];
					vector<unint_32> after = rule.second["after"]["vec"];

					r.loadRuleFromConfigInfo(rule.second["before"]["colour"],
											rule.second["middle"]["colour"],
											rule.second["after"]["colour"],
											before[0],//min
											before[1],//max, etc.
											middle[0],
											middle[1],
											after[0],
											after[1],
											rule.second["replacement"]);

					//Neat method which is broken due to config system
					/*r.loadRuleFromConfigInfo(rule.second["before"]["colour"],
											rule.second["middle"]["colour"],
											rule.second["after"]["colour"],
											unint_32(rule.second["before"]["vec"][0]),//min
											unint_32(rule.second["before"]["vec"][1]),//max, etc.
											unint_32(rule.second["middle"]["vec"][0]),
											unint_32(rule.second["middle"]["vec"][1]),
											unint_32(rule.second["after"]["vec"][0]),
											unint_32(rule.second["after"]["vec"][1]),
											rule.second["replacement"]);*/
					segmentFilter.addReplacementRule(r);
				}

				for(auto rule : transition_rules) {
					std::cout << "Loading Transition rule : " << rule.first << std::endl;

					
					ColourTransitionRule r;

					vector<unint_32> before = rule.second["before"]["vec"];
					vector<unint_32> middle = rule.second["middle"]["vec"];
					vector<unint_32> after = rule.second["after"]["vec"];

					r.loadRuleFromConfigInfo(rule.second["before"]["colour"],
											rule.second["middle"]["colour"],
											rule.second["after"]["colour"],
											before[0],//min
											before[1],//max, etc.
											middle[0],
											middle[1],
											after[0],
											after[1]);
					//Neat method which is broken due to config system
					/*r.loadRuleFromConfigInfo(rule.second["before"]["colour"],
											rule.second["middle"]["colour"],
											rule.second["after"]["colour"],
											static_cast<uint_32>(rule.second["before"]["vec"][0]),
											static_cast<uint_32>(rule.second["before"]["vec"][1]),
											static_cast<uint_32>(rule.second["middle"]["vec"][0]),
											static_cast<uint_32>(rule.second["middle"]["vec"][1]),
											static_cast<uint_32>(rule.second["after"]["vec"][0]),
											static_cast<uint_32>(rule.second["after"]["vec"][1]));											
											unint_32(rule.second["after"]["vec"][1]));*/
					segmentFilter.addTransitionRule(r);
				}
			});

            on<Trigger<Image>>([this](const Image& image) {
            	/*std::vector<arma::vec2> green_horizon_points = */
            	greenHorizon.calculateGreenHorizon(image, LUTs[current_LUT_index]);
            	std::vector<int> scan_lines = scanLines.generateScanLines(image, greenHorizon);
            	std::vector<std::vector<ColourSegment>> classified_segments_hor = scanLines.classifyHorizontalScanLines(image, scan_lines, LUTs[current_LUT_index]);
            	std::vector<std::vector<ColourSegment>> classified_segments_ver = scanLines.classifyVerticalScanLines(image, greenHorizon, LUTs[current_LUT_index]);
            	std::unique_ptr<ClassifiedImage> image = segmentFilter.classifyImage(classified_segments_hor, classified_segments_ver);
            	image->green_horizon = greenHorizon;
            	emit(image);
            	//emit(std::make_unique<ClassifiedImage>(new ClassifiedImage(classigied_segments_hor,classified_segments_ver)));
            });
        }

    }  // vision
}  // modules
