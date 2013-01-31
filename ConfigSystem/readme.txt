////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////GUIDELINES FOR MAKING/READING CONFIG FILES USING THE NUBOTS CONFIGURATION SYSTEM////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//////// Created by: 	Sophie Calland, Mitchell Metcalfe	////////////////////////////////////
//////// Edited by:											////////////////////////////////////
//////// Last updated:	17/12/2012							////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

NOTE: Add as we go, make sure to write everything down, why etc to make it easier for reference later
on and for new people later. :P

1. INFO: 
		This configuration system uses the Boost Property Tree to parse, store and write JSON files. 
	Each separate component of the NUbots code contains it's own JSON file, which is stored in one
	tree upon startup. 
	
2. STANDARDS:
		2.1. CODE:
				All code must be commented such that it can be doxygenned and easily read and interpreted
			by a human being. 
				Everything is found in the ConfigSystem namespace. Please include macro guards and 
			namespaces where appropriate.
		2.2. JSON:
				Arrays containing the preset possible values are not to be edited by programs. These are
			constants in the file and must be changed manually (please document these changes in the 
			appropriate section at the end of this readme.txt file, and your reasons for doing so, 
			according to the template provided). 
				All values stored in paths must make sense and be stored in the appropriate sections. For 
			example, when making a new constant for "behaviour", don't store it in "vision". Also ensure
			paths make sense in their respective sections.
				All contents are stored as strings in the Property Tree, meaning type information is lost
			and everything is converted to a string. If, for some reason, you're creating a new variable
			manually, please adhere	to the following JSON convention:
						.
						.
						.
						"someconstant":
						{
							"vector": "false",
							"vector_length": "0",
							
							"value": "I am a string",
							"type": "std::string",
							
							"modified": "true or false",
							"locked": "true or false",
							
							"rules":
							{
								"range":
								{
									"upper": "upperbound",
									"lower": "lowerbound"
								},
								"possiblevalues":
								[
									{
										"value": "possible value 1"
									},
									{
										"value": "possible value 2"
									},
									.
									.
									.
								],
								"conflicts":
								[
									{
										"path": "path to conflict variable 1"
									},
									{
										"path": "path to conflict variable 2"
									},
									.
									.
									.
								]
							}
						}
						
				For a new constant, value and type must be set to something, else it will be unusable.
				The array "conflicts" contains the path to variables where conflicts might arise (which
			will need to be changed based on their set of rules). For no conflicts, this can be left as
			an empty array.
				Ideally, "possiblevalues" should not exist for any type other than a string, or for
			constants where only a few values are viable. Otherwise, "range" should be used. Please note
			these values contain the specific rules for the constant and might cause problems if edited.
			
			
			
			
COMPILE USING:
 c++ -I /home/sophie/boost_1_40_0 TestConfig.cpp ConfigStorageManager.cpp -o TestConfig
 c++ -I /home/sophie/boost_1_40_0 TestConfig.cpp ConfigEnforcer.cpp ConfigStorageManager.cpp ConfigManager.cpp -o TestConfig
 
 
 
 
 
 
 
SAVE THIS CONFIG STUFF FOR LATER:
 										"limits":
										{
											"range":
											{
												"upperlimit": "upper 1",
												"lowerlimit": "lower 1"
											},
											"unallowed_values": 
											[
												{
													"value": "unallowed value 1"
												},
												{
													"value": "unallowed value 2"
												},
												. 
												.
												.
											]
										}

