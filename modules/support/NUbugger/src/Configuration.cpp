/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include <yaml-cpp/yaml.h>

#include "messages/support/nubugger/proto/Message.pb.h"
#include "utility/time/time.h"
#include "utility/file/fileutil.h"
#include "utility/strutil/strutil.h"

/**
* @author Monica Olejniczak
*/
namespace modules {
namespace support {
    using utility::file::listFiles;
    using utility::time::getUtcTimestamp;
    using utility::strutil::split;
    using messages::support::nubugger::proto::Message;
    using messages::support::nubugger::proto::ConfigurationState;

    void processNode(ConfigurationState::Node& proto, YAML::Node& yaml);

    /**
     * Processes a null YAML node by setting its respective type on the protocol node.
     *
     * @param proto The protocol node.
     */
    void processNullNode(ConfigurationState::Node& proto) {
        // set the type of the protocol node to the NULL_VALUE Node Type
        proto.set_type(ConfigurationState::Node::NULL_VALUE);
    }

    /**
     * Processes a scalar YAML node by setting its respective type on the protocol node. A scalar node may comprise a
     * boolean, number or string.
     *
     * @param proto The protocol node.
     * @param yaml A scalar YAML node.
     */
    void processScalarNode(ConfigurationState::Node& proto, YAML::Node& yaml) {
        bool bValue;
        // check if the yaml node is a boolean
        if (YAML::convert<bool>::decode(yaml, bValue)) {
            // set the type of the protocol node to the BOOLEAN Node Type and escape the case
            proto.set_type(ConfigurationState::Node::BOOLEAN);
            // set the value of the protocol node to the boolean
            proto.set_boolean_value(bValue);
            return;
        }
        long lValue;
        // check if the yaml node is a long
        if (YAML::convert<long>::decode(yaml, lValue)) {
            // set the type of the protocol node to the LONG Node Type and escape the case
            proto.set_type(ConfigurationState::Node::LONG);
            // set the value of the protocol node to the long
            proto.set_long_value(lValue);
            return;
        }
        double dValue;
        // check if the yaml node is a double
        if (YAML::convert<double>::decode(yaml, dValue)) {
            // set the type of the protocol node to the DOUBLE Node Type and escape the case
            proto.set_type(ConfigurationState::Node::DOUBLE);
            // set the value of the protocol node to the double
            proto.set_double_value(dValue);
            return;
        }
        // declare a string and then decode it
        std::string sValue;
        YAML::convert<std::string>::decode(yaml, sValue);
        // set the type of the protocol node to the STRING Node Type and escape the case
        proto.set_type(ConfigurationState::Node::STRING);
        // set the value of the protocol node to the string
        proto.set_string_value(sValue);
    }

    /**
     * Processes a sequence YAML node by settings its respective type on the protocol node. It then iterates through
     * all the nodes in this sequence and processes each node.
     *
     * @param proto The protocol node.
     * @param yaml A sequence YAML node.
     */
    void processSequenceNode(ConfigurationState::Node& proto, YAML::Node& yaml) {
        // set the type of the protocol node to the SEQUENCE Node Type
        proto.set_type(ConfigurationState::Node::SEQUENCE);
        // iterate through every yaml node in the sequence
        for (auto&& yamlNode : yaml) {
            // recursively call this function where the protocol node is a new sequence value and the yaml
            // node is the current iteration within the list
            processNode(*proto.add_sequence_value(), yamlNode);
        }
    }

    /**
     * Processes a map YAML node by setting its respective type on the protocol node. It then iterates through all the
     * nodes in this map and processes each node.
     *
     * @param proto The protocol node.
     * @param yaml A map YAML node.
     */
    void processMapNode(ConfigurationState::Node& proto, YAML::Node& yaml) {
        // set the type of the protocol node to the MAP Node Type
        proto.set_type(ConfigurationState::Node::MAP);
        // iterate through every yaml node in the map
        for (auto&& yamlNode : yaml) {
            // create a new map value from the protocol node
            auto* map = proto.add_map_value();
            // set the name of this new node to the key of the yaml node and convert it to a string
            map->set_name(yamlNode.first.as<std::string>());
            // recursively call this function with a pointer to the object that is a part of the protocol
            // buffer as its first parameter and use the value of the yaml node for the second parameter
            processNode(*map->mutable_value(), yamlNode.second);
        }
    }

    /**
     * Processes a particular YAML node into its equivalent protocol node. The tag is initially set and the type of the
     * YAML node is then evaluated.
     *
     * @param proto The protocol node.
     * @param yaml A YAML node.
     */
    void processNode(ConfigurationState::Node& proto, YAML::Node& yaml) {
        // set the tag of the protocol buffer if it exists
        if (yaml.Tag() != "?") {
            proto.set_tag(yaml.Tag());
        }
        switch (yaml.Type()) {
            case YAML::NodeType::Undefined:
                // TODO Handle it somehow?
                break;
            case YAML::NodeType::Null:
                processNullNode(proto);
                break;
                // strings and numbers
            case YAML::NodeType::Scalar:
                processScalarNode(proto, yaml);
                break;
                // arrays and lists
            case YAML::NodeType::Sequence:
                processSequenceNode(proto, yaml);
                break;
                // hashes and dictionaries
            case YAML::NodeType::Map:
                processMapNode(proto, yaml);
                break;
        }
    }

    /**
     * Processes a configuration file by setting the type of the protocol node, loading the file through its path and
     * then processing the information within that file.
     *
     * @param path The path to the configuration file.
     * @param name The name of the configuration file.
     * @param proto The protocol node.
     * @param directories The list of known directory nodes.
     */
    void processFile(std::string path, std::string name, ConfigurationState::Node& proto, std::map<std::string,
    ConfigurationState::KeyPair>& directories, std::string directoryPath) {
        // create a new map value from the current protocol node
        auto* map = proto.add_map_value();
        // set the name of the map
        map->set_name(name);
        // set the Node Type of the map
        proto.set_type(ConfigurationState::Node::FILE);
        // load the YAML file using the current iteration
        YAML::Node yaml = YAML::LoadFile(path);
        // processes the yaml node into a protocol node
        processNode(*map->mutable_value(), yaml);
    }

    /**
     * Processes a configuration file path recursively. The current path is split into nodes, where each node is either
     * a directory or a file name. If the there is only one node, then it can be assumed that the current path is a
     * configuration file and is able to be read and processed. Otherwise, it is a directory and the current path
     * needs to be adjusted such that it does not contain the current working directory.
     *
     * @param path The path of the configuration file.
     * @param currentPath The current path to use for recursion.
     * @param proto The protocol node.
     * @param directories The list of known directory nodes.
     */
    void processPath(std::string path, std::string currentPath, ConfigurationState::Node& proto, std::map<std::string,
    ConfigurationState::KeyPair>& directories) {
        // get every node within the current path using the '/' delimeter
        auto nodes = split(currentPath, '/');
        // get the name of the current file or directory
        std::string name = nodes.front();
        // calculate the path to the directory
        std::string directoryPath = path.substr(0, path.find(currentPath) + name.length());
        // check if we are at a file and assume that it is not an empty directory
        if (nodes.size() == 1) {
            processFile(path, name, proto, directories, directoryPath);
        } else {
            // retrieve the directory protocol node given the path to the directory
            auto iterator = directories.find(directoryPath);
            // get the new path by removing the first node from the current path string
            std::string newPath = currentPath.substr(nodes.front().length() + 1, currentPath.length());
            // check if the directory does not exist
            if (iterator == directories.end()) {
                auto* map = proto.add_map_value();      // create a new map value from the current protocol node
                map->set_name(name);                    // set the name of the map
                // set the Node Type of the directory
                proto.set_type(ConfigurationState::Node::DIRECTORY);
                // add the directory to the map
                directories[directoryPath] = *map;
                // processes the new path
                processPath(path, newPath, *map->mutable_value(), directories);
            } else {
                // get the directory from the iterator
                ConfigurationState::KeyPair directory = iterator->second;
                // processes the new path using the directory that was found
                std::cout << "current " << currentPath << std::endl << "directory name " << directory.name() << std::endl;
                processPath(path, newPath, *directory.mutable_value(), directories);
            }
        }
    }

    /**
     * Process a configuration file path by calling its recursive function.
     *
     * @param path The path of the configuration file.
     * @param proto The protocol node.
     * @param directories The list of known directory nodes.
     */
    void processPath(std::string path, ConfigurationState::Node& proto, std::map<std::string,
    ConfigurationState::KeyPair>& directories) {
        processPath(path, path, proto, directories);
    }

    /**
     * A command sent when the network requests the configuration state data. It retrieves all of the configuration
     * files and converts them into readable messages to send back over the network.
     */
    void NUbugger::sendConfigurationState() {
        // specify the base directory
        std::string directory = "config";
        // get the list of file paths in the shared config directory and ensure it is recursive
        std::vector<std::string> paths = listFiles(directory, true);
        // create the list of directories
        std::map<std::string, ConfigurationState::KeyPair> directories;

        // create a new message
        Message message;
        message.set_type(Message::CONFIGURATION_STATE);             // set the message type to the configuration state
        message.set_filter_id(0);                                   // ensure the message is not filtered
        message.set_utc_timestamp(getUtcTimestamp());               // set the timestamp of the message

        auto* state = message.mutable_configuration_state();        // create the configuration state from the message
        auto* root = state->mutable_root();                         // retrieve the root node from the state
        root->set_type(ConfigurationState::Node::DIRECTORY);        // process the root directory

        for (auto&& path : paths) {                                 // iterate through every file path in the config directory
            processPath(path, *root, directories);                  // process the path using the root node
        }
        // send the message over the network
        send(message);
    }
}
}