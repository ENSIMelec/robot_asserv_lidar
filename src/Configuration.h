//
// Created by Taoufik on 27/03/2020.
//

#ifndef ROBOT_CONFIGURATION_H
#define ROBOT_CONFIGURATION_H

#include <string>
#include <iostream>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

class Configuration {

private:
    Configuration();
    ~Configuration();
    // Arbre de configuration
    boost::property_tree::ptree tree;

public:
    static Configuration& instance(){
        // Instanciation au premier appel seulement
        static Configuration config;
        return config;
    }
    double getDouble(std::string string);
    int getInt(std::string string);
    float getFloat(std::string string);

};


#endif //ROBOT_CONFIGURATION_H
