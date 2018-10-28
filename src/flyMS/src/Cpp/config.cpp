#include "config.hpp"

flyMSParams::flyMSParams(){}
flyMSParams::~flyMSParams(){}


void flyMSParams::loadConfigFile(std::string filename)
{
    //Check if the file exists, if not, create one with default values
    std::ifstream fid(filename.c_str());
    if (!fid.good())
    {
        std::cerr << "Warninig! Did not find config file" << std::endl;
        std::cerr << "creating new config file here: " << filename << std::endl;

        this->writeConfigFile(filename);
    }
    fid.close();


    //Read the text .yaml file
	YAML::Node rootNode = YAML::LoadFile(filename);
    this->flyMSYamlNode = rootNode["flyMSParams"];

    //take the loaded config file and put into the class varialbes
    #define X(type, fmt, name, default) this->name = this->flyMSYamlNode[#name].as<type>();
    CORE_CONFIG_TABLE
    #undef X

}

int flyMSParams::writeConfigFile(std::string filename)
{

    std::ofstream configfile(filename);

    //Start creating the config file
    configfile << "---" <<std::endl;
    configfile << "flyMSParams: " <<std::endl;

    #define X(type, fmt, name, defaultVal) configfile << "    " << #name << ": " << defaultVal << std::endl;
    CORE_CONFIG_TABLE
    #undef X

    return 0;
}