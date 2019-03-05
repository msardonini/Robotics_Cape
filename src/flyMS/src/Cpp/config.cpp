#include "config.hpp"


//Default Constructor
flyMSParams::flyMSParams(){}

//Copy Constructor
flyMSParams::flyMSParams(const flyMSParams &obj) {
    // Copy all of the values from the copy to the new object
    #define X(type, fmt, name, default) this->config.name = obj.config.name;
    CORE_CONFIG_TABLE
    #undef X

}

//Default Destructor
flyMSParams::~flyMSParams(){}



/* This namespace assists with creating yaml files. The 
    macro defines default values to be created,
    and these templated functions convert the values to 
    the proper strings to save in the yaml file depending on the type" */
namespace conv2string
{
    template <typename T>
    const std::string getDefaultValueString(T val)
    {
        std::string tmpString(std::to_string(val));
        return tmpString;
    }

    template <> 
    const std::string getDefaultValueString<bool>(bool val)
    {
        std::string tmpString;
        (val) ? (tmpString += "true") : (tmpString += "false");
        return tmpString;
    }

    template <>
    const std::string getDefaultValueString<std::string>(std::string val)
    {
        return val;
    }
}



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
    #define X(type, fmt, name, default) this->config.name = this->flyMSYamlNode[#name].as<type>();
    CORE_CONFIG_TABLE
    #undef X

    this->config_filepath = filename;
    this->isLoaded = true;
}

int flyMSParams::writeConfigFile(std::string filename)
{

    std::ofstream configfile(filename);

    //Start creating the config file
    configfile << "---" <<std::endl;
    configfile << "flyMSParams: " <<std::endl;


  #define X(type, fmt, name, defaultVal) configfile << "    " << #name << ": " << conv2string::getDefaultValueString(static_cast<type>(defaultVal)) << std::endl;
    CORE_CONFIG_TABLE
    #undef X

    return 0;
}