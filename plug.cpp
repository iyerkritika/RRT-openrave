#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "rrt.cpp"
using namespace OpenRAVE;

class newmod : public ModuleBase
{
public:
    newmod(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RRT",boost::bind(&newmod::MyCommand,this,_1,_2),
                        "Input format: RRT goal %f,%f,%f,%f,%f,%f,%f; goalBias %f; step 0.2;start %f,%f,%f,%f,%f,%f,%f;");
    }
    virtual ~newmod() {}

    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
      // getting goal config, gb,step and start config
      std::string input;
      std::vector<double> goal,start;
      char temp ='1';
      sinput >> input;
      double q;
      if (input == "goal")
      while(temp!= ';')
      {
        sinput >> q;
        goal.push_back(q);
        sinput >> temp;
      }
      sinput >> input;
      float gb = 0.2;
      temp ='1';
      if (input == "goalBias")
        sinput >> gb;
      sinput >> temp;
      sinput>>input;
      float st =0.3;
      temp ='1';
      if (input == "step")
        sinput >> st;
      sinput >> temp;
      temp='1';
      sinput >> input;
      if (input == "start")
      while(temp!= ';')
      {
        sinput >> q;
        start.push_back(q);
        sinput >> temp;
      }
      // calling RRT
      RRTpath(GetEnv(),goal,gb,st,start);
      //rrt has smoother in it
      return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "newmod" ) {
        return InterfaceBasePtr(new newmod(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("newmod");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
