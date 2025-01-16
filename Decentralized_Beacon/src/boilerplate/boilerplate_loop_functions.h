#ifndef BOILERPLATE_LOOP_FUNCTIONS_H
#define BOILERPLATE_LOOP_FUNCTIONS_H
#include<time.h>
#include <sys/stat.h> 
#include <sys/types.h>
#include <exception>
#include <experimental/filesystem>
#include <iostream>
#include <functional>
#include <fstream>


namespace argos {
   class CPrototypeEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_radios_actuator.h>
#include <map>

namespace argos {
namespace fs = std::experimental::filesystem;
   class CBoilerplateLoopFunctions : public CLoopFunctions {

      public:
         typedef std::map<CPrototypeEntity*, std::vector<CVector3> > TWaypointMap;
         TWaypointMap m_tWaypoints;
         virtual void Init(TConfigurationNode& t_tree);
         virtual bool IsExperimentFinished();
         virtual void PreStep();
         virtual void PostStep();
         inline const TWaypointMap& GetWaypoints() const {
            return m_tWaypoints;
         }

         // The path of the output file.
         std::string m_strOutFile;
         std::ofstream m_cOutFile2;
         std::ofstream m_cOutFile3;
         std::ofstream m_cOutFile8;
         std::ofstream m_cOutFile10;

         //The stream associated to the output file.
         std::ofstream m_cOutFile;

         //The path of the output file.
         std::string m_strOutFile2;
         std::string m_strOutFile3;
         std::string m_strOutFile8;
         std::string m_strOutFile10;

         std::string filename;
         std::ostringstream oss;

      private:
         int v1;
         int v;
         double sum;
         float totalDis;
         float temp;
         const int msArraySize = 400;
         int j;
         int movingMessage=0;
         int a;
         int b;
         float enVec[16][16][6];
         int simcount = 0;
         int terminationCount = 0;
         int updateFlag = 0;
         int lastzeroCounter = 0;
         int mapfg = 1;
         char date[9];
         float currentPercentage = 0;
         int zeroCounter = 0;
         CPrototypeEntity* cFB;
   };
}

#endif

