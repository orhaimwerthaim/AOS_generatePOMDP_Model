#ifndef ENUM_MAP_IROS_H
#define ENUM_MAP_IROS_H

#include <map>

#include "state.h"
#include <vector>
#include <utility>
#include <string>
using namespace std;
namespace despot
{

  enum ActionType
{
    serve_can_to_personAction,
    navigateAction,
    enhanced_pickAction
	
};



  enum IrosResponseModuleAndTempEnums
  {
	  serve_can_to_person_moduleResponse,
	  serve_can_to_person_eDelivered,
	  serve_can_to_person_eNotDelivered,
	  serve_can_to_person_enumRealCase,
	  serve_can_to_person_action_success,
	  serve_can_to_person_failed,
	  navigate_moduleResponse,
	  navigate_eSuccess,
	  navigate_eFailed,
	  navigate_enumRealCase,
	  navigate_action_success,
	  navigate_failed,
	  enhanced_pick_moduleResponse,
	  enhanced_pick_res_pick_holding_can,
	  enhanced_pick_res_pick_not_holding,
	  enhanced_pick_res_pick_arm_outstretched_holding_can,
	  enhanced_pick_res_pick_without_can_arm_outstretched,
	  enhanced_pick_enumRealCase,
	  enhanced_pick_actual_pick_action_success,
	  enhanced_pick_actual_arm_outstretched_with_can,
	  enhanced_pick_actual_arm_outstretched_without_can,
	  enhanced_pick_actual_not_holding,
	  enhanced_pick_actual_dropped_the_object,

	  illegalActionObs = 100000
  };


  struct enum_map_iros
  {
	  static map<IrosResponseModuleAndTempEnums,std::string> CreateMapResponseEnumToString()
	  {
          map<IrosResponseModuleAndTempEnums,std::string> m;
          m[serve_can_to_person_eDelivered] = "serve_can_to_person_eDelivered";
          m[serve_can_to_person_eNotDelivered] = "serve_can_to_person_eNotDelivered";
          m[serve_can_to_person_action_success] = "serve_can_to_person_action_success";
          m[serve_can_to_person_failed] = "serve_can_to_person_failed";
          m[navigate_eSuccess] = "navigate_eSuccess";
          m[navigate_eFailed] = "navigate_eFailed";
          m[navigate_action_success] = "navigate_action_success";
          m[navigate_failed] = "navigate_failed";
          m[enhanced_pick_res_pick_holding_can] = "enhanced_pick_res_pick_holding_can";
          m[enhanced_pick_res_pick_not_holding] = "enhanced_pick_res_pick_not_holding";
          m[enhanced_pick_res_pick_arm_outstretched_holding_can] = "enhanced_pick_res_pick_arm_outstretched_holding_can";
          m[enhanced_pick_res_pick_without_can_arm_outstretched] = "enhanced_pick_res_pick_without_can_arm_outstretched";
          m[enhanced_pick_actual_pick_action_success] = "enhanced_pick_actual_pick_action_success";
          m[enhanced_pick_actual_arm_outstretched_with_can] = "enhanced_pick_actual_arm_outstretched_with_can";
          m[enhanced_pick_actual_arm_outstretched_without_can] = "enhanced_pick_actual_arm_outstretched_without_can";
          m[enhanced_pick_actual_not_holding] = "enhanced_pick_actual_not_holding";
          m[enhanced_pick_actual_dropped_the_object] = "enhanced_pick_actual_dropped_the_object";
          m[illegalActionObs] = "IllegalActionObs";
          return m;
        }

		static map<std::string, IrosResponseModuleAndTempEnums> CreateMapStringToEnum(map<IrosResponseModuleAndTempEnums,std::string> vecResponseEnumToString)
	  {
          map<std::string, IrosResponseModuleAndTempEnums> m;
		  map<IrosResponseModuleAndTempEnums,std::string>::iterator it;
		  for (it = vecResponseEnumToString.begin(); it != vecResponseEnumToString.end(); it++)
			{
				m[it->second] = it->first;
			}

          return m;
        }


		static map<ActionType,std::string> CreateMapActionTypeEnumToString()
	  {
          map<ActionType,std::string> m;
          m[serve_can_to_personAction] = "serve_can_to_person";
          m[navigateAction] = "navigate";
          m[enhanced_pickAction] = "enhanced_pick";

          return m;
        }

    static map<IrosResponseModuleAndTempEnums,std::string> vecResponseEnumToString;
	static map<std::string, IrosResponseModuleAndTempEnums> vecStringToResponseEnum;
	static map<ActionType,std::string> vecActionTypeEnumToString;
	static void Init();
  };

} // namespace despot
#endif /* ENUM_MAP_IROS_H */

 
