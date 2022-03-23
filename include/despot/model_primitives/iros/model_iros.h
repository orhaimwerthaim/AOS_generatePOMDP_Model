#ifndef MODEL_IROS_H
#define MODEL_IROS_H

#include <map>
#include <despot/core/pomdp.h>
#include "state.h"
#include <vector>
#include <utility>
#include <string>
using namespace std; 
namespace despot
{

  struct state_data
  {
    int state;
    IrosState *oState = NULL;
    int totalSamples = 0;
    map<int, int> nextStateSampled;

    state_data(int _state)
    {
      state = _state;
    }
  };
  struct model_iros
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

 
