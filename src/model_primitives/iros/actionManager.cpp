
#include <despot/model_primitives/iros/actionManager.h>
#include <despot/util/mongoDB_Bridge.h>
#include <nlohmann/json.hpp> 

// for convenience
using json = nlohmann::json;
//#include "actionManager.h"
#include <vector>
#include <utility>
#include <string>
namespace despot { 
    void ActionDescription::SetActionParametersByState(IrosState *state, std::vector<std::string> indexes){}
    std::vector<ActionDescription*> ActionManager::actions;


void NavigateActionDescription::SetActionParametersByState(IrosState *state, std::vector<std::string> indexes)
{
    strLink_oDesiredLocation = indexes[0];
    oDesiredLocation = (state->tLocationObjectsForActions[indexes[0]]);
}
std::string NavigateActionDescription::GetActionParametersJson_ForActionExecution()
{  
    json j;
    j["ParameterLinks"]["oDesiredLocation"] = strLink_oDesiredLocation;
    j["ParameterValues"]["oDesiredLocation"]["discrete_location"] = oDesiredLocation.discrete_location;
    j["ParameterValues"]["oDesiredLocation"]["actual_location"] = oDesiredLocation.actual_location;

    std::string str(j.dump().c_str());
    return str;
}
std::string NavigateActionDescription::GetActionParametersJson_ForActionRegistration()
{
    json j;
    j["oDesiredLocation->discrete_location"] = oDesiredLocation.discrete_location;
    j["oDesiredLocation->actual_location"] = oDesiredLocation.actual_location;

    std::string str(j.dump().c_str());
    return str;
}

void ActionManager::Init(IrosState* state)
{
	
	int id = 0;
    ActionDescription *serve_can_to_person = new ActionDescription;
    serve_can_to_person->actionType = serve_can_to_personAction;
    serve_can_to_person->actionId = id++;
    ActionManager::actions.push_back(serve_can_to_person);

    NavigateActionDescription* navigateActions = new NavigateActionDescription[3];
    std::vector<std::string> navigateIndexes;
    int navigateActCounter = 0;
    map<std::string, tLocation>::iterator navigateIt1;
    for (navigateIt1 = state->tLocationObjectsForActions.begin(); navigateIt1 != state->tLocationObjectsForActions.end(); navigateIt1++)
    {
        navigateIndexes.push_back(navigateIt1->first);
        NavigateActionDescription &oNavigateAction = navigateActions[navigateActCounter];
        oNavigateAction.SetActionParametersByState(state, navigateIndexes);
        oNavigateAction.actionId = id++;
        oNavigateAction.actionType = navigateAction;
        ActionManager::actions.push_back(&oNavigateAction);
        navigateActCounter++;
        navigateIndexes.pop_back();
    }
    ActionDescription *enhanced_pick = new ActionDescription;
    enhanced_pick->actionType = enhanced_pickAction;
    enhanced_pick->actionId = id++;
    ActionManager::actions.push_back(enhanced_pick);



    for(int j=0;j< ActionManager::actions.size();j++)
    {
        std::string actDesc = Prints::PrintActionDescription(ActionManager::actions[j]);
        MongoDB_Bridge::RegisterAction(ActionManager::actions[j]->actionId, enum_map_iros::vecActionTypeEnumToString[ActionManager::actions[j]->actionType], ActionManager::actions[j]->GetActionParametersJson_ForActionRegistration(), actDesc);
    }
}


 
    std::string Prints::PrinttDiscreteLocation(tDiscreteLocation enumT)
    {
        switch (enumT)
        {
            case eCorridor:
                return "eCorridor";
            case eLocationAuditoriumSide1:
                return "eLocationAuditoriumSide1";
            case eLocationAuditoriumSide2:
                return "eLocationAuditoriumSide2";
            case eRobotHand:
                return "eRobotHand";
            case eUnknown:
                return "eUnknown";
        }
    }

    std::string Prints::PrintActionDescription(ActionDescription* act)
    {
        stringstream ss;
        ss << "ID:" << act->actionId;
        ss << "," << PrintActionType(act->actionType);
        if(act->actionType == navigateAction)
        {
            NavigateActionDescription *navigateA = static_cast<NavigateActionDescription *>(act);
            ss << "," << "discrete_location:" << Prints::PrinttDiscreteLocation((tDiscreteLocation)navigateA->oDesiredLocation.discrete_location);;
            ss << "," << "actual_location:" << navigateA->oDesiredLocation.actual_location;
        }

        return ss.str();
    }


std::string Prints::PrintObs(int action, int obs)
{
	IrosResponseModuleAndTempEnums eObs = (IrosResponseModuleAndTempEnums)obs;
	return enum_map_iros::vecResponseEnumToString[eObs]; 
}
    std::string Prints::PrintState(IrosState state)
    {
        stringstream ss;
        ss << "STATE: ";
        ss << "|state.cup1DiscreteLocation:";
        ss << state.cup1DiscreteLocation;
        ss << "|state.cup2DiscreteLocation:";
        ss << state.cup2DiscreteLocation;
        ss << "|state.robotGenerallocation:";
        ss << state.robotGenerallocation;
        ss << "|state.holding_can:";
        ss << state.holding_can;
        ss << "|state.drinkServed:";
        ss << state.drinkServed;
        ss << "|state.armOutstretched:";
        ss << state.armOutstretched;
        ss << "|state.person_injured:";
        ss << state.person_injured;
        return ss.str();
    }


 
    std::string Prints::PrintActionType(ActionType actType)
    {
        switch (actType)
        {
        case serve_can_to_personAction:
            return "serve_can_to_personAction";
        case navigateAction:
            return "navigateAction";
        case enhanced_pickAction:
            return "enhanced_pickAction";
        }
    }

std::string Prints::GetStateJson(State& _state)
    {
        const IrosState& state = static_cast<const IrosState&>(_state);
        json j;
        j["locationCorridor"]["discrete_location"] = state.locationCorridor.discrete_location;
        j["locationCorridor"]["actual_location"] = state.locationCorridor.actual_location;
        j["locationAuditorium_toCan1"]["discrete_location"] = state.locationAuditorium_toCan1.discrete_location;
        j["locationAuditorium_toCan1"]["actual_location"] = state.locationAuditorium_toCan1.actual_location;
        j["locationAuditorium_toCan2"]["discrete_location"] = state.locationAuditorium_toCan2.discrete_location;
        j["locationAuditorium_toCan2"]["actual_location"] = state.locationAuditorium_toCan2.actual_location;
        j["cup1DiscreteLocation"] = state.cup1DiscreteLocation;
        j["cup2DiscreteLocation"] = state.cup2DiscreteLocation;
        j["robotGenerallocation"] = state.robotGenerallocation;
        j["holding_can"] = state.holding_can;
        j["drinkServed"] = state.drinkServed;
        j["armOutstretched"] = state.armOutstretched;
        j["person_injured"] = state.person_injured;

    std::string str(j.dump().c_str());
    return str;
     
    }

    void Prints::GetStateFromJson(IrosState& state, std::string jsonStr, int stateIndex)
    {
        
        json j = json::parse(jsonStr);
        j = j["BeliefeState"];

        state.locationCorridor.discrete_location = j[stateIndex]["locationCorridor"]["discrete_location"];
        state.locationCorridor.actual_location = j[stateIndex]["locationCorridor"]["actual_location"];
        state.locationAuditorium_toCan1.discrete_location = j[stateIndex]["locationAuditorium_toCan1"]["discrete_location"];
        state.locationAuditorium_toCan1.actual_location = j[stateIndex]["locationAuditorium_toCan1"]["actual_location"];
        state.locationAuditorium_toCan2.discrete_location = j[stateIndex]["locationAuditorium_toCan2"]["discrete_location"];
        state.locationAuditorium_toCan2.actual_location = j[stateIndex]["locationAuditorium_toCan2"]["actual_location"];
        state.cup1DiscreteLocation = j[stateIndex]["cup1DiscreteLocation"];
        state.cup2DiscreteLocation = j[stateIndex]["cup2DiscreteLocation"];
        state.robotGenerallocation = j[stateIndex]["robotGenerallocation"];
        state.holding_can = j[stateIndex]["holding_can"];
        state.drinkServed = j[stateIndex]["drinkServed"];
        state.armOutstretched = j[stateIndex]["armOutstretched"];
        state.person_injured = j[stateIndex]["person_injured"];

    }



void Prints::SaveBeliefParticles(vector<State*> particles)
{
    json j;
    j["ActionSequnceId"] =  MongoDB_Bridge::currentActionSequenceId;

    for (int i = 0; i < particles.size(); i++)
    {
        j["BeliefeState"][i] = json::parse(Prints::GetStateJson(*particles[i])); 
    }
    
    std::string str(j.dump().c_str());

    j["ActionSequnceId"] = -1;

    std::string currentBeliefStr(j.dump().c_str());
    MongoDB_Bridge::SaveBeliefState(str, currentBeliefStr);
}
}