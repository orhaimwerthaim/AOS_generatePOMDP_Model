#ifndef ACTION_MANAGER_H
#define ACTION_MANAGER_H

#include "state.h"
#include <despot/model_primitives/iros/enum_map_iros.h> 
#include <vector>
#include <utility>
#include <string>
namespace despot { 


    class ActionDescription
    {
    public:
        int actionId;
        ActionType actionType;
        virtual void SetActionParametersByState(IrosState *state, std::vector<std::string> indexes);
        virtual std::string GetActionParametersJson_ForActionExecution() { return ""; };
        virtual std::string GetActionParametersJson_ForActionRegistration() { return ""; };
        
    };

class NavigateActionDescription: public ActionDescription
{
    public:
        tLocation oDesiredLocation;
        std::string strLink_oDesiredLocation;
        NavigateActionDescription(int _oDesiredLocation_Index);
        virtual void SetActionParametersByState(IrosState *state, std::vector<std::string> indexes);
        virtual std::string GetActionParametersJson_ForActionExecution();
        virtual std::string GetActionParametersJson_ForActionRegistration();
        NavigateActionDescription(){};
};



class ActionManager {
public:
	static std::vector<ActionDescription*> actions;
    static void Init(IrosState* state);
};


class Prints
{
	public:
    static std::string PrinttDiscreteLocation(tDiscreteLocation);

	static std::string PrintActionDescription(ActionDescription*);
	static std::string PrintActionType(ActionType);
	static std::string PrintState(IrosState state);
	static std::string PrintObs(int action, int obs);
    static void SaveBeliefParticles(vector<State *> particles);
    static std::string GetStateJson(State &state);
    static void GetStateFromJson(IrosState &state, std::string jsonStr, int stateIndex);
};
}
#endif //ACTION_MANAGER_H
