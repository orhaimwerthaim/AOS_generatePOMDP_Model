#ifndef STATE_H
#define STATE_H
#include <vector>
#include <despot/core/pomdp.h> 
#include "state_var_types.h"
namespace despot
{


class IrosState : public State {
public:
    bool OneTimeRewardUsed[3]={true,true,true};
    std::vector<tDiscreteLocation> tDiscreteLocationObjects;
    std::map<std::string, tLocation> tLocationObjectsForActions;
    tLocation locationCorridor;
    tLocation locationAuditorium_toCan1;
    tLocation locationAuditorium_toCan2;
    tDiscreteLocation cup1DiscreteLocation;
    tDiscreteLocation cup2DiscreteLocation;
    tDiscreteLocation robotGenerallocation;
    bool holding_can;
    bool drinkServed;
    bool armOutstretched;
    bool person_injured;
    std::map<std::string, anyValue*> anyValueUpdateDic;


	public:
		static void SetAnyValueLinks(IrosState *state);
		
};
}
#endif //STATE_H