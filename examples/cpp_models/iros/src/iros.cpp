#include "iros.h"
#include <despot/core/pomdp.h> 
#include <stdlib.h>
#include <despot/solver/pomcp.h>
#include <sstream>
#include <despot/model_primitives/iros/actionManager.h> 
#include <despot/model_primitives/iros/enum_map_iros.h> 
#include <despot/model_primitives/iros/state.h> 
#include <algorithm>
#include <cmath> 
#include <despot/util/mongoDB_Bridge.h>
#include <functional> //for std::hash

using namespace std;

namespace despot {


bool AOSUtils::Bernoulli(double p)
{
	/* generate secret number between 1 and 100: */
    srand((unsigned int)time(NULL));
	int randInt = rand() % 100 + 1;
	return (p * 100) >= randInt;
}

/* ==============================================================================
 *IrosBelief class
 * ==============================================================================*/
int IrosBelief::num_particles = 5000;
std::string IrosBelief::beliefFromDB = "";
int IrosBelief::currentInitParticleIndex = -1;

IrosBelief::IrosBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	iros_(static_cast<const Iros*>(model)) {
}

	
 	std::string Iros::GetActionDescription(int actionId) const
	 {
		 return Prints::PrintActionDescription(ActionManager::actions[actionId]);
	 }

//void IrosBelief::Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates) {
void IrosBelief::Update(int actionId, OBS_TYPE obs) {
	history_.Add(actionId, obs);

	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = iros_->Copy(particles_[cur]);
		bool terminal = iros_->Step(*particle, Random::RANDOM.NextDouble(),
			actionId, reward, o);
 
		if (!terminal && o == obs) 
			{
				IrosState &iros_particle = static_cast<IrosState &>(*particle);
				//if(!Globals::IsInternalSimulation() && updates.size() > 0)
				//{
				//	IrosState::SetAnyValueLinks(&iros_particle);
				//	map<std::string, bool>::iterator it;
				//	for (it = updates.begin(); it != updates.end(); it++)
				//	{
				//		*(iros_particle.anyValueUpdateDic[it->first]) = it->second; 
				//	} 
				//}
				updated.push_back(particle);
		} else {
			iros_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		iros_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
 
}

/* ==============================================================================
 * IrosPOMCPPrior class
 * ==============================================================================*/

class IrosPOMCPPrior: public POMCPPrior {
private:
	const Iros* iros_;

public:
	IrosPOMCPPrior(const Iros* model) :
		POMCPPrior(model),
		iros_(model) {
	}

	void ComputePreference(const State& state) {
		const IrosState& iros_state = static_cast<const IrosState&>(state);
		weighted_preferred_actions_.clear();
        legal_actions_.clear();
		preferred_actions_.clear();
        std::vector<double> weighted_preferred_actions_un_normalized;

        double heuristicValueTotal = 0;
		for (int a = 0; a < 5; a++) {
            weighted_preferred_actions_un_normalized.push_back(0);
			double reward = 0;
			bool meetPrecondition = false; 
			Iros::CheckPreconditions(iros_state, reward, meetPrecondition, a);
            if(meetPrecondition)
            {
                legal_actions_.push_back(a);
                double __heuristicValue; 
                Iros::ComputePreferredActionValue(iros_state, __heuristicValue, a);
                heuristicValueTotal += __heuristicValue;
                weighted_preferred_actions_un_normalized[a]=__heuristicValue;
            }
        }

        if(heuristicValueTotal > 0)
        {
            for (int a = 0; a < 5; a++) 
            {
                weighted_preferred_actions_.push_back(weighted_preferred_actions_un_normalized[a] / heuristicValueTotal);
            } 
        }
    }
};

/* ==============================================================================
 * Iros class
 * ==============================================================================*/

Iros::Iros(){
	
}

int Iros::NumActions() const {
	return ActionManager::actions.size();
}

double Iros::ObsProb(OBS_TYPE obs, const State& state, int actionId) const {
	return 0.9;
}

 


std::default_random_engine Iros::generator;

std::discrete_distribution<> Iros::navigate_discrete_dist1{1.0,0.0}; //AOS.SampleDiscrete(enumRealCase,{1.0,0.0})
std::discrete_distribution<> Iros::enhanced_pick_discrete_dist2{0.727,0.181,0.09,0.0}; //AOS.SampleDiscrete(enumRealCase,{0.727,0.181,0.09,0.0})
std::discrete_distribution<> Iros::enhanced_pick_discrete_dist3{0.1,0.0,0.0,0.9}; //AOS.SampleDiscrete(enumRealCase,{0.1,0.0,0.0,0.9})

State* Iros::CreateStartState(string type) const {
    IrosState* startState = memory_pool_.Allocate();
    IrosState& state = *startState;
    startState->tDiscreteLocationObjects.push_back(eCorridor);
    startState->tDiscreteLocationObjects.push_back(eLocationAuditoriumSide1);
    startState->tDiscreteLocationObjects.push_back(eLocationAuditoriumSide2);
    startState->tDiscreteLocationObjects.push_back(eRobotHand);
    startState->tDiscreteLocationObjects.push_back(eUnknown);
    state.locationCorridor = tLocation();
    state.locationCorridor.discrete_location = eCorridor;
     state.locationCorridor.actual_location = true;
    state.locationAuditorium_toCan1 = tLocation();
    state.locationAuditorium_toCan1.discrete_location = eLocationAuditoriumSide1;
     state.locationAuditorium_toCan1.actual_location = true;
    state.locationAuditorium_toCan2 = tLocation();
    state.locationAuditorium_toCan2.discrete_location = eLocationAuditoriumSide2;
     state.locationAuditorium_toCan2.actual_location = true;
    state.cup1DiscreteLocation = eLocationAuditoriumSide1;
    state.cup2DiscreteLocation = eLocationAuditoriumSide2;
    state.holding_can=false;
    state.drinkServed=false;
    state.armOutstretched=false;
    state.person_injured=false;
    startState->tLocationObjectsForActions["state.locationCorridor"] = (state.locationCorridor);
    startState->tLocationObjectsForActions["state.locationAuditorium_toCan1"] = (state.locationAuditorium_toCan1);
    startState->tLocationObjectsForActions["state.locationAuditorium_toCan2"] = (state.locationAuditorium_toCan2);
    state.robotGenerallocation=state.locationAuditorium_toCan2.discrete_location;
    if (ActionManager::actions.size() == 0)
    {
        ActionManager::Init(const_cast <IrosState*> (startState));
    }
    return startState;
}



Belief* Iros::InitialBelief(const State* start, string type) const {
	int N = IrosBelief::num_particles;
	vector<State*> particles(N);
	for (int i = 0; i < N; i++) {
        IrosBelief::currentInitParticleIndex = i;
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}
    IrosBelief::currentInitParticleIndex = -1;
	return new IrosBelief(particles, this);
}
 

 

POMCPPrior* Iros::CreatePOMCPPrior(string name) const { 
		return new IrosPOMCPPrior(this);
}

void Iros::PrintState(const State& state, ostream& ostr) const {
	const IrosState& farstate = static_cast<const IrosState&>(state);
	if (ostr)
		ostr << Prints::PrintState(farstate);
}

void Iros::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {
	const IrosState& farstate = static_cast<const IrosState&>(state);
	
	ostr << observation <<endl;
}

void Iros::PrintBelief(const Belief& belief, ostream& out) const {
	 out << "called PrintBelief(): b printed"<<endl;
		out << endl;
	
}

void Iros::PrintAction(int actionId, ostream& out) const {
	out << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
}

State* Iros::Allocate(int state_id, double weight) const {
	IrosState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* Iros::Copy(const State* particle) const {
	IrosState* state = memory_pool_.Allocate();
	*state = *static_cast<const IrosState*>(particle);
	state->SetAllocated();



	return state;
}

void Iros::Free(State* particle) const {
	memory_pool_.Free(static_cast<IrosState*>(particle));
}

int Iros::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}
 
bool Iros::Step(State& s_state__, double rand_num, int actionId, double& reward,
	OBS_TYPE& observation) const {
    reward = 0;
	bool isNextStateFinal = false;
	Random random(rand_num);
	int __moduleExecutionTime = -1;
	bool meetPrecondition = false;
	
	IrosState &state__ = static_cast<IrosState &>(s_state__);
	 logd << "[Iros::Step] Selected Action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << "||State"<< Prints::PrintState(state__);
	CheckPreconditions(state__, reward, meetPrecondition, actionId);
	 
	State *s_state = Copy(&s_state__);
	IrosState &state = static_cast<IrosState &>(*s_state);

	
	SampleModuleExecutionTime(state__, rand_num, actionId, __moduleExecutionTime);

	ExtrinsicChangesDynamicModel(state, state__, rand_num, actionId, __moduleExecutionTime);

	State *s_state_ = Copy(&s_state__);
	IrosState &state_ = static_cast<IrosState &>(*s_state_);

    double tReward = 0;
	ModuleDynamicModel(state, state_, state__, rand_num, actionId, tReward,
					   observation, __moduleExecutionTime, meetPrecondition);
	reward += tReward;

	Free(s_state);
	Free(s_state_);
	bool finalState = ProcessSpecialStates(state__, reward);

    if (!meetPrecondition)
	{
		//__moduleExecutionTime = 0;
		//observation = illegalActionObs;
		//return false;
	}
	return finalState;
}

void Iros::CheckPreconditions(const IrosState& state, double &reward, bool &__meetPrecondition, int actionId)
    {
        ActionType &actType = ActionManager::actions[actionId]->actionType;
        __meetPrecondition = true;
            if(actType == serve_can_to_personAction)
            {
                __meetPrecondition=state.holding_can==true&&state.armOutstretched==false&&state.robotGenerallocation==eCorridor;
                if(!__meetPrecondition) reward += -12;
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
                __meetPrecondition=state.robotGenerallocation!=eUnknown;
                if(!__meetPrecondition) reward += -10;
            }
            if(actType == enhanced_pickAction)
            {
                __meetPrecondition=(state.cup1DiscreteLocation==state.robotGenerallocation||state.cup2DiscreteLocation==state.robotGenerallocation)&&!state.holding_can;
                if(!__meetPrecondition) reward += -10;
            }
    }

void Iros::ComputePreferredActionValue(const IrosState& state, double &__heuristicValue, int actionId)
    {
        __heuristicValue = 0;
        ActionType &actType = ActionManager::actions[actionId]->actionType;
            if(actType == serve_can_to_personAction)
            {
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
            }
            if(actType == enhanced_pickAction)
            {
            }
        __heuristicValue = __heuristicValue < 0 ? 0 : __heuristicValue;
    }

void Iros::SampleModuleExecutionTime(const IrosState& farstate, double rand_num, int actionId, int &__moduleExecutionTime) const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    if(actType == serve_can_to_personAction)
    {
    }
    if(actType == navigateAction)
    {
    }
    if(actType == enhanced_pickAction)
    {
    }
}

void Iros::ExtrinsicChangesDynamicModel(const IrosState& state, IrosState& state_, double rand_num, int actionId, const int &__moduleExecutionTime)  const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
}

void Iros::ModuleDynamicModel(const IrosState &state, const IrosState &state_, IrosState &state__, double rand_num, int actionId, double &__reward, OBS_TYPE &observation, const int &__moduleExecutionTime, const bool &__meetPrecondition) const
{
    std::hash<std::string> hasher;
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    observation = -1;
    int startObs = observation;
    std::string __moduleResponseStr = "NoStrResponse";
    OBS_TYPE &__moduleResponse = observation;
    if(actType == serve_can_to_personAction)
    {
        IrosResponseModuleAndTempEnums  realCase;
        realCase=__meetPrecondition?serve_can_to_person_action_success:serve_can_to_person_failed;
        state__.person_injured=state.armOutstretched==true;
        state__.drinkServed=(realCase==serve_can_to_person_action_success&&!state__.person_injured);
        __moduleResponse=serve_can_to_person_eDelivered;
        __reward=state__.person_injured?-20:-8;
    }
    if(actType == navigateAction)
    {
        NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
        tLocation &oDesiredLocation = act.oDesiredLocation;
        IrosResponseModuleAndTempEnums  realCase;
        realCase=(IrosResponseModuleAndTempEnums)(navigate_enumRealCase + 1 + Iros::navigate_discrete_dist1(Iros::generator));
        if(realCase==navigate_action_success)state__.robotGenerallocation=oDesiredLocation.discrete_location;
        if(realCase==navigate_action_success)__moduleResponse=navigate_eSuccess;
        else __moduleResponse=navigate_eFailed;
        __reward=-6;
    }
    if(actType == enhanced_pickAction)
    {
        IrosResponseModuleAndTempEnums  realCase;
        realCase=!__meetPrecondition?enhanced_pick_actual_not_holding:(state.cup1DiscreteLocation==state.robotGenerallocation?(IrosResponseModuleAndTempEnums)(enhanced_pick_enumRealCase + 1 + Iros::enhanced_pick_discrete_dist2(Iros::generator)):(IrosResponseModuleAndTempEnums)(enhanced_pick_enumRealCase + 1 + Iros::enhanced_pick_discrete_dist3(Iros::generator)));
        state__.holding_can=(realCase==enhanced_pick_actual_pick_action_success||realCase==enhanced_pick_actual_arm_outstretched_with_can);
        if(state.cup1DiscreteLocation==state.robotGenerallocation){state__.cup1DiscreteLocation=(realCase==enhanced_pick_actual_pick_action_success||realCase==enhanced_pick_actual_arm_outstretched_with_can)?eRobotHand:(realCase==enhanced_pick_actual_not_holding?state.cup1DiscreteLocation:eUnknown);
        };
        if(state.cup2DiscreteLocation==state.robotGenerallocation){state__.cup2DiscreteLocation=(realCase==enhanced_pick_actual_pick_action_success||realCase==enhanced_pick_actual_arm_outstretched_with_can)?eRobotHand:(realCase==enhanced_pick_actual_not_holding?state.cup2DiscreteLocation:eUnknown);
        };
        state__.armOutstretched=realCase==enhanced_pick_actual_arm_outstretched_with_can||realCase==enhanced_pick_actual_arm_outstretched_without_can;
        if(realCase==enhanced_pick_actual_not_holding||realCase==enhanced_pick_actual_dropped_the_object)__moduleResponse=enhanced_pick_res_pick_not_holding;
        if(realCase==enhanced_pick_actual_pick_action_success)__moduleResponse=enhanced_pick_res_pick_holding_can;
        if(realCase==enhanced_pick_actual_arm_outstretched_with_can)__moduleResponse=AOSUtils::Bernoulli(0.818)?enhanced_pick_res_pick_arm_outstretched_holding_can:enhanced_pick_res_pick_without_can_arm_outstretched;
        if(realCase==enhanced_pick_actual_arm_outstretched_without_can)__moduleResponse=enhanced_pick_res_pick_without_can_arm_outstretched;
        __reward=-2;
    }
    if(__moduleResponseStr != "NoStrResponse")
    {
        IrosResponseModuleAndTempEnums responseHash = (IrosResponseModuleAndTempEnums)hasher(__moduleResponseStr);
        enum_map_iros::vecResponseEnumToString[responseHash] = __moduleResponseStr;
        enum_map_iros::vecStringToResponseEnum[__moduleResponseStr] = responseHash;
        __moduleResponse = responseHash;
    }
    if(startObs == __moduleResponse)
    {
    stringstream ss;
    ss << "Observation/__moduleResponse Not initialized!!! on action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
    loge << ss.str() << endl;
    throw 1;
    }
}

bool Iros::ProcessSpecialStates(IrosState &state, double &reward) const
{
    bool isFinalState = false;
    if(state.OneTimeRewardUsed[0])
    {
        if (state.drinkServed == true)
        {
            reward += 30;
            isFinalState = true;
        }
    }
    if(state.OneTimeRewardUsed[1])
    {
        if (state.person_injured == true)
        {
            reward += -10;
            isFinalState = true;
        }
    }
    if(state.OneTimeRewardUsed[2])
    {
        if (state.cup1DiscreteLocation == eUnknown && state.cup2DiscreteLocation == eUnknown)
        {
            reward += 0;
            isFinalState = true;
        }
    }
    return isFinalState;
}





std::string Iros::PrintObs(int action, OBS_TYPE obs) const 
{
	return Prints::PrintObs(action, obs);
}

std::string Iros::PrintStateStr(const State &state) const { return ""; };
}// namespace despot
