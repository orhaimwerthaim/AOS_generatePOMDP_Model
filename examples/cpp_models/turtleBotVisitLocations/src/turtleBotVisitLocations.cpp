#include "turtleBotVisitLocations.h"
#include <despot/core/pomdp.h> 
#include <stdlib.h>
#include <despot/solver/pomcp.h>
#include <sstream>
#include <despot/model_primitives/turtleBotVisitLocations/actionManager.h> 
#include <despot/model_primitives/turtleBotVisitLocations/enum_map_turtleBotVisitLocations.h> 
#include <despot/model_primitives/turtleBotVisitLocations/state.h> 
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
 *TurtleBotVisitLocationsBelief class
 * ==============================================================================*/
int TurtleBotVisitLocationsBelief::num_particles = 5000;
std::string TurtleBotVisitLocationsBelief::beliefFromDB = "";
int TurtleBotVisitLocationsBelief::currentInitParticleIndex = -1;

TurtleBotVisitLocationsBelief::TurtleBotVisitLocationsBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	turtleBotVisitLocations_(static_cast<const TurtleBotVisitLocations*>(model)) {
}

	
 	std::string TurtleBotVisitLocations::GetActionDescription(int actionId) const
	 {
		 return Prints::PrintActionDescription(ActionManager::actions[actionId]);
	 }

//void TurtleBotVisitLocationsBelief::Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates) {
void TurtleBotVisitLocationsBelief::Update(int actionId, OBS_TYPE obs) {
	history_.Add(actionId, obs);

	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = turtleBotVisitLocations_->Copy(particles_[cur]);
		bool terminal = turtleBotVisitLocations_->Step(*particle, Random::RANDOM.NextDouble(),
			actionId, reward, o);
 
		if (!terminal && o == obs) 
			{
				TurtleBotVisitLocationsState &turtleBotVisitLocations_particle = static_cast<TurtleBotVisitLocationsState &>(*particle);
				//if(!Globals::IsInternalSimulation() && updates.size() > 0)
				//{
				//	TurtleBotVisitLocationsState::SetAnyValueLinks(&turtleBotVisitLocations_particle);
				//	map<std::string, bool>::iterator it;
				//	for (it = updates.begin(); it != updates.end(); it++)
				//	{
				//		*(turtleBotVisitLocations_particle.anyValueUpdateDic[it->first]) = it->second; 
				//	} 
				//}
				updated.push_back(particle);
		} else {
			turtleBotVisitLocations_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		turtleBotVisitLocations_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
 
}

/* ==============================================================================
 * TurtleBotVisitLocationsPOMCPPrior class
 * ==============================================================================*/

class TurtleBotVisitLocationsPOMCPPrior: public POMCPPrior {
private:
	const TurtleBotVisitLocations* turtleBotVisitLocations_;

public:
	TurtleBotVisitLocationsPOMCPPrior(const TurtleBotVisitLocations* model) :
		POMCPPrior(model),
		turtleBotVisitLocations_(model) {
	}

	void ComputePreference(const State& state) {
		const TurtleBotVisitLocationsState& turtleBotVisitLocations_state = static_cast<const TurtleBotVisitLocationsState&>(state);
		weighted_preferred_actions_.clear();
        legal_actions_.clear();
		preferred_actions_.clear();
        std::vector<double> weighted_preferred_actions_un_normalized;

        double heuristicValueTotal = 0;
		for (int a = 0; a < 7; a++) {
            weighted_preferred_actions_un_normalized.push_back(0);
			double reward = 0;
			bool meetPrecondition = false; 
			TurtleBotVisitLocations::CheckPreconditions(turtleBotVisitLocations_state, reward, meetPrecondition, a);
            if(meetPrecondition)
            {
                legal_actions_.push_back(a);
                double __heuristicValue; 
                TurtleBotVisitLocations::ComputePreferredActionValue(turtleBotVisitLocations_state, __heuristicValue, a);
                heuristicValueTotal += __heuristicValue;
                weighted_preferred_actions_un_normalized[a]=__heuristicValue;
            }
        }

        if(heuristicValueTotal > 0)
        {
            for (int a = 0; a < 7; a++) 
            {
                weighted_preferred_actions_.push_back(weighted_preferred_actions_un_normalized[a] / heuristicValueTotal);
            } 
        }
    }
};

/* ==============================================================================
 * TurtleBotVisitLocations class
 * ==============================================================================*/

TurtleBotVisitLocations::TurtleBotVisitLocations(){
	
}

int TurtleBotVisitLocations::NumActions() const {
	return ActionManager::actions.size();
}

double TurtleBotVisitLocations::ObsProb(OBS_TYPE obs, const State& state, int actionId) const {
	return 0.9;
}

 


std::default_random_engine TurtleBotVisitLocations::generator;


State* TurtleBotVisitLocations::CreateStartState(string type) const {
    TurtleBotVisitLocationsState* startState = memory_pool_.Allocate();
    TurtleBotVisitLocationsState& state = *startState;
    startState->tDiscreteLocationObjects.push_back(eL1);
    startState->tDiscreteLocationObjects.push_back(eL2);
    startState->tDiscreteLocationObjects.push_back(eL3);
    startState->tDiscreteLocationObjects.push_back(eL4);
    startState->tDiscreteLocationObjects.push_back(eL5);
    startState->tDiscreteLocationObjects.push_back(eL6);
    startState->tDiscreteLocationObjects.push_back(eL7);
    startState->tDiscreteLocationObjects.push_back(eL8);
    state.v1 = tVisitedLocation();
    state.v1.desc = eL1;
    state.v2 = tVisitedLocation();
    state.v2.desc = eL2;
    state.v3 = tVisitedLocation();
    state.v3.desc = eL3;
    state.v4 = tVisitedLocation();
    state.v4.desc = eL4;
    state.v5 = tVisitedLocation();
    state.v5.desc = eL5;
    state.v6 = tVisitedLocation();
    state.v6.desc = eL6;
    state.v7 = tVisitedLocation();
    state.v7.desc = eL7;
    state.l1 = tLocation();
    state.l1.x = -1.01606154442;
     state.l1.y = 0.660750925541;
     state.l1.z =-0.00454711914062;
     state.l1.desc = eL1;
    state.l2 = tLocation();
    state.l2.x = 0.00500533776358;
     state.l2.y = 0.640727937222;
     state.l2.z =-0.00143432617188;
     state.l2.desc = eL2;
    state.l3 = tLocation();
    state.l3.x = 0.986030161381;
     state.l3.y = 0.610693752766;
     state.l3.z =-0.00143432617188;
     state.l3.desc = eL3;
    state.l4 = tLocation();
    state.l4.x = 1.99708676338;
     state.l4.y = 0.620704889297;
     state.l4.z =-0.00143432617188;
     state.l4.desc = eL4;
    state.l5 = tLocation();
    state.l5.x = 1.9770655632;
     state.l5.y = -0.700796544552;
     state.l5.z =0.0025634765625;
     state.l5.desc = eL5;
    state.l6 = tLocation();
    state.l6.x = 0.989744365215;
     state.l6.y = -1.67847764492;
     state.l6.z =-0.00347900390625;
     state.l6.desc = eL6;
    state.l7 = tLocation();
    state.l7.x = -0.01501581911;
     state.l7.y = -1.49169492722;
     state.l7.z =0.00253295898438;
     state.l7.desc = eL7;
    state.l8 = tLocation();
    state.l8.x = -2.01710748672;
     state.l8.y = 0.570648550987;
     state.l8.z =-0.165466308594;
     state.l8.desc = eL8;
    state.robotL = tLocation();
    state.robotL = state.l8;
    startState->tVisitedLocationObjects.push_back(&(state.v1));
    startState->tVisitedLocationObjects.push_back(&(state.v2));
    startState->tVisitedLocationObjects.push_back(&(state.v3));
    startState->tVisitedLocationObjects.push_back(&(state.v4));
    startState->tVisitedLocationObjects.push_back(&(state.v5));
    startState->tVisitedLocationObjects.push_back(&(state.v6));
    startState->tVisitedLocationObjects.push_back(&(state.v7));
    startState->tLocationObjectsForActions["state.l1"] = (state.l1);
    startState->tLocationObjectsForActions["state.l2"] = (state.l2);
    startState->tLocationObjectsForActions["state.l3"] = (state.l3);
    startState->tLocationObjectsForActions["state.l4"] = (state.l4);
    startState->tLocationObjectsForActions["state.l5"] = (state.l5);
    startState->tLocationObjectsForActions["state.l6"] = (state.l6);
    startState->tLocationObjectsForActions["state.l7"] = (state.l7);
    startState->tLocationObjects.push_back(&(state.l8));
    startState->tLocationObjects.push_back(&(state.robotL));
    if (ActionManager::actions.size() == 0)
    {
        ActionManager::Init(const_cast <TurtleBotVisitLocationsState*> (startState));
    }
    return startState;
}



Belief* TurtleBotVisitLocations::InitialBelief(const State* start, string type) const {
	int N = TurtleBotVisitLocationsBelief::num_particles;
	vector<State*> particles(N);
	for (int i = 0; i < N; i++) {
        TurtleBotVisitLocationsBelief::currentInitParticleIndex = i;
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}
    TurtleBotVisitLocationsBelief::currentInitParticleIndex = -1;
	return new TurtleBotVisitLocationsBelief(particles, this);
}
 

 

POMCPPrior* TurtleBotVisitLocations::CreatePOMCPPrior(string name) const { 
		return new TurtleBotVisitLocationsPOMCPPrior(this);
}

void TurtleBotVisitLocations::PrintState(const State& state, ostream& ostr) const {
	const TurtleBotVisitLocationsState& farstate = static_cast<const TurtleBotVisitLocationsState&>(state);
	if (ostr)
		ostr << Prints::PrintState(farstate);
}

void TurtleBotVisitLocations::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {
	const TurtleBotVisitLocationsState& farstate = static_cast<const TurtleBotVisitLocationsState&>(state);
	
	ostr << observation <<endl;
}

void TurtleBotVisitLocations::PrintBelief(const Belief& belief, ostream& out) const {
	 out << "called PrintBelief(): b printed"<<endl;
		out << endl;
	
}

void TurtleBotVisitLocations::PrintAction(int actionId, ostream& out) const {
	out << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
}

State* TurtleBotVisitLocations::Allocate(int state_id, double weight) const {
	TurtleBotVisitLocationsState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* TurtleBotVisitLocations::Copy(const State* particle) const {
	TurtleBotVisitLocationsState* state = memory_pool_.Allocate();
	*state = *static_cast<const TurtleBotVisitLocationsState*>(particle);
	state->SetAllocated();

    state->tVisitedLocationObjects[0] = &(state->v1);
    state->tVisitedLocationObjects[1] = &(state->v2);
    state->tVisitedLocationObjects[2] = &(state->v3);
    state->tVisitedLocationObjects[3] = &(state->v4);
    state->tVisitedLocationObjects[4] = &(state->v5);
    state->tVisitedLocationObjects[5] = &(state->v6);
    state->tVisitedLocationObjects[6] = &(state->v7);
    state->tLocationObjects[0] = &(state->l8);
    state->tLocationObjects[1] = &(state->robotL);


	return state;
}

void TurtleBotVisitLocations::Free(State* particle) const {
	memory_pool_.Free(static_cast<TurtleBotVisitLocationsState*>(particle));
}

int TurtleBotVisitLocations::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

bool TurtleBotVisitLocations::Step(State& s_state__, double rand_num, int actionId, double& reward,
	OBS_TYPE& observation) const {
    reward = 0;
	bool isNextStateFinal = false;
	Random random(rand_num);
	int __moduleExecutionTime = -1;
	bool meetPrecondition = false;
	
	TurtleBotVisitLocationsState &state__ = static_cast<TurtleBotVisitLocationsState &>(s_state__);
	 logd << "[TurtleBotVisitLocations::Step] Selected Action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << "||State"<< Prints::PrintState(state__);
	CheckPreconditions(state__, reward, meetPrecondition, actionId);
	 
	State *s_state = Copy(&s_state__);
	TurtleBotVisitLocationsState &state = static_cast<TurtleBotVisitLocationsState &>(*s_state);

	
	SampleModuleExecutionTime(state__, rand_num, actionId, __moduleExecutionTime);

	ExtrinsicChangesDynamicModel(state, state__, rand_num, actionId, __moduleExecutionTime);

	State *s_state_ = Copy(&s_state__);
	TurtleBotVisitLocationsState &state_ = static_cast<TurtleBotVisitLocationsState &>(*s_state_);

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

void TurtleBotVisitLocations::CheckPreconditions(const TurtleBotVisitLocationsState& state, double &reward, bool &__meetPrecondition, int actionId)
    {
        ActionType &actType = ActionManager::actions[actionId]->actionType;
        __meetPrecondition = true;
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
                __meetPrecondition=false;
                for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
                {
                    tVisitedLocation loc = *(state.tVisitedLocationObjects[ind0]);
                    if (loc.desc== oDesiredLocation.desc)
                    {
                        __meetPrecondition= !loc.visited;
                    }
                }
                if(!__meetPrecondition) reward += -800;
            }
    }

void TurtleBotVisitLocations::ComputePreferredActionValue(const TurtleBotVisitLocationsState& state, double &__heuristicValue, int actionId)
    {
        __heuristicValue = 0;
        ActionType &actType = ActionManager::actions[actionId]->actionType;
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
            }
        __heuristicValue = __heuristicValue < 0 ? 0 : __heuristicValue;
    }

void TurtleBotVisitLocations::SampleModuleExecutionTime(const TurtleBotVisitLocationsState& farstate, double rand_num, int actionId, int &__moduleExecutionTime) const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    if(actType == navigateAction)
    {
    }
}

void TurtleBotVisitLocations::ExtrinsicChangesDynamicModel(const TurtleBotVisitLocationsState& state, TurtleBotVisitLocationsState& state_, double rand_num, int actionId, const int &__moduleExecutionTime)  const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
}

void TurtleBotVisitLocations::ModuleDynamicModel(const TurtleBotVisitLocationsState &state, const TurtleBotVisitLocationsState &state_, TurtleBotVisitLocationsState &state__, double rand_num, int actionId, double &__reward, OBS_TYPE &observation, const int &__moduleExecutionTime, const bool &__meetPrecondition) const
{
    std::hash<std::string> hasher;
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    observation = -1;
    int startObs = observation;
    std::string __moduleResponseStr = "NoStrResponse";
    OBS_TYPE &__moduleResponse = observation;
    if(actType == navigateAction)
    {
        NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
        tLocation &oDesiredLocation = act.oDesiredLocation;
        for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
        {
            tVisitedLocation &loc = *(state__.tVisitedLocationObjects[ind0]);
            if (loc.desc== oDesiredLocation.desc)
            {
                loc.visited = true;
            }
        }
        state__.robotL.x=oDesiredLocation.x;
        state__.robotL.y=oDesiredLocation.y;
        state__.robotL.z=oDesiredLocation.z;
        __moduleResponse=navigate_eSuccess;
        __reward=100-sqrt(pow(state.robotL.x-oDesiredLocation.x,2.0)+pow(state.robotL.y-oDesiredLocation.y,2.0))*10;
    }
    if(__moduleResponseStr != "NoStrResponse")
    {
        TurtleBotVisitLocationsResponseModuleAndTempEnums responseHash = (TurtleBotVisitLocationsResponseModuleAndTempEnums)hasher(__moduleResponseStr);
        enum_map_turtleBotVisitLocations::vecResponseEnumToString[responseHash] = __moduleResponseStr;
        enum_map_turtleBotVisitLocations::vecStringToResponseEnum[__moduleResponseStr] = responseHash;
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

bool TurtleBotVisitLocations::ProcessSpecialStates(TurtleBotVisitLocationsState &state, double &reward) const
{
    bool isFinalState = false;
    if(state.OneTimeRewardUsed[0])
    {
        if (state.v1.visited && state.v2.visited && state.v3.visited && state.v4.visited && state.v5.visited && state.v6.visited && state.v7.visited)
        {
            reward += 8000;
            isFinalState = true;
        }
    }
    return isFinalState;
}





std::string TurtleBotVisitLocations::PrintObs(int action, OBS_TYPE obs) const 
{
	return Prints::PrintObs(action, obs);
}

std::string TurtleBotVisitLocations::PrintStateStr(const State &state) const { return ""; };
}// namespace despot
