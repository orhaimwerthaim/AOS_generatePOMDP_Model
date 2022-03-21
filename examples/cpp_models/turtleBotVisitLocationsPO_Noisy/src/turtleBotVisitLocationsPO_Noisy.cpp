#include "turtleBotVisitLocationsPO_Noisy.h"
#include <despot/core/pomdp.h> 
#include <stdlib.h>
#include <despot/solver/pomcp.h>
#include <sstream>
#include <despot/model_primitives/turtleBotVisitLocationsPO_Noisy/actionManager.h> 
#include <despot/model_primitives/turtleBotVisitLocationsPO_Noisy/enum_map_turtleBotVisitLocationsPO_Noisy.h> 
#include <despot/model_primitives/turtleBotVisitLocationsPO_Noisy/state.h> 
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
 *TurtleBotVisitLocationsPO_NoisyBelief class
 * ==============================================================================*/
int TurtleBotVisitLocationsPO_NoisyBelief::num_particles = 40234;
std::string TurtleBotVisitLocationsPO_NoisyBelief::beliefFromDB = "";
int TurtleBotVisitLocationsPO_NoisyBelief::currentInitParticleIndex = -1;

TurtleBotVisitLocationsPO_NoisyBelief::TurtleBotVisitLocationsPO_NoisyBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	turtleBotVisitLocationsPO_Noisy_(static_cast<const TurtleBotVisitLocationsPO_Noisy*>(model)) {
}

	
 	std::string TurtleBotVisitLocationsPO_Noisy::GetActionDescription(int actionId) const
	 {
		 return Prints::PrintActionDescription(ActionManager::actions[actionId]);
	 }

//void TurtleBotVisitLocationsPO_NoisyBelief::Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates) {
void TurtleBotVisitLocationsPO_NoisyBelief::Update(int actionId, OBS_TYPE obs) {
	history_.Add(actionId, obs);

	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = turtleBotVisitLocationsPO_Noisy_->Copy(particles_[cur]);
		bool terminal = turtleBotVisitLocationsPO_Noisy_->Step(*particle, Random::RANDOM.NextDouble(),
			actionId, reward, o);
 
		if (!terminal && o == obs) 
			{
				TurtleBotVisitLocationsPO_NoisyState &turtleBotVisitLocationsPO_Noisy_particle = static_cast<TurtleBotVisitLocationsPO_NoisyState &>(*particle);
				//if(!Globals::IsInternalSimulation() && updates.size() > 0)
				//{
				//	TurtleBotVisitLocationsPO_NoisyState::SetAnyValueLinks(&turtleBotVisitLocationsPO_Noisy_particle);
				//	map<std::string, bool>::iterator it;
				//	for (it = updates.begin(); it != updates.end(); it++)
				//	{
				//		*(turtleBotVisitLocationsPO_Noisy_particle.anyValueUpdateDic[it->first]) = it->second; 
				//	} 
				//}
				updated.push_back(particle);
		} else {
			turtleBotVisitLocationsPO_Noisy_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		turtleBotVisitLocationsPO_Noisy_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
 
}

/* ==============================================================================
 * TurtleBotVisitLocationsPO_NoisyPOMCPPrior class
 * ==============================================================================*/

class TurtleBotVisitLocationsPO_NoisyPOMCPPrior: public POMCPPrior {
private:
	const TurtleBotVisitLocationsPO_Noisy* turtleBotVisitLocationsPO_Noisy_;

public:
	TurtleBotVisitLocationsPO_NoisyPOMCPPrior(const TurtleBotVisitLocationsPO_Noisy* model) :
		POMCPPrior(model),
		turtleBotVisitLocationsPO_Noisy_(model) {
	}

	void ComputePreference(const State& state) {
		const TurtleBotVisitLocationsPO_NoisyState& turtleBotVisitLocationsPO_Noisy_state = static_cast<const TurtleBotVisitLocationsPO_NoisyState&>(state);
		weighted_preferred_actions_.clear();
        legal_actions_.clear();
		preferred_actions_.clear();
        std::vector<double> weighted_preferred_actions_un_normalized;

        double heuristicValueTotal = 0;
		for (int a = 0; a < 6; a++) {
            weighted_preferred_actions_un_normalized.push_back(0);
			double reward = 0;
			bool meetPrecondition = false; 
			TurtleBotVisitLocationsPO_Noisy::CheckPreconditions(turtleBotVisitLocationsPO_Noisy_state, reward, meetPrecondition, a);
            if(meetPrecondition)
            {
                legal_actions_.push_back(a);
                double __heuristicValue; 
                TurtleBotVisitLocationsPO_Noisy::ComputePreferredActionValue(turtleBotVisitLocationsPO_Noisy_state, __heuristicValue, a);
                heuristicValueTotal += __heuristicValue;
                weighted_preferred_actions_un_normalized[a]=__heuristicValue;
            }
        }

        if(heuristicValueTotal > 0)
        {
            for (int a = 0; a < 6; a++) 
            {
                weighted_preferred_actions_.push_back(weighted_preferred_actions_un_normalized[a] / heuristicValueTotal);
            } 
        }
    }
};

/* ==============================================================================
 * TurtleBotVisitLocationsPO_Noisy class
 * ==============================================================================*/

TurtleBotVisitLocationsPO_Noisy::TurtleBotVisitLocationsPO_Noisy(){
	
}

int TurtleBotVisitLocationsPO_Noisy::NumActions() const {
	return ActionManager::actions.size();
}

double TurtleBotVisitLocationsPO_Noisy::ObsProb(OBS_TYPE obs, const State& state, int actionId) const {
	return 0.9;
}

 


std::default_random_engine TurtleBotVisitLocationsPO_Noisy::generator;

std::normal_distribution<double>  TurtleBotVisitLocationsPO_Noisy::navigate_normal_dist1(40000,10000); //AOS.SampleNormal(40000,10000)

State* TurtleBotVisitLocationsPO_Noisy::CreateStartState(string type) const {
    TurtleBotVisitLocationsPO_NoisyState* startState = memory_pool_.Allocate();
    TurtleBotVisitLocationsPO_NoisyState& state = *startState;
    startState->tDiscreteLocationObjects.push_back(eL1);
    startState->tDiscreteLocationObjects.push_back(eL2);
    startState->tDiscreteLocationObjects.push_back(eL3);
    startState->tDiscreteLocationObjects.push_back(eL4);
    startState->tDiscreteLocationObjects.push_back(eL8);
    startState->tDiscreteLocationObjects.push_back(eL9);
    startState->tDiscreteLocationObjects.push_back(eLInitRobLoc);
    state.numberOfCompletePatrols = 0;
    state.v1 = tVisitedLocation();
    state.v1.desc = eL1;
    state.v2 = tVisitedLocation();
    state.v2.desc = eL2;
    state.v3 = tVisitedLocation();
    state.v3.desc = eL3;
    state.v4 = tVisitedLocation();
    state.v4.desc = eL4;
    state.v8 = tVisitedLocation();
    state.v8.desc = eL8;
    state.v9 = tVisitedLocation();
    state.v9.desc = eL9;
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
    state.l8 = tLocation();
    state.l8.x = 0.174183383584;
     state.l8.y = -3.19066166878;
     state.l8.z =0.00250244140625;
     state.l8.desc = eL8;
    state.l9 = tLocation();
    state.l9.x = 4.76382398605;
     state.l9.y = 2.90320682526;
     state.l9.z =0.00247192382812;
     state.l9.desc = eL9;
    state.lInitRobLoc = tLocation();
    state.lInitRobLoc.x = -2.01710748672;
     state.lInitRobLoc.y = 0.570648550987;
     state.lInitRobLoc.z =-0.165466308594;
     state.lInitRobLoc.desc = eLInitRobLoc;
    state.robotL = tLocation();
    state.robotL = state.lInitRobLoc;
    startState->tVisitedLocationObjects.push_back(&(state.v1));
    startState->tVisitedLocationObjects.push_back(&(state.v2));
    startState->tVisitedLocationObjects.push_back(&(state.v3));
    startState->tVisitedLocationObjects.push_back(&(state.v4));
    startState->tVisitedLocationObjects.push_back(&(state.v8));
    startState->tVisitedLocationObjects.push_back(&(state.v9));
    startState->tLocationObjectsForActions["state.l1"] = (state.l1);
    startState->tLocationObjectsForActions["state.l2"] = (state.l2);
    startState->tLocationObjectsForActions["state.l3"] = (state.l3);
    startState->tLocationObjectsForActions["state.l4"] = (state.l4);
    startState->tLocationObjectsForActions["state.l8"] = (state.l8);
    startState->tLocationObjectsForActions["state.l9"] = (state.l9);
    startState->tLocationObjects.push_back(&(state.lInitRobLoc));
    startState->tLocationObjects.push_back(&(state.robotL));
    for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
    {
        tVisitedLocation &loc = *(state.tVisitedLocationObjects[ind0]);
        if (true)
        {
            loc.reachable = AOSUtils::Bernoulli(0.66);
        }
    }
    if (ActionManager::actions.size() == 0)
    {
        ActionManager::Init(const_cast <TurtleBotVisitLocationsPO_NoisyState*> (startState));
    }
    return startState;
}



Belief* TurtleBotVisitLocationsPO_Noisy::InitialBelief(const State* start, string type) const {
	int N = TurtleBotVisitLocationsPO_NoisyBelief::num_particles;
	vector<State*> particles(N);
	for (int i = 0; i < N; i++) {
        TurtleBotVisitLocationsPO_NoisyBelief::currentInitParticleIndex = i;
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}
    TurtleBotVisitLocationsPO_NoisyBelief::currentInitParticleIndex = -1;
	return new TurtleBotVisitLocationsPO_NoisyBelief(particles, this);
}
 

 

POMCPPrior* TurtleBotVisitLocationsPO_Noisy::CreatePOMCPPrior(string name) const { 
		return new TurtleBotVisitLocationsPO_NoisyPOMCPPrior(this);
}

void TurtleBotVisitLocationsPO_Noisy::PrintState(const State& state, ostream& ostr) const {
	const TurtleBotVisitLocationsPO_NoisyState& farstate = static_cast<const TurtleBotVisitLocationsPO_NoisyState&>(state);
	if (ostr)
		ostr << Prints::PrintState(farstate);
}

void TurtleBotVisitLocationsPO_Noisy::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {
	const TurtleBotVisitLocationsPO_NoisyState& farstate = static_cast<const TurtleBotVisitLocationsPO_NoisyState&>(state);
	
	ostr << observation <<endl;
}

void TurtleBotVisitLocationsPO_Noisy::PrintBelief(const Belief& belief, ostream& out) const {
	 out << "called PrintBelief(): b printed"<<endl;
		out << endl;
	
}

void TurtleBotVisitLocationsPO_Noisy::PrintAction(int actionId, ostream& out) const {
	out << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
}

State* TurtleBotVisitLocationsPO_Noisy::Allocate(int state_id, double weight) const {
	TurtleBotVisitLocationsPO_NoisyState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* TurtleBotVisitLocationsPO_Noisy::Copy(const State* particle) const {
	TurtleBotVisitLocationsPO_NoisyState* state = memory_pool_.Allocate();
	*state = *static_cast<const TurtleBotVisitLocationsPO_NoisyState*>(particle);
	state->SetAllocated();

    state->tVisitedLocationObjects[0] = &(state->v1);
    state->tVisitedLocationObjects[1] = &(state->v2);
    state->tVisitedLocationObjects[2] = &(state->v3);
    state->tVisitedLocationObjects[3] = &(state->v4);
    state->tVisitedLocationObjects[4] = &(state->v8);
    state->tVisitedLocationObjects[5] = &(state->v9);
    state->tLocationObjects[0] = &(state->lInitRobLoc);
    state->tLocationObjects[1] = &(state->robotL);


	return state;
}

void TurtleBotVisitLocationsPO_Noisy::Free(State* particle) const {
	memory_pool_.Free(static_cast<TurtleBotVisitLocationsPO_NoisyState*>(particle));
}

int TurtleBotVisitLocationsPO_Noisy::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

bool TurtleBotVisitLocationsPO_Noisy::Step(State& s_state__, double rand_num, int actionId, double& reward,
	OBS_TYPE& observation) const {
    reward = 0;
	bool isNextStateFinal = false;
	Random random(rand_num);
	int __moduleExecutionTime = -1;
	bool meetPrecondition = false;
	
	TurtleBotVisitLocationsPO_NoisyState &state__ = static_cast<TurtleBotVisitLocationsPO_NoisyState &>(s_state__);
	 logd << "[TurtleBotVisitLocationsPO_Noisy::Step] Selected Action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << "||State"<< Prints::PrintState(state__);
	CheckPreconditions(state__, reward, meetPrecondition, actionId);
	 
	State *s_state = Copy(&s_state__);
	TurtleBotVisitLocationsPO_NoisyState &state = static_cast<TurtleBotVisitLocationsPO_NoisyState &>(*s_state);

	
	SampleModuleExecutionTime(state__, rand_num, actionId, __moduleExecutionTime);

	ExtrinsicChangesDynamicModel(state, state__, rand_num, actionId, __moduleExecutionTime);

	State *s_state_ = Copy(&s_state__);
	TurtleBotVisitLocationsPO_NoisyState &state_ = static_cast<TurtleBotVisitLocationsPO_NoisyState &>(*s_state_);

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

void TurtleBotVisitLocationsPO_Noisy::CheckPreconditions(const TurtleBotVisitLocationsPO_NoisyState& state, double &reward, bool &__meetPrecondition, int actionId)
    {
        ActionType &actType = ActionManager::actions[actionId]->actionType;
        __meetPrecondition = true;
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
                __meetPrecondition=true;
                for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
                {
                    tVisitedLocation loc = *(state.tVisitedLocationObjects[ind0]);
                    if (loc.desc== oDesiredLocation.desc)
                    {
                        __meetPrecondition= loc.reachable && loc.visitCount == state.numberOfCompletePatrols;
                    }
                }
                if(!__meetPrecondition) reward += 0;
            }
    }

void TurtleBotVisitLocationsPO_Noisy::ComputePreferredActionValue(const TurtleBotVisitLocationsPO_NoisyState& state, double &__heuristicValue, int actionId)
    {
        __heuristicValue = 0;
        ActionType &actType = ActionManager::actions[actionId]->actionType;
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
                __heuristicValue=(state.robotL.x-oDesiredLocation.x+state.robotL.y-oDesiredLocation.y==0)?0:abs(log(sqrt(pow(state.robotL.x-oDesiredLocation.x,2.0)+pow(state.robotL.y-oDesiredLocation.y,2.0))/5));
            }
        __heuristicValue = __heuristicValue < 0 ? 0 : __heuristicValue;
    }

void TurtleBotVisitLocationsPO_Noisy::SampleModuleExecutionTime(const TurtleBotVisitLocationsPO_NoisyState& farstate, double rand_num, int actionId, int &__moduleExecutionTime) const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    if(actType == navigateAction)
    {
        __moduleExecutionTime=TurtleBotVisitLocationsPO_Noisy::navigate_normal_dist1(TurtleBotVisitLocationsPO_Noisy::generator);
    }
}

void TurtleBotVisitLocationsPO_Noisy::ExtrinsicChangesDynamicModel(const TurtleBotVisitLocationsPO_NoisyState& state, TurtleBotVisitLocationsPO_NoisyState& state_, double rand_num, int actionId, const int &__moduleExecutionTime)  const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
}

void TurtleBotVisitLocationsPO_Noisy::ModuleDynamicModel(const TurtleBotVisitLocationsPO_NoisyState &state, const TurtleBotVisitLocationsPO_NoisyState &state_, TurtleBotVisitLocationsPO_NoisyState &state__, double rand_num, int actionId, double &__reward, OBS_TYPE &observation, const int &__moduleExecutionTime, const bool &__meetPrecondition) const
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
        bool isValidVisit;
        bool isLocationReachable;
        for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
        {
            tVisitedLocation loc = *(state.tVisitedLocationObjects[ind0]);
            if (loc.desc== oDesiredLocation.desc)
            {
                isValidVisit= loc.reachable && loc.visitCount == state.numberOfCompletePatrols && state__.robotL.desc != oDesiredLocation.desc;
                 isLocationReachable = loc.reachable;
            }
        }
        for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
        {
            tVisitedLocation &loc = *(state__.tVisitedLocationObjects[ind0]);
            if (loc.desc== oDesiredLocation.desc && isLocationReachable && state__.robotL.desc != oDesiredLocation.desc)
            {
                if(isValidVisit){loc.visitCount++;
                };
            }
        }
        if(isLocationReachable){state__.robotL.x=oDesiredLocation.x;
        state__.robotL.y=oDesiredLocation.y;
        state__.robotL.z=oDesiredLocation.z;
        state__.robotL.desc=oDesiredLocation.desc;
        };
        __moduleResponse=isLocationReachable&&AOSUtils::Bernoulli(0.9)?navigate_eSuccess:navigate_eFailed;
        state__.numberOfCompletePatrols=10;
        for (int ind0 = 0; ind0 < state.tVisitedLocationObjects.size(); ind0++)
        {
            tVisitedLocation &loc = *(state__.tVisitedLocationObjects[ind0]);
            if (loc.reachable)
            {
                if(state__.numberOfCompletePatrols > loc.visitCount){state__.numberOfCompletePatrols=loc.visitCount;
                };
            }
        }
        __reward=!isValidVisit?-100:-pow(sqrt(pow(state.robotL.x-oDesiredLocation.x,2.0)+pow(state.robotL.y-oDesiredLocation.y,2.0)),2.0)*3;
    }
    __moduleResponse = __moduleResponseStr == "NoStrResponse" ? __moduleResponse : (int)hasher(__moduleResponseStr);
    if(startObs == __moduleResponse)
    {
    stringstream ss;
    ss << "Observation/__moduleResponse Not initialized!!! on action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
    loge << ss.str() << endl;
    throw 1;
    }
}

bool TurtleBotVisitLocationsPO_Noisy::ProcessSpecialStates(TurtleBotVisitLocationsPO_NoisyState &state, double &reward) const
{
    bool isFinalState = false;
    if(state.OneTimeRewardUsed[0])
    {
        if (state.numberOfCompletePatrols == 1)
        {
            state.OneTimeRewardUsed[0] = false;
            reward += 4000;
        }
    }
    if(state.OneTimeRewardUsed[1])
    {
        if (state.numberOfCompletePatrols == 2)
        {
            reward += 7000;
            isFinalState = true;
        }
    }
    return isFinalState;
}





std::string TurtleBotVisitLocationsPO_Noisy::PrintObs(int action, OBS_TYPE obs) const 
{
	return Prints::PrintObs(action, obs);
}

std::string TurtleBotVisitLocationsPO_Noisy::PrintStateStr(const State &state) const { return ""; };
}// namespace despot
