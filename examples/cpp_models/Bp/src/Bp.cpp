#include "Bp.h"
#include <despot/core/pomdp.h> 
#include <stdlib.h>
#include <despot/solver/pomcp.h>
#include <sstream>
#include <despot/model_primitives/Bp/actionManager.h> 
#include <despot/model_primitives/Bp/enum_map_Bp.h> 
#include <despot/model_primitives/Bp/state.h> 
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
 *BpBelief class
 * ==============================================================================*/
int BpBelief::num_particles = 5000;
std::string BpBelief::beliefFromDB = "";
int BpBelief::currentInitParticleIndex = -1;

BpBelief::BpBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	Bp_(static_cast<const Bp*>(model)) {
}

	
 	std::string Bp::GetActionDescription(int actionId) const
	 {
		 return Prints::PrintActionDescription(ActionManager::actions[actionId]);
	 }

//void BpBelief::Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates) {
void BpBelief::Update(int actionId, OBS_TYPE obs) {
	history_.Add(actionId, obs);

	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = Bp_->Copy(particles_[cur]);
		bool terminal = Bp_->Step(*particle, Random::RANDOM.NextDouble(),
			actionId, reward, o);
 
		if (!terminal && o == obs) 
			{
				BpState &Bp_particle = static_cast<BpState &>(*particle);
				//if(!Globals::IsInternalSimulation() && updates.size() > 0)
				//{
				//	BpState::SetAnyValueLinks(&Bp_particle);
				//	map<std::string, bool>::iterator it;
				//	for (it = updates.begin(); it != updates.end(); it++)
				//	{
				//		*(Bp_particle.anyValueUpdateDic[it->first]) = it->second; 
				//	} 
				//}
				updated.push_back(particle);
		} else {
			Bp_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		Bp_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
 
}

/* ==============================================================================
 * BpPOMCPPrior class
 * ==============================================================================*/

class BpPOMCPPrior: public POMCPPrior {
private:
	const Bp* Bp_;

public:
	BpPOMCPPrior(const Bp* model) :
		POMCPPrior(model),
		Bp_(model) {
	}

	void ComputePreference(const State& state) {
		const BpState& Bp_state = static_cast<const BpState&>(state);
		weighted_preferred_actions_.clear();
        legal_actions_.clear();
		preferred_actions_.clear();
        std::vector<double> weighted_preferred_actions_un_normalized;

        double heuristicValueTotal = 0;
		for (int a = 0; a < 12; a++) {
            weighted_preferred_actions_un_normalized.push_back(0);
			double reward = 0;
			bool meetPrecondition = false; 
			Bp::CheckPreconditions(Bp_state, reward, meetPrecondition, a);
            if(meetPrecondition)
            {
                legal_actions_.push_back(a);
                double __heuristicValue; 
                Bp::ComputePreferredActionValue(Bp_state, __heuristicValue, a);
                heuristicValueTotal += __heuristicValue;
                weighted_preferred_actions_un_normalized[a]=__heuristicValue;
            }
        }

        if(heuristicValueTotal > 0)
        {
            for (int a = 0; a < 12; a++) 
            {
                weighted_preferred_actions_.push_back(weighted_preferred_actions_un_normalized[a] / heuristicValueTotal);
            } 
        }
    }
};

/* ==============================================================================
 * Bp class
 * ==============================================================================*/

Bp::Bp(){
	
}

int Bp::NumActions() const {
	return ActionManager::actions.size();
}

double Bp::ObsProb(OBS_TYPE obs, const State& state, int actionId) const {
	return 0.9;
}

 


std::default_random_engine Bp::generator;

std::discrete_distribution<> Bp::Environment_discrete_dist1{0.5,0,0,0}; //AOS.SampleDiscrete(tCell,{0.5,0,0,0})

State* Bp::CreateStartState(string type) const {
    BpState* startState = memory_pool_.Allocate();
    BpState& state = *startState;
    startState->tPushTypeObjects.push_back(SingleAgentPush);
    startState->tPushTypeObjects.push_back(JointPush);
    startState->tDirectionObjects.push_back(Up);
    startState->tDirectionObjects.push_back(Down);
    startState->tDirectionObjects.push_back(Left);
    startState->tDirectionObjects.push_back(Right);
    startState->tDirectionObjects.push_back(None);
    startState->tCellObjects.push_back(L00);
    startState->tCellObjects.push_back(L10);
    startState->tCellObjects.push_back(L01);
    startState->tCellObjects.push_back(L11);
    startState->tCellObjects.push_back(OUT);
    state.agentOneLoc = L10;
    state.agentTwoLoc = L01;
    state.bOneLoc = L00;
    state.bTwoLoc = L00;
    state.isAgentOneTurn = true;
    state.ParamUp = Up;;
    state.ParamDown = Down;;
    state.ParamLeft = Left;;
    state.ParamRight = Right;;
    state.ParamSingleAgentPush = SingleAgentPush;;
    state.ParamJointPush = JointPush;;
    state.JointPushDirection = None;;
    startState->tDirectionObjectsForActions["state.ParamUp"] = (state.ParamUp);
    startState->tDirectionObjectsForActions["state.ParamDown"] = (state.ParamDown);
    startState->tDirectionObjectsForActions["state.ParamLeft"] = (state.ParamLeft);
    startState->tDirectionObjectsForActions["state.ParamRight"] = (state.ParamRight);
    startState->tPushTypeObjectsForActions["state.ParamSingleAgentPush"] = (state.ParamSingleAgentPush);
    startState->tPushTypeObjectsForActions["state.ParamJointPush"] = (state.ParamJointPush);
    state.bOneLoc=state.tCellObjects[Environment_discrete_dist1(generator)];;
    state.bTwoLoc=state.tCellObjects[Environment_discrete_dist1(generator)];;
    if (ActionManager::actions.size() == 0)
    {
        ActionManager::Init(const_cast <BpState*> (startState));
    }
    return startState;
}



Belief* Bp::InitialBelief(const State* start, string type) const {
	int N = BpBelief::num_particles;
	vector<State*> particles(N);
	for (int i = 0; i < N; i++) {
        BpBelief::currentInitParticleIndex = i;
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}
    BpBelief::currentInitParticleIndex = -1;
	return new BpBelief(particles, this);
}
 

 

POMCPPrior* Bp::CreatePOMCPPrior(string name) const { 
		return new BpPOMCPPrior(this);
}

void Bp::PrintState(const State& state, ostream& ostr) const {
	const BpState& farstate = static_cast<const BpState&>(state);
	if (ostr)
		ostr << Prints::PrintState(farstate);
}

void Bp::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {
	const BpState& farstate = static_cast<const BpState&>(state);
	
	ostr << observation <<endl;
}

void Bp::PrintBelief(const Belief& belief, ostream& out) const {
	 out << "called PrintBelief(): b printed"<<endl;
		out << endl;
	
}

void Bp::PrintAction(int actionId, ostream& out) const {
	out << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
}

State* Bp::Allocate(int state_id, double weight) const {
	BpState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* Bp::Copy(const State* particle) const {
	BpState* state = memory_pool_.Allocate();
	*state = *static_cast<const BpState*>(particle);
	state->SetAllocated();



	return state;
}

void Bp::Free(State* particle) const {
	memory_pool_.Free(static_cast<BpState*>(particle));
}

int Bp::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

bool Bp::Step(State& s_state__, double rand_num, int actionId, double& reward,
	OBS_TYPE& observation) const {
    reward = 0;
	bool isNextStateFinal = false;
	Random random(rand_num);
	int __moduleExecutionTime = -1;
	bool meetPrecondition = false;
	
	BpState &state__ = static_cast<BpState &>(s_state__);
	 logd << "[Bp::Step] Selected Action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << "||State"<< Prints::PrintState(state__);
	CheckPreconditions(state__, reward, meetPrecondition, actionId);
	 
	State *s_state = Copy(&s_state__);
	BpState &state = static_cast<BpState &>(*s_state);

	
	SampleModuleExecutionTime(state__, rand_num, actionId, __moduleExecutionTime);

	ExtrinsicChangesDynamicModel(state, state__, rand_num, actionId, __moduleExecutionTime);

	State *s_state_ = Copy(&s_state__);
	BpState &state_ = static_cast<BpState &>(*s_state_);

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

void Bp::CheckPreconditions(const BpState& state, double &reward, bool &__meetPrecondition, int actionId)
    {
        ActionType &actType = ActionManager::actions[actionId]->actionType;
        __meetPrecondition = true;
            if(actType == pushAction)
            {
                PushActionDescription act = *(static_cast<PushActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
                tPushType &oIsJointPush = act.oIsJointPush;
                if(oIsJointPush==JointPush&&!state.isAgentOneTurn&&oDirection!=state.JointPushDirection)__meetPrecondition=false;
                if(oIsJointPush==JointPush&&state.agentOneLoc!=state.bTwoLoc)__meetPrecondition=false;
                if(oIsJointPush==SingleAgentPush&&((state.isAgentOneTurn&&state.agentOneLoc!=state.bOneLoc)||(!state.isAgentOneTurn&&state.agentTwoLoc!=state.bOneLoc)))__meetPrecondition=false;
                if(oIsJointPush==JointPush&&state.agentOneLoc!=state.agentTwoLoc)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L10||state.agentOneLoc==L11)&&oDirection==Up)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L00||state.agentOneLoc==L01)&&oDirection==Down)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L10||state.agentOneLoc==L00)&&oDirection==Left)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L01||state.agentOneLoc==L11)&&oDirection==Right)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L10||state.agentTwoLoc==L11)&&oDirection==Up)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L00||state.agentTwoLoc==L01)&&oDirection==Down)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L10||state.agentTwoLoc==L00)&&oDirection==Left)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L01||state.agentTwoLoc==L11)&&oDirection==Right)__meetPrecondition=false;
                if(!__meetPrecondition) reward += -1;
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L10||state.agentOneLoc==L11)&&oDirection==Up)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L00||state.agentOneLoc==L01)&&oDirection==Down)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L10||state.agentOneLoc==L00)&&oDirection==Left)__meetPrecondition=false;
                if(state.isAgentOneTurn&&(state.agentOneLoc==L01||state.agentOneLoc==L11)&&oDirection==Right)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L10||state.agentTwoLoc==L11)&&oDirection==Up)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L00||state.agentTwoLoc==L01)&&oDirection==Down)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L10||state.agentTwoLoc==L00)&&oDirection==Left)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&(state.agentTwoLoc==L01||state.agentTwoLoc==L11)&&oDirection==Right)__meetPrecondition=false;
                if(!__meetPrecondition) reward += -1;
            }
    }

void Bp::ComputePreferredActionValue(const BpState& state, double &__heuristicValue, int actionId)
    {
        __heuristicValue = 0;
        ActionType &actType = ActionManager::actions[actionId]->actionType;
            if(actType == pushAction)
            {
                PushActionDescription act = *(static_cast<PushActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
                tPushType &oIsJointPush = act.oIsJointPush;
                if(oIsJointPush==JointPush&&!state.isAgentOneTurn&&oDirection==state.JointPushDirection)__heuristicValue=100;
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
            }
        __heuristicValue = __heuristicValue < 0 ? 0 : __heuristicValue;
    }

void Bp::SampleModuleExecutionTime(const BpState& farstate, double rand_num, int actionId, int &__moduleExecutionTime) const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    if(actType == pushAction)
    {
    }
    if(actType == navigateAction)
    {
    }
}

void Bp::ExtrinsicChangesDynamicModel(const BpState& state, BpState& state_, double rand_num, int actionId, const int &__moduleExecutionTime)  const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
}

void Bp::ModuleDynamicModel(const BpState &state, const BpState &state_, BpState &state__, double rand_num, int actionId, double &__reward, OBS_TYPE &observation, const int &__moduleExecutionTime, const bool &__meetPrecondition) const
{
    std::hash<std::string> hasher;
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    observation = -1;
    int startObs = observation;
    std::string __moduleResponseStr = "NoStrResponse";
    OBS_TYPE &__moduleResponse = observation;
    if(actType == pushAction)
    {
        PushActionDescription act = *(static_cast<PushActionDescription *>(ActionManager::actions[actionId]));
        tDirection &oDirection = act.oDirection;
        tPushType &oIsJointPush = act.oIsJointPush;
        tCell agentLocation;
        agentLocation=state.isAgentOneTurn?state.agentOneLoc:state.agentTwoLoc;
        tCell boxNextLocation;
        boxNextLocation=agentLocation;
        if(oDirection==Down){boxNextLocation=agentLocation==L10?L00:agentLocation==L11?L01:OUT;
        };
        if(oDirection==Up){boxNextLocation=agentLocation==L00?L10:agentLocation==L01?L11:OUT;
        };
        if(oDirection==Left){boxNextLocation=agentLocation==L01?L00:agentLocation==L11?L10:OUT;
        };
        if(oDirection==Right){boxNextLocation=agentLocation==L10?L11:agentLocation==L00?L01:OUT;
        };
        if(__meetPrecondition&&oIsJointPush==SingleAgentPush){state__.bOneLoc=boxNextLocation;
        };
        if(__meetPrecondition&&oIsJointPush==JointPush){if(state.isAgentOneTurn)state__.JointPushDirection=oDirection;
        else {state__.bTwoLoc=boxNextLocation;
        }};
        state__.isAgentOneTurn=!state.isAgentOneTurn;
        if(!(state.isAgentOneTurn&&oIsJointPush==JointPush))state__.JointPushDirection=None;
        __moduleResponse=push_eSuccess;
        __reward=-1;
    }
    if(actType == navigateAction)
    {
        NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
        tDirection &oDirection = act.oDirection;
        tCell agentLocation;
        agentLocation=state.isAgentOneTurn?state.agentOneLoc:state.agentTwoLoc;
        tCell agentNextLocation;
        agentNextLocation=agentLocation;
        if(oDirection==Down){agentNextLocation=agentLocation==L10?L00:agentLocation==L11?L01:agentLocation;
        };
        if(oDirection==Up){agentNextLocation=agentLocation==L00?L10:agentLocation==L01?L11:agentLocation;
        };
        if(oDirection==Left){agentNextLocation=agentLocation==L01?L00:agentLocation==L11?L10:agentLocation;
        };
        if(oDirection==Right){agentNextLocation=agentLocation==L10?L11:agentLocation==L00?L01:agentLocation;
        };
        if(state.isAgentOneTurn){state__.agentOneLoc=agentNextLocation;
        }else {state__.agentTwoLoc=agentNextLocation;
        };
        state__.isAgentOneTurn=!state.isAgentOneTurn;
        state__.JointPushDirection=None;
        __moduleResponse=navigate_eSuccess;
        __reward=-1;
    }
    if(__moduleResponseStr != "NoStrResponse")
    {
        BpResponseModuleAndTempEnums responseHash = (BpResponseModuleAndTempEnums)hasher(__moduleResponseStr);
        enum_map_Bp::vecResponseEnumToString[responseHash] = __moduleResponseStr;
        enum_map_Bp::vecStringToResponseEnum[__moduleResponseStr] = responseHash;
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

bool Bp::ProcessSpecialStates(BpState &state, double &reward) const
{
    bool isFinalState = false;
    if(state.OneTimeRewardUsed[0])
    {
        if (state.bOneLoc == L11 && state.bTwoLoc == L11)
        {
            reward += 1300;
            isFinalState = true;
        }
    }
    return isFinalState;
}





std::string Bp::PrintObs(int action, OBS_TYPE obs) const 
{
	return Prints::PrintObs(action, obs);
}

std::string Bp::PrintStateStr(const State &state) const { return ""; };
}// namespace despot
