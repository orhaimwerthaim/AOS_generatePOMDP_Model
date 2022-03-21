#include "Bp_with_int_model.h"
#include <despot/core/pomdp.h> 
#include <stdlib.h>
#include <despot/solver/pomcp.h>
#include <sstream>
#include <despot/model_primitives/Bp_with_int_model/actionManager.h> 
#include <despot/model_primitives/Bp_with_int_model/enum_map_Bp_with_int_model.h> 
#include <despot/model_primitives/Bp_with_int_model/state.h> 
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
 *Bp_with_int_modelBelief class
 * ==============================================================================*/
int Bp_with_int_modelBelief::num_particles = 5000;
std::string Bp_with_int_modelBelief::beliefFromDB = "";
int Bp_with_int_modelBelief::currentInitParticleIndex = -1;

Bp_with_int_modelBelief::Bp_with_int_modelBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	Bp_with_int_model_(static_cast<const Bp_with_int_model*>(model)) {
}

	
 	std::string Bp_with_int_model::GetActionDescription(int actionId) const
	 {
		 return Prints::PrintActionDescription(ActionManager::actions[actionId]);
	 }

//void Bp_with_int_modelBelief::Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates) {
void Bp_with_int_modelBelief::Update(int actionId, OBS_TYPE obs) {
	history_.Add(actionId, obs);

	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = Bp_with_int_model_->Copy(particles_[cur]);
		bool terminal = Bp_with_int_model_->Step(*particle, Random::RANDOM.NextDouble(),
			actionId, reward, o);
 
		if (!terminal && o == obs) 
			{
				Bp_with_int_modelState &Bp_with_int_model_particle = static_cast<Bp_with_int_modelState &>(*particle);
				//if(!Globals::IsInternalSimulation() && updates.size() > 0)
				//{
				//	Bp_with_int_modelState::SetAnyValueLinks(&Bp_with_int_model_particle);
				//	map<std::string, bool>::iterator it;
				//	for (it = updates.begin(); it != updates.end(); it++)
				//	{
				//		*(Bp_with_int_model_particle.anyValueUpdateDic[it->first]) = it->second; 
				//	} 
				//}
				updated.push_back(particle);
		} else {
			Bp_with_int_model_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		Bp_with_int_model_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
 
}

/* ==============================================================================
 * Bp_with_int_modelPOMCPPrior class
 * ==============================================================================*/

class Bp_with_int_modelPOMCPPrior: public POMCPPrior {
private:
	const Bp_with_int_model* Bp_with_int_model_;

public:
	Bp_with_int_modelPOMCPPrior(const Bp_with_int_model* model) :
		POMCPPrior(model),
		Bp_with_int_model_(model) {
	}

	void ComputePreference(const State& state) {
		const Bp_with_int_modelState& Bp_with_int_model_state = static_cast<const Bp_with_int_modelState&>(state);
		weighted_preferred_actions_.clear();
        legal_actions_.clear();
		preferred_actions_.clear();
        std::vector<double> weighted_preferred_actions_un_normalized;

        double heuristicValueTotal = 0;
		for (int a = 0; a < 12; a++) {
            weighted_preferred_actions_un_normalized.push_back(0);
			double reward = 0;
			bool meetPrecondition = false; 
			Bp_with_int_model::CheckPreconditions(Bp_with_int_model_state, reward, meetPrecondition, a);
            if(meetPrecondition)
            {
                legal_actions_.push_back(a);
                double __heuristicValue; 
                Bp_with_int_model::ComputePreferredActionValue(Bp_with_int_model_state, __heuristicValue, a);
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
 * Bp_with_int_model class
 * ==============================================================================*/

Bp_with_int_model::Bp_with_int_model(){
	
}

int Bp_with_int_model::NumActions() const {
	return ActionManager::actions.size();
}

double Bp_with_int_model::ObsProb(OBS_TYPE obs, const State& state, int actionId) const {
	return 0.9;
}

 


std::default_random_engine Bp_with_int_model::generator;


State* Bp_with_int_model::CreateStartState(string type) const {
    Bp_with_int_modelState* startState = memory_pool_.Allocate();
    Bp_with_int_modelState& state = *startState;
    startState->tPushTypeObjects.push_back(SingleAgentPush);
    startState->tPushTypeObjects.push_back(JointPush);
    startState->tDirectionObjects.push_back(Up);
    startState->tDirectionObjects.push_back(Down);
    startState->tDirectionObjects.push_back(Left);
    startState->tDirectionObjects.push_back(Right);
    startState->tDirectionObjects.push_back(None);
    state.bTwoLocGoal = tLocation();
    state.bTwoLocGoal.x=2;
    state.bTwoLocGoal.y=1;
    state.bOneLocGoal = tLocation();
    state.bOneLocGoal.x=1;
    state.bOneLocGoal.y=1;
    state.agentOneLoc = tLocation();
    state.agentOneLoc.x=0;
    state.agentOneLoc.y=1;
    state.agentTwoLoc = tLocation();
    state.agentTwoLoc.x=1;
    state.agentTwoLoc.y=0;
    state.bOneLoc = tLocation();
    state.bOneLoc.x=0;
    state.bOneLoc.y=0;
    state.bTwoLoc = tLocation();
    state.bTwoLoc.x=0;
    state.bTwoLoc.y=0;
    state.isAgentOneTurn = true;
    state.ParamUp = Up;;
    state.ParamDown = Down;;
    state.ParamLeft = Left;;
    state.ParamRight = Right;;
    state.ParamSingleAgentPush = SingleAgentPush;;
    state.ParamJointPush = JointPush;;
    state.JointPushDirection = None;;
    state.MaxGridx = 2;;
    state.MaxGridy = 2;;
    startState->tLocationObjects.push_back(&(state.bTwoLocGoal));
    startState->tLocationObjects.push_back(&(state.bOneLocGoal));
    startState->tLocationObjects.push_back(&(state.agentOneLoc));
    startState->tLocationObjects.push_back(&(state.agentTwoLoc));
    startState->tLocationObjects.push_back(&(state.bOneLoc));
    startState->tLocationObjects.push_back(&(state.bTwoLoc));
    startState->tDirectionObjectsForActions["state.ParamUp"] = (state.ParamUp);
    startState->tDirectionObjectsForActions["state.ParamDown"] = (state.ParamDown);
    startState->tDirectionObjectsForActions["state.ParamLeft"] = (state.ParamLeft);
    startState->tDirectionObjectsForActions["state.ParamRight"] = (state.ParamRight);
    startState->tPushTypeObjectsForActions["state.ParamSingleAgentPush"] = (state.ParamSingleAgentPush);
    startState->tPushTypeObjectsForActions["state.ParamJointPush"] = (state.ParamJointPush);
    if (ActionManager::actions.size() == 0)
    {
        ActionManager::Init(const_cast <Bp_with_int_modelState*> (startState));
    }
    return startState;
}



Belief* Bp_with_int_model::InitialBelief(const State* start, string type) const {
	int N = Bp_with_int_modelBelief::num_particles;
	vector<State*> particles(N);
	for (int i = 0; i < N; i++) {
        Bp_with_int_modelBelief::currentInitParticleIndex = i;
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}
    Bp_with_int_modelBelief::currentInitParticleIndex = -1;
	return new Bp_with_int_modelBelief(particles, this);
}
 

 

POMCPPrior* Bp_with_int_model::CreatePOMCPPrior(string name) const { 
		return new Bp_with_int_modelPOMCPPrior(this);
}

void Bp_with_int_model::PrintState(const State& state, ostream& ostr) const {
	const Bp_with_int_modelState& farstate = static_cast<const Bp_with_int_modelState&>(state);
	if (ostr)
		ostr << Prints::PrintState(farstate);
}

void Bp_with_int_model::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {
	const Bp_with_int_modelState& farstate = static_cast<const Bp_with_int_modelState&>(state);
	
	ostr << observation <<endl;
}

void Bp_with_int_model::PrintBelief(const Belief& belief, ostream& out) const {
	 out << "called PrintBelief(): b printed"<<endl;
		out << endl;
	
}

void Bp_with_int_model::PrintAction(int actionId, ostream& out) const {
	out << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
}

State* Bp_with_int_model::Allocate(int state_id, double weight) const {
	Bp_with_int_modelState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* Bp_with_int_model::Copy(const State* particle) const {
	Bp_with_int_modelState* state = memory_pool_.Allocate();
	*state = *static_cast<const Bp_with_int_modelState*>(particle);
	state->SetAllocated();

    state->tLocationObjects[0] = &(state->bTwoLocGoal);
    state->tLocationObjects[1] = &(state->bOneLocGoal);
    state->tLocationObjects[2] = &(state->agentOneLoc);
    state->tLocationObjects[3] = &(state->agentTwoLoc);
    state->tLocationObjects[4] = &(state->bOneLoc);
    state->tLocationObjects[5] = &(state->bTwoLoc);


	return state;
}

void Bp_with_int_model::Free(State* particle) const {
	memory_pool_.Free(static_cast<Bp_with_int_modelState*>(particle));
}

int Bp_with_int_model::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

bool Bp_with_int_model::Step(State& s_state__, double rand_num, int actionId, double& reward,
	OBS_TYPE& observation) const {
    reward = 0;
	bool isNextStateFinal = false;
	Random random(rand_num);
	int __moduleExecutionTime = -1;
	bool meetPrecondition = false;
	
	Bp_with_int_modelState &state__ = static_cast<Bp_with_int_modelState &>(s_state__);
	 logd << "[Bp_with_int_model::Step] Selected Action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << "||State"<< Prints::PrintState(state__);
	CheckPreconditions(state__, reward, meetPrecondition, actionId);
	 
	State *s_state = Copy(&s_state__);
	Bp_with_int_modelState &state = static_cast<Bp_with_int_modelState &>(*s_state);

	
	SampleModuleExecutionTime(state__, rand_num, actionId, __moduleExecutionTime);

	ExtrinsicChangesDynamicModel(state, state__, rand_num, actionId, __moduleExecutionTime);

	State *s_state_ = Copy(&s_state__);
	Bp_with_int_modelState &state_ = static_cast<Bp_with_int_modelState &>(*s_state_);

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

void Bp_with_int_model::CheckPreconditions(const Bp_with_int_modelState& state, double &reward, bool &__meetPrecondition, int actionId)
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
                if(state.isAgentOneTurn&&state.agentOneLoc.y>=state.MaxGridy&&oDirection==Up)__meetPrecondition=false;
                if(state.isAgentOneTurn&&state.agentOneLoc.y<=0&&oDirection==Down)__meetPrecondition=false;
                if(state.isAgentOneTurn&&state.agentOneLoc.x<=0&&oDirection==Left)__meetPrecondition=false;
                if(state.isAgentOneTurn&&state.agentOneLoc.x>=state.MaxGridx&&oDirection==Right)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.y>=state.MaxGridy&&oDirection==Up)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.y<=0&&oDirection==Down)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.x<=0&&oDirection==Left)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.x>=state.MaxGridx&&oDirection==Right)__meetPrecondition=false;
                if(!__meetPrecondition) reward += -1;
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
                if(state.isAgentOneTurn&&state.agentOneLoc.y>=state.MaxGridy&&oDirection==Up)__meetPrecondition=false;
                if(state.isAgentOneTurn&&state.agentOneLoc.y<=0&&oDirection==Down)__meetPrecondition=false;
                if(state.isAgentOneTurn&&state.agentOneLoc.x<=0&&oDirection==Left)__meetPrecondition=false;
                if(state.isAgentOneTurn&&state.agentOneLoc.x>=state.MaxGridx&&oDirection==Right)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.y>=state.MaxGridy&&oDirection==Up)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.y<=0&&oDirection==Down)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.x<=0&&oDirection==Left)__meetPrecondition=false;
                if(!state.isAgentOneTurn&&state.agentTwoLoc.x>=state.MaxGridx&&oDirection==Right)__meetPrecondition=false;
                if(!__meetPrecondition) reward += -1;
            }
    }

void Bp_with_int_model::ComputePreferredActionValue(const Bp_with_int_modelState& state, double &__heuristicValue, int actionId)
    {
        __heuristicValue = 0;
        ActionType &actType = ActionManager::actions[actionId]->actionType;
            if(actType == pushAction)
            {
                PushActionDescription act = *(static_cast<PushActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
                tPushType &oIsJointPush = act.oIsJointPush;
                if(oIsJointPush==JointPush&&!state.isAgentOneTurn&&oDirection==state.JointPushDirection)__heuristicValue=100;
                if(__heuristicValue==0)__heuristicValue=1;
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tDirection &oDirection = act.oDirection;
                int agentX;
                agentX=state.isAgentOneTurn?state.agentOneLoc.x:state.agentTwoLoc.x;
                int agentY;
                agentY=state.isAgentOneTurn?state.agentOneLoc.y:state.agentTwoLoc.y;
                int nextAgentX;
                nextAgentX=agentX+(oDirection==Left?-1:(oDirection==Right?1:0));
                int nextAgentY;
                nextAgentY=agentY+(oDirection==Down?-1:(oDirection==Up?1:0));
                float bOneManhattanDistance;
                bOneManhattanDistance=std::abs(nextAgentX-state.bOneLoc.x)+std::abs(nextAgentY-state.bOneLoc.y);
                float bTwoManhattanDistance;
                bTwoManhattanDistance=(state.bTwoLoc.x==state.bTwoLocGoal.x&&state.bTwoLoc.y==state.bTwoLocGoal.y)?0:std::abs(nextAgentX-state.bTwoLoc.x)+std::abs(nextAgentY-state.bTwoLoc.y);
                bTwoManhattanDistance=bTwoManhattanDistance==0?0.1:bTwoManhattanDistance;
                bOneManhattanDistance=bOneManhattanDistance==0?0.1:bOneManhattanDistance;
                if((state.bOneLoc.x==state.bOneLocGoal.x&&state.bOneLoc.y==state.bOneLocGoal.y)||(state.bOneLoc.x==agentX&&state.bOneLoc.y==agentY))bOneManhattanDistance=-1;
                if((state.bTwoLoc.x==state.bTwoLocGoal.x&&state.bTwoLoc.y==state.bTwoLocGoal.y)||(state.bTwoLoc.x==agentX&&state.bTwoLoc.y==agentY))bTwoManhattanDistance=-1;
                __heuristicValue=std::max(1/bTwoManhattanDistance,1/bOneManhattanDistance);
            }
        __heuristicValue = __heuristicValue < 0 ? 0 : __heuristicValue;
    }

void Bp_with_int_model::SampleModuleExecutionTime(const Bp_with_int_modelState& farstate, double rand_num, int actionId, int &__moduleExecutionTime) const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    if(actType == pushAction)
    {
    }
    if(actType == navigateAction)
    {
    }
}

void Bp_with_int_model::ExtrinsicChangesDynamicModel(const Bp_with_int_modelState& state, Bp_with_int_modelState& state_, double rand_num, int actionId, const int &__moduleExecutionTime)  const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
        state_.isAgentOneTurn=!state.isAgentOneTurn;
}

void Bp_with_int_model::ModuleDynamicModel(const Bp_with_int_modelState &state, const Bp_with_int_modelState &state_, Bp_with_int_modelState &state__, double rand_num, int actionId, double &__reward, OBS_TYPE &observation, const int &__moduleExecutionTime, const bool &__meetPrecondition) const
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
        if(oDirection==Down&&__meetPrecondition){if(oIsJointPush==SingleAgentPush)state__.bOneLoc.y--;
        else if(!state.isAgentOneTurn)state__.bTwoLoc.y--;
        };
        if(oDirection==Up&&__meetPrecondition){if(oIsJointPush==SingleAgentPush)state__.bOneLoc.y++;
        else if(!state.isAgentOneTurn)state__.bTwoLoc.y++;
        };
        if(oDirection==Left&&__meetPrecondition){if(oIsJointPush==SingleAgentPush)state__.bOneLoc.x--;
        else if(!state.isAgentOneTurn)state__.bTwoLoc.x--;
        };
        if(oDirection==Right&&__meetPrecondition){if(oIsJointPush==SingleAgentPush)state__.bOneLoc.x++;
        else if(!state.isAgentOneTurn)state__.bTwoLoc.x++;
        };
        if(__meetPrecondition&&oIsJointPush==JointPush){if(state.isAgentOneTurn)state__.JointPushDirection=oDirection;
        };
        if(!(state.isAgentOneTurn&&oIsJointPush==JointPush))state__.JointPushDirection=None;
        __moduleResponse=push_eSuccess;
        __reward=-1;
    }
    if(actType == navigateAction)
    {
        NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
        tDirection &oDirection = act.oDirection;
        if(oDirection==Down&&__meetPrecondition){if(state.isAgentOneTurn)state__.agentOneLoc.y--;
        else state__.agentTwoLoc.y--;
        };
        if(oDirection==Up&&__meetPrecondition){if(state.isAgentOneTurn)state__.agentOneLoc.y++;
        else state__.agentTwoLoc.y++;
        };
        if(oDirection==Left&&__meetPrecondition){if(state.isAgentOneTurn)state__.agentOneLoc.x--;
        else state__.agentTwoLoc.x--;
        };
        if(oDirection==Right&&__meetPrecondition){if(state.isAgentOneTurn)state__.agentOneLoc.x++;
        else state__.agentTwoLoc.x++;
        };
        state__.JointPushDirection=None;
        __moduleResponse=navigate_eSuccess;
        __reward=-1;
    }
    if(__moduleResponseStr != "NoStrResponse")
    {
        Bp_with_int_modelResponseModuleAndTempEnums responseHash = (Bp_with_int_modelResponseModuleAndTempEnums)hasher(__moduleResponseStr);
        enum_map_Bp_with_int_model::vecResponseEnumToString[responseHash] = __moduleResponseStr;
        enum_map_Bp_with_int_model::vecStringToResponseEnum[__moduleResponseStr] = responseHash;
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

bool Bp_with_int_model::ProcessSpecialStates(Bp_with_int_modelState &state, double &reward) const
{
    bool isFinalState = false;
    if(state.OneTimeRewardUsed[0])
    {
        if (state.bOneLoc.x==state.bOneLocGoal.x && state.bOneLoc.y==state.bOneLocGoal.y && state.bTwoLoc.x==state.bTwoLocGoal.x && state.bTwoLoc.y==state.bTwoLocGoal.y)
        {
            reward += 8300;
            isFinalState = true;
        }
    }
    return isFinalState;
}





std::string Bp_with_int_model::PrintObs(int action, OBS_TYPE obs) const 
{
	return Prints::PrintObs(action, obs);
}

std::string Bp_with_int_model::PrintStateStr(const State &state) const { return ""; };
}// namespace despot
