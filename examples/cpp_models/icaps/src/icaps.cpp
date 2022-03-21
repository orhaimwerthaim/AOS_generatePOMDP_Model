#include "icaps.h"
#include <despot/core/pomdp.h> 
#include <stdlib.h>
#include <despot/solver/pomcp.h>
#include <sstream>
#include <despot/model_primitives/icaps/actionManager.h> 
#include <despot/model_primitives/icaps/enum_map_icaps.h> 
#include <despot/model_primitives/icaps/state.h> 
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
 *IcapsBelief class
 * ==============================================================================*/
int IcapsBelief::num_particles = 5234;
std::string IcapsBelief::beliefFromDB = "";
int IcapsBelief::currentInitParticleIndex = -1;

IcapsBelief::IcapsBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	icaps_(static_cast<const Icaps*>(model)) {
}

	
 	std::string Icaps::GetActionDescription(int actionId) const
	 {
		 return Prints::PrintActionDescription(ActionManager::actions[actionId]);
	 }

//void IcapsBelief::Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates) {
void IcapsBelief::Update(int actionId, OBS_TYPE obs) {
	history_.Add(actionId, obs);

	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = icaps_->Copy(particles_[cur]);
		bool terminal = icaps_->Step(*particle, Random::RANDOM.NextDouble(),
			actionId, reward, o);
 
		if (!terminal && o == obs) 
			{
				IcapsState &icaps_particle = static_cast<IcapsState &>(*particle);
				//if(!Globals::IsInternalSimulation() && updates.size() > 0)
				//{
				//	IcapsState::SetAnyValueLinks(&icaps_particle);
				//	map<std::string, bool>::iterator it;
				//	for (it = updates.begin(); it != updates.end(); it++)
				//	{
				//		*(icaps_particle.anyValueUpdateDic[it->first]) = it->second; 
				//	} 
				//}
				updated.push_back(particle);
		} else {
			icaps_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		icaps_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
 
}

/* ==============================================================================
 * IcapsPOMCPPrior class
 * ==============================================================================*/

class IcapsPOMCPPrior: public POMCPPrior {
private:
	const Icaps* icaps_;

public:
	IcapsPOMCPPrior(const Icaps* model) :
		POMCPPrior(model),
		icaps_(model) {
	}

	void ComputePreference(const State& state) {
		const IcapsState& icaps_state = static_cast<const IcapsState&>(state);
		weighted_preferred_actions_.clear();
        legal_actions_.clear();
		preferred_actions_.clear();
        std::vector<double> weighted_preferred_actions_un_normalized;

        double heuristicValueTotal = 0;
		for (int a = 0; a < 7; a++) {
            weighted_preferred_actions_un_normalized.push_back(0);
			double reward = 0;
			bool meetPrecondition = false; 
			Icaps::CheckPreconditions(icaps_state, reward, meetPrecondition, a);
            if(meetPrecondition)
            {
                legal_actions_.push_back(a);
                double __heuristicValue; 
                Icaps::ComputePreferredActionValue(icaps_state, __heuristicValue, a);
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
 * Icaps class
 * ==============================================================================*/

Icaps::Icaps(){
	
}

int Icaps::NumActions() const {
	return ActionManager::actions.size();
}

double Icaps::ObsProb(OBS_TYPE obs, const State& state, int actionId) const {
	return 0.9;
}

 


std::default_random_engine Icaps::generator;

std::normal_distribution<double>  Icaps::place_normal_dist1(30000,15000); //AOS.SampleNormal(30000,15000)
std::normal_distribution<double>  Icaps::observe_normal_dist2(15000,2000); //AOS.SampleNormal(15000,2000)
std::normal_distribution<double>  Icaps::pick_normal_dist3(40000,10000); //AOS.SampleNormal(40000,10000)
std::normal_distribution<double>  Icaps::navigate_normal_dist4(40000,10000); //AOS.SampleNormal(40000,10000)
std::discrete_distribution<> Icaps::place_discrete_dist1{0.8,0,0}; //AOS.SampleDiscrete(enumRealCase,{0.8,0,0})
std::discrete_distribution<> Icaps::pick_discrete_dist2{0.8,0,0,0}; //AOS.SampleDiscrete(enumRealCase,{0.8,0,0,0})
std::discrete_distribution<> Icaps::navigate_discrete_dist3{0.95,0.05}; //AOS.SampleDiscrete(enumRealCase,{0.95,0.05})
std::discrete_distribution<> Icaps::Environment_discrete_dist4{0.04,0,0.6}; //AOS.SampleDiscrete(tDiscreteLocation,{0.04,0,0.6})

State* Icaps::CreateStartState(string type) const {
    IcapsState* startState = memory_pool_.Allocate();
    IcapsState& state = *startState;
    startState->tDiscreteLocationObjects.push_back(eOutside_lab211);
    startState->tDiscreteLocationObjects.push_back(eAuditorium);
    startState->tDiscreteLocationObjects.push_back(eCorridor);
    startState->tDiscreteLocationObjects.push_back(eNear_elevator1);
    startState->tDiscreteLocationObjects.push_back(eUnknown);
    state.cupDiscreteGeneralLocation = eCorridor;
    state.cupAccurateLocation = false;
    state.handEmpty = true;
    state.locationOutside_lab211 = tLocation();
    state.locationOutside_lab211.discrete_location = eOutside_lab211;
     state.locationOutside_lab211.actual_location = true;
    state.locationAuditorium = tLocation();
    state.locationAuditorium.discrete_location = eAuditorium;
     state.locationAuditorium.actual_location = true;
    state.locationNear_elevator1 = tLocation();
    state.locationNear_elevator1.discrete_location = eNear_elevator1;
     state.locationNear_elevator1.actual_location = true;
    state.locationCorridor = tLocation();
    state.locationCorridor.discrete_location = eCorridor;
     state.locationCorridor.actual_location = true;
    startState->tLocationObjectsForActions["state.locationOutside_lab211"] = (state.locationOutside_lab211);
    startState->tLocationObjectsForActions["state.locationAuditorium"] = (state.locationAuditorium);
    startState->tLocationObjectsForActions["state.locationNear_elevator1"] = (state.locationNear_elevator1);
    startState->tLocationObjectsForActions["state.locationCorridor"] = (state.locationCorridor);
    state.robotGenerallocation=state.locationNear_elevator1.discrete_location;
    state.cupDiscreteGeneralLocation=state.tDiscreteLocationObjects[Environment_discrete_dist4(generator)];;
    if (ActionManager::actions.size() == 0)
    {
        ActionManager::Init(const_cast <IcapsState*> (startState));
    }
    return startState;
}



Belief* Icaps::InitialBelief(const State* start, string type) const {
	int N = IcapsBelief::num_particles;
	vector<State*> particles(N);
	for (int i = 0; i < N; i++) {
        IcapsBelief::currentInitParticleIndex = i;
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}
    IcapsBelief::currentInitParticleIndex = -1;
	return new IcapsBelief(particles, this);
}
 

 

POMCPPrior* Icaps::CreatePOMCPPrior(string name) const { 
		return new IcapsPOMCPPrior(this);
}

void Icaps::PrintState(const State& state, ostream& ostr) const {
	const IcapsState& farstate = static_cast<const IcapsState&>(state);
	if (ostr)
		ostr << Prints::PrintState(farstate);
}

void Icaps::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {
	const IcapsState& farstate = static_cast<const IcapsState&>(state);
	
	ostr << observation <<endl;
}

void Icaps::PrintBelief(const Belief& belief, ostream& out) const {
	 out << "called PrintBelief(): b printed"<<endl;
		out << endl;
	
}

void Icaps::PrintAction(int actionId, ostream& out) const {
	out << Prints::PrintActionDescription(ActionManager::actions[actionId]) << endl;
}

State* Icaps::Allocate(int state_id, double weight) const {
	IcapsState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* Icaps::Copy(const State* particle) const {
	IcapsState* state = memory_pool_.Allocate();
	*state = *static_cast<const IcapsState*>(particle);
	state->SetAllocated();



	return state;
}

void Icaps::Free(State* particle) const {
	memory_pool_.Free(static_cast<IcapsState*>(particle));
}

int Icaps::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

bool Icaps::Step(State& s_state__, double rand_num, int actionId, double& reward,
	OBS_TYPE& observation) const {
    reward = 0;
	bool isNextStateFinal = false;
	Random random(rand_num);
	int __moduleExecutionTime = -1;
	bool meetPrecondition = false;
	
	IcapsState &state__ = static_cast<IcapsState &>(s_state__);
	 logd << "[Icaps::Step] Selected Action:" << Prints::PrintActionDescription(ActionManager::actions[actionId]) << "||State"<< Prints::PrintState(state__);
	CheckPreconditions(state__, reward, meetPrecondition, actionId);
	 
	State *s_state = Copy(&s_state__);
	IcapsState &state = static_cast<IcapsState &>(*s_state);

	
	SampleModuleExecutionTime(state__, rand_num, actionId, __moduleExecutionTime);

	ExtrinsicChangesDynamicModel(state, state__, rand_num, actionId, __moduleExecutionTime);

	State *s_state_ = Copy(&s_state__);
	IcapsState &state_ = static_cast<IcapsState &>(*s_state_);

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

void Icaps::CheckPreconditions(const IcapsState& state, double &reward, bool &__meetPrecondition, int actionId)
    {
        ActionType &actType = ActionManager::actions[actionId]->actionType;
        __meetPrecondition = true;
            if(actType == placeAction)
            {
                __meetPrecondition=state.handEmpty==false&&(state.robotGenerallocation==eAuditorium||state.robotGenerallocation==eOutside_lab211||state.robotGenerallocation==eCorridor);
                if(!__meetPrecondition) reward += -800;
            }
            if(actType == observeAction)
            {
                if(!__meetPrecondition) reward += 0;
            }
            if(actType == pickAction)
            {
                __meetPrecondition=(state.cupAccurateLocation == true)&&state.handEmpty==true&&state.robotGenerallocation==state.cupDiscreteGeneralLocation;
                if(!__meetPrecondition) reward += -800;
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
                __meetPrecondition=state.robotGenerallocation!=eUnknown;
                if(!__meetPrecondition) reward += -800;
            }
    }

void Icaps::ComputePreferredActionValue(const IcapsState& state, double &__heuristicValue, int actionId)
    {
        __heuristicValue = 0;
        ActionType &actType = ActionManager::actions[actionId]->actionType;
            if(actType == placeAction)
            {
            }
            if(actType == observeAction)
            {
            }
            if(actType == pickAction)
            {
            }
            if(actType == navigateAction)
            {
                NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
                tLocation &oDesiredLocation = act.oDesiredLocation;
            }
        __heuristicValue = __heuristicValue < 0 ? 0 : __heuristicValue;
    }

void Icaps::SampleModuleExecutionTime(const IcapsState& farstate, double rand_num, int actionId, int &__moduleExecutionTime) const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    if(actType == placeAction)
    {
        __moduleExecutionTime=Icaps::place_normal_dist1(Icaps::generator);
    }
    if(actType == observeAction)
    {
        __moduleExecutionTime=Icaps::observe_normal_dist2(Icaps::generator);
    }
    if(actType == pickAction)
    {
        __moduleExecutionTime=Icaps::pick_normal_dist3(Icaps::generator);
    }
    if(actType == navigateAction)
    {
        __moduleExecutionTime=Icaps::pick_normal_dist3(Icaps::generator);
    }
}

void Icaps::ExtrinsicChangesDynamicModel(const IcapsState& state, IcapsState& state_, double rand_num, int actionId, const int &__moduleExecutionTime)  const
{
    ActionType &actType = ActionManager::actions[actionId]->actionType;
}

void Icaps::ModuleDynamicModel(const IcapsState &state, const IcapsState &state_, IcapsState &state__, double rand_num, int actionId, double &__reward, OBS_TYPE &observation, const int &__moduleExecutionTime, const bool &__meetPrecondition) const
{
    std::hash<std::string> hasher;
    ActionType &actType = ActionManager::actions[actionId]->actionType;
    observation = -1;
    int startObs = observation;
    std::string __moduleResponseStr = "NoStrResponse";
    OBS_TYPE &__moduleResponse = observation;
    if(actType == placeAction)
    {
        IcapsResponseModuleAndTempEnums  realCase;
        realCase=(IcapsResponseModuleAndTempEnums)(place_enumRealCase + 1 + Icaps::place_discrete_dist1(Icaps::generator));
        state__.handEmpty=(realCase==place_success||realCase==place_droppedObject)?true:false;
        if(realCase==place_success)state__.cupDiscreteGeneralLocation=state.robotGenerallocation;
        if(realCase==place_droppedObject)state__.cupDiscreteGeneralLocation=eUnknown;
        if(realCase==place_success){if(AOSUtils::Bernoulli(0.9))__moduleResponse=place_ePlaceActionSuccess;
        else __moduleResponse=place_eFailedUnknown;
        }if(realCase==place_droppedObject)__moduleResponse=(AOSUtils::Bernoulli(0.9))?place_eDroppedObject:place_eFailedUnknown;
        if(realCase==place_unknownFailure)__moduleResponse=place_eFailedUnknown;
        __reward=-100;
    }
    if(actType == observeAction)
    {
        IcapsResponseModuleAndTempEnums  realCase;
        if(state.robotGenerallocation==state.cupDiscreteGeneralLocation){if(AOSUtils::Bernoulli(0.99))realCase=observe_observed;
        else realCase=observe_notObserved;
        }else realCase=observe_notObserved;
        if(realCase==observe_observed&&AOSUtils::Bernoulli(0.9)){state__.cupAccurateLocation = true;
        __moduleResponse=observe_eObserved;
        }else {state__.cupAccurateLocation = false;
        __moduleResponse=observe_eNotObserved;
        };
        __reward=-100;
    }
    if(actType == pickAction)
    {
        IcapsResponseModuleAndTempEnums  realCase;
        realCase=(IcapsResponseModuleAndTempEnums)(pick_enumRealCase + 1 + Icaps::pick_discrete_dist2(Icaps::generator));
        state__.handEmpty=(realCase==pick_actual_pick_action_success)?false:true;
        if(realCase==pick_actual_pick_action_success||realCase==pick_actual_broke_the_object){state__.cupDiscreteGeneralLocation=eUnknown;
        state__.cupAccurateLocation = false;
        };
        if(realCase==pick_actual_not_holding){if(AOSUtils::Bernoulli(0.9))__moduleResponse=pick_res_not_holding;
        else __moduleResponse=pick_res_pick_action_success;
        }if(realCase==pick_actual_pick_action_success){if(AOSUtils::Bernoulli(0.9))__moduleResponse=pick_res_pick_action_success;
        else __moduleResponse=pick_res_not_holding;
        }else __moduleResponse=pick_res_broke_the_object;
        __reward=-100;
    }
    if(actType == navigateAction)
    {
        NavigateActionDescription act = *(static_cast<NavigateActionDescription *>(ActionManager::actions[actionId]));
        tLocation &oDesiredLocation = act.oDesiredLocation;
        IcapsResponseModuleAndTempEnums  realCase;
        realCase=(IcapsResponseModuleAndTempEnums)(navigate_enumRealCase + 1 + Icaps::navigate_discrete_dist3(Icaps::generator));
        if(realCase==navigate_action_success)state__.robotGenerallocation=oDesiredLocation.discrete_location;
        if(realCase==navigate_action_success&&AOSUtils::Bernoulli(0.9))__moduleResponse=navigate_eSuccess;
        else __moduleResponse=navigate_eFailed;
        __reward=-100;
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

bool Icaps::ProcessSpecialStates(IcapsState &state, double &reward) const
{
    bool isFinalState = false;
    if(state.OneTimeRewardUsed[0])
    {
        if (state.robotGenerallocation == eNear_elevator1 && state.cupDiscreteGeneralLocation == eAuditorium)
        {
            reward += 8000;
            isFinalState = true;
        }
    }
    return isFinalState;
}





std::string Icaps::PrintObs(int action, OBS_TYPE obs) const 
{
	return Prints::PrintObs(action, obs);
}

std::string Icaps::PrintStateStr(const State &state) const { return ""; };
}// namespace despot
