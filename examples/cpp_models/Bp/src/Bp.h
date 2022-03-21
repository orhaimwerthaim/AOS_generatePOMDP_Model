
#include "globals.h"
#include <despot/core/pomdp.h>
#include <despot/solver/pomcp.h> 
#include <random>
#include <string>
#include <despot/model_primitives/Bp/enum_map_Bp.h> 
#include <despot/model_primitives/Bp/state.h> 
namespace despot {

/* ==============================================================================
 * BpState class
 * ==============================================================================*/

class BpState;
class AOSUtils
{
	public:
	static bool Bernoulli(double);
};
 




/* ==============================================================================
 * Bp and PocmanBelief class
 * ==============================================================================*/
class Bp;
class BpBelief: public ParticleBelief {
protected:
	const Bp* Bp_;
public:
	static std::string beliefFromDB;
	static int currentInitParticleIndex;
	static int num_particles; 
	BpBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior =
		NULL);
	void Update(int actionId, OBS_TYPE obs);
	//void Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates);
};

/* ==============================================================================
 * Bp 
 * ==============================================================================*/
/**
 * The implementation is adapted from that included in the POMCP software.
 */

class Bp: public DSPOMDP {
public:
	virtual std::string PrintObs(int action, OBS_TYPE obs) const;
	virtual std::string PrintStateStr(const State &state) const;
	virtual std::string GetActionDescription(int) const;
	void UpdateStateByRealModuleObservation(State &state, int actionId, OBS_TYPE &observation) const;
	virtual bool Step(State &state, double rand_num, int actionId, double &reward,
					  OBS_TYPE &observation) const;
	int NumActions() const;
	virtual double ObsProb(OBS_TYPE obs, const State& state, int actionId) const;

	virtual State* CreateStartState(std::string type = "DEFAULT") const;
	virtual Belief* InitialBelief(const State* start,
		std::string type = "PARTICLE") const;

	inline double GetMaxReward() const {
		return globals::MAX_IMMEDIATE_REWARD;
	}
	 

	inline ValuedAction GetMinRewardAction() const {
		return ValuedAction(0, globals::MIN_IMMEDIATE_REWARD);
	}
	 
	POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;

	virtual void PrintState(const State& state, std::ostream& out = std::cout) const;
	

	
	virtual void PrintObs(const State& state, OBS_TYPE observation,
		std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintAction(int actionId, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;
	virtual State* Copy(const State* particle) const;
	virtual void Free(State* particle) const;
	int NumActiveParticles() const;
 	static void CheckPreconditions(const BpState& state, double &reward, bool &meetPrecondition, int actionId);
    static void ComputePreferredActionValue(const BpState& state, double &__heuristicValue, int actionId);
     
 
	Bp(); 

private:
	void SampleModuleExecutionTime(const BpState& state, double rand_num, int actionId, int &moduleExecutionTime) const;
	void ExtrinsicChangesDynamicModel(const BpState& initState, BpState& afterExState, double rand_num, int actionId,
		const int &moduleExecutionTime) const;
	void ModuleDynamicModel(const BpState &initState, const BpState &afterExState, BpState &nextState, double rand_num, int actionId, double &reward,
								 OBS_TYPE &observation, const int &moduleExecutionTime, const bool &__meetPrecondition) const;
	bool ProcessSpecialStates(BpState &state, double &reward) const;

	mutable MemoryPool<BpState> memory_pool_;
	static std::default_random_engine generator;
    static std::discrete_distribution<> Environment_discrete_dist1; //AOS.SampleDiscrete(tCell,{0.5,0,0,0})

};
} // namespace despot
 