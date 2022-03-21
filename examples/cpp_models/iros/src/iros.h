
#include "globals.h"
#include <despot/core/pomdp.h>
#include <despot/solver/pomcp.h> 
#include <random>
#include <string>
#include <despot/model_primitives/iros/enum_map_iros.h> 
#include <despot/model_primitives/iros/state.h> 
namespace despot {

/* ==============================================================================
 * IrosState class
 * ==============================================================================*/

class IrosState;
class AOSUtils
{
	public:
	static bool Bernoulli(double);
};
 




/* ==============================================================================
 * Iros and PocmanBelief class
 * ==============================================================================*/
class Iros;
class IrosBelief: public ParticleBelief {
protected:
	const Iros* iros_;
public:
	static std::string beliefFromDB;
	static int currentInitParticleIndex;
	static int num_particles; 
	IrosBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior =
		NULL);
	void Update(int actionId, OBS_TYPE obs);
	//void Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates);
};

/* ==============================================================================
 * Iros 
 * ==============================================================================*/
/**
 * The implementation is adapted from that included in the POMCP software.
 */

class Iros: public DSPOMDP {
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
 	static void CheckPreconditions(const IrosState& state, double &reward, bool &meetPrecondition, int actionId);
    static void ComputePreferredActionValue(const IrosState& state, double &__heuristicValue, int actionId);
     
 
	Iros(); 

private:
	void SampleModuleExecutionTime(const IrosState& state, double rand_num, int actionId, int &moduleExecutionTime) const;
	void ExtrinsicChangesDynamicModel(const IrosState& initState, IrosState& afterExState, double rand_num, int actionId,
		const int &moduleExecutionTime) const;
	void ModuleDynamicModel(const IrosState &initState, const IrosState &afterExState, IrosState &nextState, double rand_num, int actionId, double &reward,
								 OBS_TYPE &observation, const int &moduleExecutionTime, const bool &__meetPrecondition) const;
	bool ProcessSpecialStates(IrosState &state, double &reward) const;

	mutable MemoryPool<IrosState> memory_pool_;
	static std::default_random_engine generator;
    static std::discrete_distribution<> navigate_discrete_dist1; //AOS.SampleDiscrete(enumRealCase,{1.0,0.0})
    static std::discrete_distribution<> enhanced_pick_discrete_dist2; //AOS.SampleDiscrete(enumRealCase,{0.727,0.181,0.09,0.0})
    static std::discrete_distribution<> enhanced_pick_discrete_dist3; //AOS.SampleDiscrete(enumRealCase,{0.1,0.0,0.0,0.9})

};
} // namespace despot
 