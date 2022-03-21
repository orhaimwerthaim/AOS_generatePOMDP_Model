
#include "globals.h"
#include <despot/core/pomdp.h>
#include <despot/solver/pomcp.h> 
#include <random>
#include <string>
#include <despot/model_primitives/icaps/enum_map_icaps.h> 
#include <despot/model_primitives/icaps/state.h> 
namespace despot {

/* ==============================================================================
 * IcapsState class
 * ==============================================================================*/

class IcapsState;
class AOSUtils
{
	public:
	static bool Bernoulli(double);
};
 




/* ==============================================================================
 * Icaps and PocmanBelief class
 * ==============================================================================*/
class Icaps;
class IcapsBelief: public ParticleBelief {
protected:
	const Icaps* icaps_;
public:
	static std::string beliefFromDB;
	static int currentInitParticleIndex;
	static int num_particles; 
	IcapsBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior =
		NULL);
	void Update(int actionId, OBS_TYPE obs);
	//void Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates);
};

/* ==============================================================================
 * Icaps 
 * ==============================================================================*/
/**
 * The implementation is adapted from that included in the POMCP software.
 */

class Icaps: public DSPOMDP {
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
 	static void CheckPreconditions(const IcapsState& state, double &reward, bool &meetPrecondition, int actionId);
    static void ComputePreferredActionValue(const IcapsState& state, double &__heuristicValue, int actionId);
     
 
	Icaps(); 

private:
	void SampleModuleExecutionTime(const IcapsState& state, double rand_num, int actionId, int &moduleExecutionTime) const;
	void ExtrinsicChangesDynamicModel(const IcapsState& initState, IcapsState& afterExState, double rand_num, int actionId,
		const int &moduleExecutionTime) const;
	void ModuleDynamicModel(const IcapsState &initState, const IcapsState &afterExState, IcapsState &nextState, double rand_num, int actionId, double &reward,
								 OBS_TYPE &observation, const int &moduleExecutionTime, const bool &__meetPrecondition) const;
	bool ProcessSpecialStates(IcapsState &state, double &reward) const;

	mutable MemoryPool<IcapsState> memory_pool_;
	static std::default_random_engine generator;
    static std::normal_distribution<> place_normal_dist1; //AOS.SampleNormal(30000,15000)
    static std::normal_distribution<> observe_normal_dist2; //AOS.SampleNormal(15000,2000)
    static std::normal_distribution<> pick_normal_dist3; //AOS.SampleNormal(40000,10000)
    static std::normal_distribution<> navigate_normal_dist4; //AOS.SampleNormal(40000,10000)
    static std::discrete_distribution<> place_discrete_dist1; //AOS.SampleDiscrete(enumRealCase,{0.8,0,0})
    static std::discrete_distribution<> pick_discrete_dist2; //AOS.SampleDiscrete(enumRealCase,{0.8,0,0,0})
    static std::discrete_distribution<> navigate_discrete_dist3; //AOS.SampleDiscrete(enumRealCase,{0.95,0.05})
    static std::discrete_distribution<> Environment_discrete_dist4; //AOS.SampleDiscrete(tDiscreteLocation,{0.04,0,0.6})

};
} // namespace despot
 