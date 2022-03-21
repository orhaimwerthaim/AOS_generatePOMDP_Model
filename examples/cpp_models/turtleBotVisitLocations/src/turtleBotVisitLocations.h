
#include "globals.h"
#include <despot/core/pomdp.h>
#include <despot/solver/pomcp.h> 
#include <random>
#include <string>
#include <despot/model_primitives/turtleBotVisitLocations/enum_map_turtleBotVisitLocations.h> 
#include <despot/model_primitives/turtleBotVisitLocations/state.h> 
namespace despot {

/* ==============================================================================
 * TurtleBotVisitLocationsState class
 * ==============================================================================*/

class TurtleBotVisitLocationsState;
class AOSUtils
{
	public:
	static bool Bernoulli(double);
};
 




/* ==============================================================================
 * TurtleBotVisitLocations and PocmanBelief class
 * ==============================================================================*/
class TurtleBotVisitLocations;
class TurtleBotVisitLocationsBelief: public ParticleBelief {
protected:
	const TurtleBotVisitLocations* turtleBotVisitLocations_;
public:
	static std::string beliefFromDB;
	static int currentInitParticleIndex;
	static int num_particles; 
	TurtleBotVisitLocationsBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior =
		NULL);
	void Update(int actionId, OBS_TYPE obs);
	//void Update(int actionId, OBS_TYPE obs, std::map<std::string,bool> updates);
};

/* ==============================================================================
 * TurtleBotVisitLocations 
 * ==============================================================================*/
/**
 * The implementation is adapted from that included in the POMCP software.
 */

class TurtleBotVisitLocations: public DSPOMDP {
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
 	static void CheckPreconditions(const TurtleBotVisitLocationsState& state, double &reward, bool &meetPrecondition, int actionId);
    static void ComputePreferredActionValue(const TurtleBotVisitLocationsState& state, double &__heuristicValue, int actionId);
     
 
	TurtleBotVisitLocations(); 

private:
	void SampleModuleExecutionTime(const TurtleBotVisitLocationsState& state, double rand_num, int actionId, int &moduleExecutionTime) const;
	void ExtrinsicChangesDynamicModel(const TurtleBotVisitLocationsState& initState, TurtleBotVisitLocationsState& afterExState, double rand_num, int actionId,
		const int &moduleExecutionTime) const;
	void ModuleDynamicModel(const TurtleBotVisitLocationsState &initState, const TurtleBotVisitLocationsState &afterExState, TurtleBotVisitLocationsState &nextState, double rand_num, int actionId, double &reward,
								 OBS_TYPE &observation, const int &moduleExecutionTime, const bool &__meetPrecondition) const;
	bool ProcessSpecialStates(TurtleBotVisitLocationsState &state, double &reward) const;

	mutable MemoryPool<TurtleBotVisitLocationsState> memory_pool_;
	static std::default_random_engine generator;

};
} // namespace despot
 