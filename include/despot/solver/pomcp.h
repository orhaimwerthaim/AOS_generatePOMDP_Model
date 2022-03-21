#ifndef POMCP_H
#define POMCP_H
//#define WITH_ROOT_EPSILON_GREEDY
//#define WITH_FULL_EPSILON_GREEDY


#ifdef WITH_FULL_EPSILON_GREEDY
#define WITH_ROOT_EPSILON_GREEDY
#endif

#include <despot/core/pomdp.h>
#include <despot/core/node.h>
#include <despot/core/globals.h> 
#include <despot/model_primitives/iros/actionManager.h>
namespace despot {

/* =============================================================================
 * POMCPPrior class
 * =============================================================================*/
 
class POMCPPrior
{
protected:
	const DSPOMDP* model_;
	History history_;
	double exploration_constant_;
    std::vector<double> weighted_preferred_actions_;
	std::vector<int> preferred_actions_;
	std::vector<int> legal_actions_;

public:
	POMCPPrior(const DSPOMDP* model);
	virtual ~POMCPPrior();

	inline void exploration_constant(double constant) {
		exploration_constant_ = constant;
	}

	inline double exploration_constant() const {
		return exploration_constant_;
	}

	inline virtual int SmartCount(int action) const {
		return 10;
	}

	inline virtual double SmartValue(int action) const {
		return 1;
	}

	inline virtual const History& history() const {
		return history_;
	}

	inline virtual void history(History h) {
		history_ = h;
	}

	inline virtual void Add(int action, OBS_TYPE obs) {
		history_.Add(action, obs);
	}

	inline virtual void PopLast() {
		history_.RemoveLast();
	}

  inline virtual void PopAll() {
		history_.Truncate(0);
	}

	virtual void ComputePreference(const State& state) = 0;

	const std::vector<int>& preferred_actions() const;
	const std::vector<int>& legal_actions() const;
    const std::vector<double>& weighted_preferred_actions() const;

	int GetAction(const State& state);
};

/* =============================================================================
 * UniformPOMCPPrior class
 * =============================================================================*/

class UniformPOMCPPrior: public POMCPPrior {
public:
	UniformPOMCPPrior(const DSPOMDP* model);
	virtual ~UniformPOMCPPrior();

	void ComputePreference(const State& state);
};

/* =============================================================================
 * POMCP class
 * =============================================================================*/



class POMCP: public Solver {
protected:
	VNode* root_;
	POMCPPrior* prior_;
	bool reuse_;

public:
	POMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief = NULL);
	virtual ValuedAction Search();
	virtual ValuedAction Search(double timeout);

	void reuse(bool r);
	virtual void belief(Belief* b);
	virtual void Update(int action, OBS_TYPE obs);
	//virtual void Update(int action, OBS_TYPE obs, std::map<std::string, bool> updatesFromAction);
	static VNode* CreateVNode(int depth, const State*, POMCPPrior* prior,
		const DSPOMDP* model);
	static double Simulate(State* particle, RandomStreams& streams,
		VNode* vnode, const DSPOMDP* model, POMCPPrior* prior);
	static double Rollout(State* particle, int depth, const DSPOMDP* model,
		POMCPPrior* prior, std::vector<int>* simulateActionSequence);
	static double Rollout(State* particle, RandomStreams& streams, int depth,
		const DSPOMDP* model, POMCPPrior* prior);
	static ValuedAction Evaluate(VNode* root, std::vector<State*>& particles,
		RandomStreams& streams, const DSPOMDP* model, POMCPPrior* prior);
	static int UpperBoundAction(const VNode* vnode, double explore_constant, const DSPOMDP* model, Belief* b);
	static int UpperBoundAction(const VNode* vnode, double explore_constant);
    static double Simulate(State* particle, VNode* root, const DSPOMDP* model,
		POMCPPrior* prior, std::vector<int>* simulateActionSequence);

#ifdef WITH_ROOT_EPSILON_GREEDY
static std::default_random_engine generator;
static std::uniform_int_distribution<int> rand_action_distribution;
static int UpperBoundAction(const VNode* vnode, double explore_constant, bool is_root_node);
static double Simulate(State* particle, VNode* root, const DSPOMDP* model,
		POMCPPrior* prior, std::vector<int>* simulateActionSequence, bool is_root_node);
#endif

	static ValuedAction OptimalAction(const VNode* vnode);
	static int Count(const VNode* vnode);
	 //std::string GenerateDotGraph(VNode *root, int depthLimit, const DSPOMDP* model);
	 //void GenerateDotGraphVnode(VNode *vnode, int &currentNodeID, std::stringstream &ssNodes, std::stringstream &ssEdges, int depthLimit, const DSPOMDP* model);
	 void GenerateDebugJsonVnode(VNode *vnode, std::stringstream &ss, int depthLimit, const DSPOMDP* model);
	 std::string GenerateDebugJson(VNode *root, int depthLimit, const DSPOMDP* model);
};

/* =============================================================================
 * DPOMCP class
 * =============================================================================*/

class DPOMCP: public POMCP {
public:
	DPOMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief = NULL);

	virtual ValuedAction Search(double timeout);
	static VNode* ConstructTree(std::vector<State*>& particles,
		RandomStreams& streams, const DSPOMDP* model, POMCPPrior* prior,
		History& history, double timeout);

	virtual void belief(Belief* b);
	virtual void Update(int action, OBS_TYPE obs);
	
};


} // namespace despot

#endif
