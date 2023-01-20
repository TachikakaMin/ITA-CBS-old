#pragma once

#include <map>
#include <libMultiRobotPlanning/timer.hpp>
#include "a_star.hpp"

namespace libMultiRobotPlanning {

/*!
  \example cbs_ta.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Conflict-Based-Search with Optimal Task Assignment (CBS-TA) algorithm
to find tasks and collision-free paths jointly, minimizing sum-of-cost.

This class implements the Conflict-Based-Search with Optimal Task Assignment
(CBS-TA) algorithm.
This algorithm assigns tasks and finds collision-free path for multiple agents
with start and
goal locations given for each agent.
CBS-TA is an extension of the CBS algorithms, operating in a search forest
rather
than a search tree (where each root node refers to a possible assignment).
CBS-TA is optimal with respect to the sum-of-individual costs.

Details of the algorithm can be found in the following paper:\n
W. Hönig, S. Kiesel, A. Tinka, J. W. Durham, and N. Ayanian.\n
"Conflict-Based Search with Optimal Task Assignment",\n
In Proc. of the 17th International Conference on Autonomous Agents and
Multiagent Systems (AAMAS)\n
Stockholm, Sweden, July 2018.

The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\param Task Custom task type to be used for assignment.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints

  - `Cost admissibleHeuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state for the current agent.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first conflict for the given solution for each agent. Return true
if a conflict was found and false otherwise.

  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.

  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.

  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/
// CBSTA<State, Action, int, Conflict, Constraints, Location, Environment>
    template<typename State, typename Action, typename Cost, typename Conflict,
            typename Constraints, typename Task, typename Environment>
    class CBSTA {
    public:
        CBSTA(Environment &environment) : m_env(environment) {}

        int id;
        double avg_next_ta_time, d_cost;
        double p_ta_time, p_lowlevel_search_time, p_lowlevel_queue_time, p_create_low_level_node_time;

        bool search(const std::vector<State> &initialStates,
                    std::vector<PlanResult<State, Action, Cost> > &solution) {
            Timer timer, timer_ta, timer_lowlevel_search, timer_lowlevel_queue, timer_low_level_node;
            HighLevelNode start;
            size_t numAgents = initialStates.size();
            start.solution.resize(numAgents);
            start.constraints.resize(numAgents);
            start.cost = 0;
            start.id = 0;
            start.isRoot = true;
            p_ta_time = 0;
            p_lowlevel_search_time = 0;
            p_lowlevel_queue_time = 0;
            p_create_low_level_node_time = 0;
            // First Assignment
            timer_ta.reset();
            m_env.nextTaskAssignment(start.tasks);
            timer_ta.stop();
            p_ta_time += timer_ta.elapsedSeconds();


//    for (const auto& [key, value] : start.tasks) {
//      std::cout << '[' << key << "] = (" << value.x << ',' << value.y << ") ; " << std::endl;
//    }

            // find individual paths using low-level()
            for (size_t i = 0; i < initialStates.size(); ++i) {

                bool success = false;
                if (!start.tasks.empty()) {
                    timer_lowlevel_search.reset();
                    LowLevelEnvironment llenv(m_env, i, start.constraints[i],
                                              start.task(i));
                    LowLevelSearch_t lowLevel(llenv);
                    success = lowLevel.search(initialStates[i], start.solution[i]);
                    timer_lowlevel_search.stop();
                    p_lowlevel_search_time += timer_lowlevel_search.elapsedSeconds();
                }
                if (!success) {
                    return false;
                }

                start.cost += start.solution[i].cost;
            }
            d_cost = start.cost;
            // std::priority_queue<HighLevelNode> open;
            typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                    boost::heap::mutable_<true> >
                    open;

            timer_lowlevel_queue.reset();
            auto handle = open.push(start);
            (*handle).handle = handle;
            timer_lowlevel_queue.stop();
            p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();



            timer.reset();
            avg_next_ta_time = 0;
            double last_time = timer.elapsedSeconds();
            int ta_cnt = 0;

            solution.clear();
            this->id = 1;
            while (!open.empty()) {
                // P ← the best node from OPEN // the lowest solution cost
                timer_low_level_node.reset();
                HighLevelNode P = open.top();
                timer_low_level_node.stop();
                p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();

                m_env.onExpandHighLevelNode(P.cost); //
                // std::cout << "expand: " << P << std::endl;

                timer_lowlevel_queue.reset();
                open.pop();
                timer_lowlevel_queue.stop();
                p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();


                // Validate the paths in P until a conflict occurs.
                Conflict conflict;
                if (!m_env.getFirstConflict(P.solution, conflict)) {
                    std::cout << "done; cost: " << P.cost << std::endl;
                    d_cost = P.cost - d_cost;
                    solution = P.solution;
                    return true;
                }

                if (P.isRoot) {
                    // std::cout << "root node expanded; add new root" << std::endl;
                    timer_low_level_node.reset();
                    HighLevelNode n;
                    timer_low_level_node.stop();
                    p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();


                    // New R node, R.assignment ← nextAssignment()
                    timer_ta.reset();
                    m_env.nextTaskAssignment(n.tasks);
                    timer_ta.stop();
                    p_ta_time += timer_ta.elapsedSeconds();


                    if (n.tasks.size() > 0) {
                        timer_low_level_node.reset();
                        n.solution.resize(numAgents);
                        n.constraints.resize(numAgents);
                        n.cost = 0;
                        n.id = this->id;
                        n.isRoot = true;
                        timer_low_level_node.stop();
                        p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();


                        bool allSuccessful = true;
                        // R.solution ← find individual paths using low-level()
                        for (size_t i = 0; i < numAgents; ++i) {
                            timer_lowlevel_search.reset();
                            LowLevelEnvironment llenv(m_env, i, n.constraints[i], n.task(i));
                            LowLevelSearch_t lowLevel(llenv);
                            bool success = lowLevel.search(initialStates[i], n.solution[i]);
                            timer_lowlevel_search.stop();
                            p_lowlevel_search_time += timer_lowlevel_search.elapsedSeconds();
                            if (!success) {
                                allSuccessful = false;
                                break;
                            }
                            n.cost += n.solution[i].cost;
                        }
                        if (allSuccessful) {
                            timer_lowlevel_queue.reset();
                            auto handle = open.push(n);
                            (*handle).handle = handle;
                            ++(this->id);
                            timer_lowlevel_queue.stop();
                            p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();
                        }
                        timer.stop();
                        double new_time = timer.elapsedSeconds();
                        avg_next_ta_time =
                                avg_next_ta_time / (ta_cnt + 1) * ta_cnt + (new_time - last_time) / (ta_cnt + 1);
                        last_time = new_time;
                        ta_cnt++;
                    }
                }

                // create additional nodes to resolve conflict
                // std::cout << "Found conflict: " << conflict << std::endl;
                // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
                // conflict.type << std::endl;

                std::map<size_t, Constraints> constraints;
                m_env.createConstraintsFromConflict(conflict, constraints);
                for (const auto &c: constraints) {
                    // std::cout << "Add HL node for " << c.first << std::endl;
                    size_t i = c.first;
                    // std::cout << "create child with id " << id << std::endl;
                    HighLevelNode newNode = P;
                    newNode.id = this->id;
                    // (optional) check that this constraint was not included already
                    // std::cout << newNode.constraints[i] << std::endl;
                    // std::cout << c.second << std::endl;
                    assert(!newNode.constraints[i].overlap(c.second));

                    newNode.constraints[i].add(c.second);

                    newNode.cost -= newNode.solution[i].cost;

                    timer_lowlevel_search.reset();
                    LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                              newNode.task(i));
                    LowLevelSearch_t lowLevel(llenv);
                    bool success = lowLevel.search(initialStates[i], newNode.solution[i]);
                    timer_lowlevel_search.stop();
                    p_lowlevel_search_time += timer_lowlevel_search.elapsedSeconds();

                    newNode.cost += newNode.solution[i].cost;

                    if (success) {
                        // std::cout << "  success. cost: " << newNode.cost << std::endl;
                        timer_lowlevel_queue.reset();
                        auto handle = open.push(newNode);
                        ++(this->id);
                        (*handle).handle = handle;
                        timer_lowlevel_queue.stop();
                        p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();
                    }
                }
            }

            return false;
        }

    private:
        struct HighLevelNode {
            std::vector<PlanResult<State, Action, Cost> > solution;
            std::vector<Constraints> constraints;
            std::map<size_t, Task> tasks; // maps from index to task (and does not contain an entry if no task was assigned)

            Cost cost;

            int id;
            bool isRoot;

            typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                    boost::heap::mutable_<true> >::handle_type
                    handle;

            bool operator<(const HighLevelNode &n) const {
                // if (cost != n.cost)
                return cost > n.cost;
                // return id > n.id;
            }

            Task *task(size_t idx) {
                Task *task = nullptr;
                auto iter = tasks.find(idx);
                if (iter != tasks.end()) {
                    task = &iter->second;
                }
                return task;
            }

            friend std::ostream &operator<<(std::ostream &os, const HighLevelNode &c) {
                os << "id: " << c.id << " cost: " << c.cost << std::endl;
                for (size_t i = 0; i < c.solution.size(); ++i) {
                    os << "Agent: " << i << std::endl;
                    os << " States:" << std::endl;
                    for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
                        os << "  " << c.solution[i].states[t].first << std::endl;
                    }
                    os << " Constraints:" << std::endl;
                    os << c.constraints[i];
                    os << " cost: " << c.solution[i].cost << std::endl;
                }
                return os;
            }
        };

        struct LowLevelEnvironment {
            LowLevelEnvironment(Environment &env, size_t agentIdx,
                                const Constraints &constraints, const Task *task)
                    : m_env(env)
            // , m_agentIdx(agentIdx)
            // , m_constraints(constraints)
            {
                m_env.setLowLevelContext(agentIdx, &constraints, task);
            }

            Cost admissibleHeuristic(const State &s) {
                return m_env.admissibleHeuristic(s);
            }

            bool isSolution(const State &s) { return m_env.isSolution(s); }

            void getNeighbors(const State &s,
                              std::vector<Neighbor<State, Action, Cost> > &neighbors) {
                m_env.getNeighbors(s, neighbors);
            }

            void onExpandNode(const State &s, Cost fScore, Cost gScore) {
                // std::cout << "LL expand: " << s << std::endl;
                m_env.onExpandLowLevelNode(s, fScore, gScore);
            }

            void onDiscover(const State & /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
                // std::cout << "LL discover: " << s << std::endl;
                // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
            }

        private:
            Environment &m_env;
            // size_t m_agentIdx;
            // const Constraints& m_constraints;
        };

    private:
        Environment &m_env;
        typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
    };

}  // namespace libMultiRobotPlanning
