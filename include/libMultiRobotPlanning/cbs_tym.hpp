//
// Created by YIMIN TANG on 2022/09/13.
//

#pragma once

#include <map>
#include <libMultiRobotPlanning/timer.hpp>
#include "a_star.hpp"
#include "dynamic_hungarian_assignment.hpp"

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
        CBSTA(Environment &environment) : m_env(environment) { }

        int id;
        double p_ta_time, p_lowlevel_search_time, p_lowlevel_queue_time, p_create_low_level_node_time;
        double avg_next_ta_time, d_cost;

        bool search(const std::vector<State> &initialStates,
                    std::vector<PlanResult<State, Action, Cost> > &solution) {
            HighLevelNode start;
            size_t numAgents = initialStates.size();
            start.solution.resize(numAgents);
            start.constraints.resize(numAgents);
            start.cost = 0;
            start.id = 0;
            Timer timer, timer_ta, timer_lowlevel_search, timer_lowlevel_queue, timer_low_level_node;
            p_ta_time = 0;
            p_lowlevel_search_time = 0;
            p_lowlevel_queue_time = 0;
            p_create_low_level_node_time = 0;




            for (size_t i = 0; i < numAgents; ++i) {
                for (auto goal: m_env.m_agents_goals[i]) {
                    timer_lowlevel_search.reset();
                    Task tmp_task(goal.x, goal.y);
                    LowLevelEnvironment llenv(m_env, i, start.constraints[i], &tmp_task);
                    LowLevelSearch_t lowLevel(llenv);
                    PlanResult<State, Action, Cost> tmp_solution;
                    bool success = lowLevel.search(initialStates[i], tmp_solution);
                    timer_lowlevel_search.stop();
                    p_lowlevel_search_time += timer_lowlevel_search.elapsedSeconds();


                    if (!success) tmp_solution.cost = 1e9;
                    timer_low_level_node.reset();
                    start.cost_matrix[std::make_pair<>(i, tmp_task)] = tmp_solution.cost;
                    start.cost_matrix_path[std::make_pair<>(i, tmp_task)] = tmp_solution;
                    timer_low_level_node.stop();
                    p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();

                }
            }






            timer_ta.reset();
            // First Assignment
            m_env.nextCostMatrixAssignment(start.cost_matrix, start.tasks);
// ----------------------------------------------
//            start.new_assignment = m_env.new_assignment;
//            start.newFirstTaskAssignment(m_env);
            timer_ta.stop();
            p_ta_time += timer_ta.elapsedSeconds();


            // find individual paths using low-level()
            for (size_t i = 0; i < numAgents; ++i) {

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
//                printf("%d --> (%d, %d)\n", i, start.task(i)->x, start.task(i)->y);
                start.cost += start.solution[i].cost;
            }

            d_cost = start.cost;

            typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                    boost::heap::mutable_<true> >
                    open;


            timer_lowlevel_queue.reset();
            auto handle = open.push(std::move(start));
            (*handle).handle = handle;
            timer_lowlevel_queue.stop();
            p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();



            timer.reset();
            avg_next_ta_time = 0;
            double last_time = timer.elapsedSeconds();
            int new_ta_cnt = 0;

            solution.clear();
            this->id = 1;
            while (!open.empty()) {
                // P ← the best node from OPEN
                // the lowest solution cost
                timer_low_level_node.reset();
                HighLevelNode P = std::move(open.top());
                timer_low_level_node.stop();
                p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();


                timer_lowlevel_queue.reset();
                open.pop();
                timer_lowlevel_queue.stop();
                p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();


                // Validate the paths in P until a conflict occurs.
                Conflict conflict;
                if (!m_env.getFirstConflict(P.solution, conflict)) {
                    std::cout << "done; cost: " << P.cost << std::endl;
                    solution = P.solution;
                    d_cost = P.cost - d_cost;
                    return true;
                }

                std::map<size_t, Constraints> constraints;
                m_env.createConstraintsFromConflict(conflict, constraints);

                for (unsigned short cur_i = 0; const auto &c: constraints) {
                    cur_i ++;
                    size_t i = c.first;

                    timer_low_level_node.reset();
                    HighLevelNode newNode;
                    timer_low_level_node.stop();
                    p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();

                    std::map<size_t, Task> org_tasks = P.tasks;
                    if (cur_i == constraints.size()) newNode = std::move(P);
                        else newNode = P;
                    newNode.id = this->id;
                    assert(!newNode.constraints[i].overlap(c.second));

                    newNode.constraints[i].add(c.second);

                    bool success = false;
                    for (auto goal: m_env.m_agents_goals[i]) {

                        timer_lowlevel_search.reset();
                        Task tmp_task(goal.x, goal.y);
                        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                                  &tmp_task);
                        LowLevelSearch_t lowLevel(llenv);
                        PlanResult <State, Action, Cost> tmp_solution;
                        success = lowLevel.search(initialStates[i], tmp_solution);
                        timer_lowlevel_search.stop();
                        p_lowlevel_search_time += timer_lowlevel_search.elapsedSeconds();

                        if (!success) {
                            break;
                        }
                        timer_low_level_node.reset();
                        newNode.cost_matrix[std::make_pair<>(i, tmp_task)] = tmp_solution.cost;
                        newNode.cost_matrix_path[std::make_pair<>(i, tmp_task)] = tmp_solution;
                        timer_low_level_node.stop();
                        p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();
                    }
                    if (!success) continue;

                    timer_ta.reset();
                    m_env.nextCostMatrixAssignment(newNode.cost_matrix, newNode.tasks);
//                    newNode.incrementalTaskAssignmentX(m_env, i);
                    timer_ta.stop();
                    p_ta_time += timer_ta.elapsedSeconds();

                    newNode.cost = 0;
                    for (size_t j = 0; j < numAgents; ++j) {
                        std::pair<size_t, Task> x;
                        x.first = j;
                        x.second = *newNode.task(j);

                        timer_low_level_node.reset();
                        newNode.solution[j] = newNode.cost_matrix_path[x];
                        newNode.cost += newNode.solution[j].cost;
                        timer_low_level_node.stop();
                        p_create_low_level_node_time += timer_low_level_node.elapsedSeconds();


                        auto iter = org_tasks.find(j);
                        if (iter != org_tasks.end()) {
                            if (!(x.second == iter->second))
                            {
                                timer.stop();
                                double new_time = timer.elapsedSeconds();
                                avg_next_ta_time = avg_next_ta_time / (new_ta_cnt + 1) * new_ta_cnt + (new_time-last_time) / (new_ta_cnt + 1);
//                                std::cout<<"avg_next_ta_time: "<<avg_next_ta_time<< std::endl;
                                last_time = new_time;
                                new_ta_cnt ++ ;
                            }
                        }

                    }
                    timer_lowlevel_queue.reset();
                    auto handle = open.push(std::move(newNode));
                    ++(this->id);
                    (*handle).handle = handle;
                    timer_lowlevel_queue.stop();
                    p_lowlevel_queue_time += timer_lowlevel_queue.elapsedSeconds();

                }
            }

            return false;
        }


        struct HighLevelNode {
            std::vector<PlanResult<State, Action, Cost> > solution;
            std::vector<Constraints> constraints;
            std::map<size_t, Task> tasks; // maps from index to task (and does not contain an entry if no task was assigned)
            std::map<std::pair<size_t, Task>, long> cost_matrix; // individual cost matrix
            std::map<std::pair<size_t, Task>, PlanResult<State, Action, Cost> > cost_matrix_path;
//            DynamicHungarianAssignment<size_t, Task> new_assignment;
            Cost cost;
            int id;
            typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                    boost::heap::mutable_<true> >::handle_type
                    handle;

            bool operator<(const HighLevelNode &n) const {
                // if (cost != n.cost)
                return cost > n.cost;
                // return id > n.id;
            }

//            void newFirstTaskAssignment(Environment& env)
//            {
//                if (env.m_numTaskAssignments > env.m_maxTaskAssignments) {
//                    return;
//                }
//
//                int64_t cost = new_assignment.firstSolution(cost_matrix, tasks);
//                if (!tasks.empty()) {
//                    std::cout << "newFirstTaskAssignment: cost: " << cost << std::endl;
//                    ++env.m_numTaskAssignments;
//                }
//            }

//            void incrementalTaskAssignmentX(Environment& env, size_t u)
//            {
//                if (env.m_numTaskAssignments > env.m_maxTaskAssignments) {
//                    return;
//                }
//                int64_t cost = new_assignment.incrementalSolutionX(cost_matrix, tasks, u);
//                if (!tasks.empty()) {
//                    ++env.m_numTaskAssignments;
//                }
//            }

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
    private:
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