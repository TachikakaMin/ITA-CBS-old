//
// Created by YIMIN TANG on 2022/10/18.
//
#pragma once

#include <queue>
#include <set>
#include <cmath>
#include <vector>
#include "timer.hpp"
#include <libMultiRobotPlanning/cbs_tym.hpp>

namespace libMultiRobotPlanning {
    template <typename Agent, typename Task>
    class DynamicHungarianAssignment {

    public:
        std::map<std::pair<Agent, Task>, long> m_cost;
        std::map<Task, int> task_set;
        std::map<int, Task> task_set_reverse;
        int cnt_task;
        int org_n, org_m, n;
        std::vector<std::vector<long> > W;
        std::vector<int> mateL, mateR, p;
        std::vector<long> lx, ly, slack;
        std::vector<bool> m;
        long inf;

        DynamicHungarianAssignment()
        {
            task_set.clear();
            cnt_task = 0;
            inf = std::numeric_limits<long>::max();
        }

        void initHungarian(const std::vector<State> & startStates,
                           const std::vector<std::unordered_set<Task> > & goals)
        {
            org_n = startStates.size();
            std::vector<Task> all_goals;
            for (size_t i = 0; i < startStates.size(); ++i) {
                for (const auto &goal:  goals[i])
                    all_goals.push_back(goal);

            }
            std::sort(all_goals.begin(), all_goals.end());
            for (const auto &goal:  all_goals)
                if (task_set.find(goal) == task_set.end()) {
                    task_set[goal] = cnt_task;
                    task_set_reverse[cnt_task] = goal;
                    cnt_task++;
                }

            org_m = cnt_task;
            n = std::max(org_m, org_n);
            W = std::vector<std::vector<long> >(n, std::vector<long>(n));
            mateL = std::vector<int>(n, -1);
            mateR = std::vector<int>(n, -1);
            p = std::vector<int>(n);
            lx = std::vector<long>(n, -inf);
            ly = std::vector<long>(n);
            slack = std::vector<long>(n);
            m = std::vector<bool>(n);
        }

        void addEdge(int u, int v, int w) {
            W[u][v] = 1e9 - w;
        }

        void augment(int j) {
            int i, next;
            do {
                i = p[j];
                mateR[j] = i;
                next = mateL[i];
                mateL[i] = j;
                if (next != -1) j = next;
            } while (next != -1);
        }

        int64_t solve_hungarian() {
            int vrex = 0;
            for (int i = 0; i < n; i++) if (mateL[i] == -1) vrex++;
            while (vrex > 0) {
                for (int i = 0; i < n; i++) {
                    m[i] = false;
                    p[i] = -1;
                    slack[i] = inf;
                }
                const int MAXV = n;
                bool aug = false, Q[MAXV];
                int numQ = 0;
                memset(Q, 0, sizeof(Q));
                for (int i = 0; i < n; i++)
                    if (mateL[i] == -1) {
                        Q[i] = true;
                        numQ++;
                    }

                do {
                    int i, j;
                    for (int k = 0; k < n; k++)
                        if (Q[k]) {
                            i = k;
                            numQ--;
                            Q[i] = false;
                            m[i] = true;
                            j = 0;
                            break;
                        }

                    while (aug == false && j < n) {
                        if (mateL[i] != j) {
                            if (lx[i] + ly[j] - W[i][j] < slack[j]) {
                                slack[j] = lx[i] + ly[j] - W[i][j];
                                p[j] = i;
                                if (slack[j] == 0) {
                                    if (mateR[j] == -1) {
                                        augment(j);
                                        aug = true;
                                        vrex--;
                                    } else {
                                        if (Q[mateR[j]] == false) {
                                            Q[mateR[j]] = true;
                                            numQ++;
                                        }
                                    }
                                }
                            }
                        }
                        j++;
                    }

                    if (aug == false && numQ == 0) {
                        long tao = inf;
                        for (int k = 0; k < n; k++)
                            if (slack[k] > 0)
                                tao = std::min(tao, slack[k]);
                        for (int k = 0; k < n; k++)
                            if (m[k])
                                lx[k] -= tao;

                        int x = -1;
                        bool X[MAXV];
                        for (int k = 0; k < n; k++)
                            if (slack[k] == 0)
                                ly[k] += tao;
                            else {
                                slack[k] -= tao;
                                if (slack[k] == 0 && mateR[k] == -1) x = k;
                                if (slack[k] == 0) X[k] = true;
                                else X[k] = false;
                            }

                        if (x == -1) {
                            for (int k = 0; k < n; k++)
                                if (X[k]) {
                                    Q[mateR[k]] = true;
                                    numQ++;
                                }
                        } else {
                            augment(x);
                            aug = true;
                            vrex--;
                        }
                    }
                } while (aug == false);
            }

            int64_t ans = 0;
            for (int i = 0; i < org_n; i++) {
                ans += 1e9 - (lx[i] + ly[i]);
            }
            return ans;
        }


        void create_cost_matrix(const std::map<std::pair<size_t, Task>, long>& cost_matrix)
        {
            for (const auto& [key, value] : cost_matrix) {
                size_t agent_idx = key.first;
                size_t task_idx = task_set[key.second];
                addEdge(agent_idx, task_idx, value);
            }
        }


        int64_t firstSolution(const std::map<std::pair<size_t, Task>, long>& cost_matrix,
                              std::map<size_t, Task>& out_tasks)
        {
            create_cost_matrix(cost_matrix);
            for(int i=0;i<n;i++)
            {
                for(int j=0;j<n;j++) lx[i] = std::max(lx[i], W[i][j]);
                ly[i] = 0;
            }
            int64_t ans = solve_hungarian();
            for (int i = 0; i < org_n; i++) {
                int task_idx = mateL[i];
                Task goal = task_set_reverse[task_idx];
                out_tasks[i] = goal;
            }
            return ans;
        }

        int64_t incrementalSolutionX(const std::map<std::pair<size_t, Task>, long>& cost_matrix,
                                    std::map<size_t, Task>& out_tasks,
                                    const size_t & u)
        {
            create_cost_matrix(cost_matrix);
            mateR[ mateL[u] ] = -1; mateL[u] = -1;
            lx[u] = -inf; for(int i=0;i<n;i++) lx[u] = std::max(lx[u], W[u][i]-ly[i]);
            int64_t ans = solve_hungarian();
            for (int i = 0; i < org_n; i++) {
                int task_idx = mateL[i];
                Task goal = task_set_reverse[task_idx];
                out_tasks[i] = goal;
            }
            return ans;
        }
    };
}


//#ifndef LIBMULTIROBOTPLANNING_DYNAMIC_HUNGARIAN_ASSIGNMENT_HPP
//#define LIBMULTIROBOTPLANNING_DYNAMIC_HUNGARIAN_ASSIGNMENT_HPP
//
//#endif //LIBMULTIROBOTPLANNING_DYNAMIC_HUNGARIAN_ASSIGNMENT_HPP
