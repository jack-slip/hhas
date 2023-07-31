#ifndef HHAS_A_STAR_H
#define HHAS_A_STAR_H

#include <queue>
#include <functional>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <future>

namespace hhas
{
    // fwd decl
    namespace states
    {
        template <typename State>
        struct StateTraits;
    }

    template <class Derived, class State>
    class AStar
    {
    public:
        AStar() = default;
        AStar(const AStar &) = delete;
        AStar &operator=(const AStar &) = delete;
        AStar(AStar &&) = delete;
        AStar &operator=(AStar &&) = delete;
        ~AStar() = default;

        using StateTraits_t = states::StateTraits<State>;

        using PQ_t = std::priority_queue<std::pair<State, double>,
                                         std::vector<std::pair<State, double>>,
                                         typename StateTraits_t::PQCompare>;

        using CameFromGScore_t = std::unordered_map<State,
                                                    std::pair<State, double>,
                                                    typename StateTraits_t::Hash,
                                                    typename StateTraits_t::Equal>;

        std::vector<State> plan(const State &start_idx, const State &goal_idx, const int max_planning_time_ms = 1000)
        {

            // run A* on the hybridized search space
            auto plan_start = std::chrono::high_resolution_clock::now();
            PQ_t open_set{};
            // came from and g_cost map
            CameFromGScore_t cfg{0, typename StateTraits_t::Hash{}, typename StateTraits_t::Equal{}};

            auto &derived = static_cast<Derived &>(*this);

            open_set.emplace(start_idx, derived.heuristic(start_idx, goal_idx));
            cfg.emplace(start_idx, std::make_pair(start_idx, 0.0));

            int states_expanded = 0;
            while (!open_set.empty())
            {

                auto [curr_idx, _] = open_set.top();
                open_set.pop();
                states_expanded++;

                if (states_expanded % 10000 == 0)
                {
                    // check timeout
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - plan_start)
                            .count() > max_planning_time_ms)
                    {
                        std::cout << "Timeout reached" << std::endl;
                        break;
                    }
                }

                if (curr_idx == goal_idx)
                {
                    auto path = reconstructPath(cfg, curr_idx);
                    auto plan_end = std::chrono::high_resolution_clock::now();
                    std::cout
                        << "Planning time: "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(plan_end - plan_start).count()
                        << " ms" << std::endl;

                    std::cout << "States expanded: " << states_expanded << std::endl;

                    return path;
                }
                auto neighbors = derived.neighbors(curr_idx);
                for (const auto &nbor : neighbors)
                {
                    float tentative_g_score = cfg[curr_idx].second + derived.cost(curr_idx, nbor);
                    auto [iter, inserted] = cfg.emplace(nbor, std::make_pair(curr_idx, tentative_g_score));
                    if (!inserted)
                    {
                        // the element already existed
                        if (tentative_g_score < iter->second.second)
                        {
                            iter->second = std::make_pair(curr_idx, tentative_g_score);
                            open_set.emplace(nbor, tentative_g_score + derived.heuristic(nbor, goal_idx));
                        }
                    }
                    else
                    {
                        open_set.emplace(nbor, tentative_g_score + derived.heuristic(nbor, goal_idx));
                    }
                }
            }

            auto plan_end = std::chrono::high_resolution_clock::now();
            std::cout << "Planning time: "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(plan_end - plan_start).count()
                      << " ms" << std::endl;

            std::cout << "States expanded: " << states_expanded << std::endl;

            return {};
        }

        std::vector<State> reconstructPath(const CameFromGScore_t &came_from, const State &curr_idx)
        {
            auto curr_idx_copy = curr_idx;
            std::vector<State> path{};
            path.push_back(curr_idx_copy);

            while (
                // the start node has a self loop so this terminates the backtrace
                !(came_from.at(curr_idx_copy).first == curr_idx_copy))
            {
                curr_idx_copy = came_from.at(curr_idx_copy).first;
                path.push_back(curr_idx_copy);
            }

            std::reverse(path.begin(), path.end());

            return path;
        }
    };

} // namespace hhas

#endif // HHAS_HHAS_H
