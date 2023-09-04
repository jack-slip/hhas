#ifndef HHAS_SE2_GRID_A_STAR_FP_H
#define HHAS_SE2_GRID_A_STAR_FP_H

#include <hhas/a_star.h>
#include <hhas/states/se2_grid_state.h>
#include <cmath>
#include <array>
#include <hhas/geometry.h>
#include <type_traits>
#include <shared_mutex>

#include <hhas/t2_kd_tree/t2_kd_tree.h>

#include <thread>
#include <omp.h>

namespace hhas
{
    namespace detail
    {
        template <typename...>
        using void_t = void;

        template <class, template <class> class, class = void_t<>>
        struct detect : std::false_type
        {
        };

        template <class T, template <class> class Op>
        struct detect<T, Op, void_t<Op<T>>> : std::true_type
        {
        };

        // Check for inside method with proper signature
        template <class T>
        using inside_expr = decltype(std::declval<T>().inside(std::declval<const geometry::Vec2 &>()));

        template <class T>
        using has_inside_method = detect<T, inside_expr>;

        template <class T>
        constexpr bool has_inside_method_v = has_inside_method<T>::value;
    }

    template <class Footprint>
    class SE2GridAStarFootprint : public AStar<SE2GridAStarFootprint<Footprint>, states::SE2GridState>
    {
    public:
        static_assert(detail::has_inside_method_v<Footprint>, "Footprint type must have a method bool Footprint::inside(const Vec2 &)");

        SE2GridAStarFootprint(const size_t width, const size_t height, const std::vector<std::vector<uint8_t>> &grid, const Footprint &footprint)
            : width_(width), height_(height), grid_(grid), footprint_(footprint)
        {
            omp_set_num_threads(std::thread::hardware_concurrency());
            constexpr uint8_t occupied_value = 1u;
            constexpr int leaf_max_size = 10;
            int num_threads_build = std::thread::hardware_concurrency();

            auto occupied_indices = detail::toOccupiedIndices(grid_, occupied_value, [](const uint8_t a, const uint8_t b)
                                                              { return a == b; });

            // for each occupied index, assert that the value in grid is occupied_value
            for (const auto &index : occupied_indices)
            {
                int i = index[0];
                int j = index[1];
                assert(grid_[i][j] == occupied_value);
            }

            kd_tree_ = std::make_unique<T2KDTree>(occupied_indices, leaf_max_size, num_threads_build);

            feasibility_grid_ = std::vector<std::vector<std::vector<int8_t>>>(
                width, std::vector<std::vector<int8_t>>(
                           height, std::vector<int8_t>(depth_, -1)));

            computeFeasibilityGrid();
        }

        using State = states::SE2GridState;

        double cost(const State &start, const State &goal)
        {
            // Assuming cost is Euclidean distance
            double dx = start.x - goal.x;
            double dy = start.y - goal.y;
            double dtheta_normalized = std::min(std::abs(start.theta - goal.theta), 360 - std::abs(start.theta - goal.theta));
            double dist_cost = dx * dx + dy * dy;
            return dist_cost + 0.5 * (dtheta_normalized * dtheta_normalized);
        }

        double heuristic(const State &start, const State &goal)
        {
            // Using Euclidean distance as heuristic
            return cost(start, goal);
        }

        std::vector<State> neighbors(const State &state)
        {
            // Assuming 8 connected grid
            static std::vector<std::vector<int>> offsets = {
                {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

            std::vector<State> neighbors;
            neighbors.reserve(offsets.size() * 7UL + 7UL);
            for (const auto &offset : offsets)
            {
                int x = state.x + offset[0]; // offset.first
                int y = state.y + offset[1]; // offset.second
                if (x >= 0 && x < width_ && y >= 0 && y < height_)
                {
                    // delta_theta is 15 degrees, discretized in 5 degree increments
                    for (int delta_theta = -15; delta_theta <= 15; delta_theta += 5)
                    {
                        int theta = state.theta + delta_theta;
                        if (theta < 0)
                        {
                            theta += 360;
                        }
                        else if (theta >= 360)
                        {
                            theta -= 360;
                        }
                        int theta_idx = theta / 5;
                        if (feasibility_grid_[y][x][theta_idx] == 0)
                            neighbors.push_back({x, y, theta});
                    }
                }
            }
            // allow rotation on same square
            for (int delta_theta = -15; delta_theta <= 15; delta_theta += 5)
            {
                int theta = state.theta + delta_theta;
                if (theta < 0)
                {
                    theta += 360;
                }
                else if (theta >= 360)
                {
                    theta -= 360;
                }
                int theta_idx = theta / 5;
                if (feasibility_grid_[state.y][state.x][theta_idx] == 0)
                    neighbors.push_back({state.x, state.y, theta});
            }
            return neighbors;
        }

        const std::vector<std::vector<std::vector<int8_t>>> &getFeasibilityGrid() const
        {
            return feasibility_grid_;
        }

    private:
        size_t width_, height_;
        size_t depth_{360 / 5};
        std::vector<std::vector<uint8_t>> grid_;

        std::shared_mutex feasibility_grid_mutex_;
        std::vector<std::vector<std::vector<int8_t>>> feasibility_grid_; // -1 is uncalculated, 0 is infeasible, 1 is feasible

        Footprint footprint_;

        std::unique_ptr<T2KDTree> kd_tree_{nullptr};

        void computeFeasibilityGrid()
        {
            // assume circle for now

            std::unique_lock<std::shared_mutex> lock(feasibility_grid_mutex_);
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

#pragma omp parallel for shared(feasibility_grid_)
            for (size_t i = 0; i < height_; ++i)
            {
                for (size_t j = 0; j < width_; ++j)
                {
                    float x = j;
                    float y = i;
                    if (isColliding(x, y, 0))
                    {
                        std::fill(feasibility_grid_[i][j].begin(), feasibility_grid_[i][j].end(), 1);
                    }
                    else
                        std::fill(feasibility_grid_[i][j].begin(), feasibility_grid_[i][j].end(), 0);
                }
            }

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time to compute feasibility grid = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
            std::cout << "Infeasibility grid size (mB) = " << (width_ * height_ * depth_ * sizeof(int8_t)) / 1e6f << std::endl;
        }

        bool isColliding(const float x, const float y, const float theta)
        {
            geometry::Circle transformed_footprint = footprint_;
            transformed_footprint.move(
                x, y); // make it centered and oriented at pose

            // make the query point raw array
            float query_idx[2] = {y, x};

            auto search_params = nanoflann::SearchParameters();
            search_params.sorted = false;

            float radius = footprint_.r;

            std::vector<nanoflann::ResultItem<size_t, float>> indices_dists;
            kd_tree_->radiusSearch(&query_idx[0], radius * radius, indices_dists, search_params);

            if (indices_dists.size() > 0)
                return true;
            return false;
        }
    };

} // namespace hhas

#endif // HHAS_SE2_GRID_A_STAR_H