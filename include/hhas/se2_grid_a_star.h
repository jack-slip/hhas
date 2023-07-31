#ifndef HHAS_SE2_GRID_A_STAR_H
#define HHAS_SE2_GRID_A_STAR_H

#include <hhas/a_star.h>
#include <hhas/states/se2_grid_state.h>
#include <cmath>
#include <array>

namespace hhas
{

    class SE2GridAStar : public AStar<SE2GridAStar, states::SE2GridState>
    {
    public:
        SE2GridAStar(const size_t width, const size_t height, const std::vector<std::vector<uint8_t>> &grid)
            : width_(width), height_(height), grid_(grid)
        {
        }
        SE2GridAStar(const size_t width, const size_t height, std::vector<std::vector<uint8_t>> &&grid)
            : width_(width), height_(height), grid_(std::move(grid))
        {
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
                    if (grid_[y][x] == 1)
                    {
                        continue;
                    }
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
                neighbors.push_back({state.x, state.y, theta});
            }
            return neighbors;
        }

    private:
        size_t width_, height_;
        std::vector<std::vector<uint8_t>> grid_;
    };

} // namespace hhas

#endif // HHAS_SE2_GRID_A_STAR_H