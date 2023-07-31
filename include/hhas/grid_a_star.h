#include <hhas/a_star.h>
#include <hhas/states/t2_grid_state.h>
#include <cmath>
#include <array>

namespace hhas
{

    class GridAStar : public AStar<GridAStar, states::T2GridState>
    {
    public:
        GridAStar(const size_t width, const size_t height, const std::vector<std::vector<uint8_t>> &grid)
            : width_(width), height_(height), grid_(grid)
        {
        }
        GridAStar(const size_t width, const size_t height, std::vector<std::vector<uint8_t>> &&grid)
            : width_(width), height_(height), grid_(std::move(grid))
        {
        }
        using State = states::T2GridState;

        double cost(const State &start, const State &goal)
        {
            // Assuming cost is Euclidean distance
            double dx = start.x - goal.x;
            double dy = start.y - goal.y;
            return dx * dx + dy * dy;
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
            neighbors.reserve(offsets.size());
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
                    neighbors.push_back({x, y});
                }
            }
            return neighbors;
        }

    private:
        size_t width_, height_;
        std::vector<std::vector<uint8_t>> grid_;
    };

} // namespace hhas
