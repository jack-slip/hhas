#include <hhas/grid_a_star.h>

int main()
{
    int width = 10;
    int height = 10;
    auto grid = std::vector<std::vector<uint8_t>>(
        height, std::vector<uint8_t>(width, 0));

    // place obstacles at 2,2 and 3,3
    grid[2][2] = 1;
    grid[3][3] = 1;

    hhas::GridAStar planner(10, 10, std::move(grid));
    grid.clear();
    auto path = planner.plan({0, 0}, {9, 8});

    for (auto &state : path)
    {
        std::cout << "(" << state.x << ", " << state.y << ")" << std::endl;
    }
    return 0;
}