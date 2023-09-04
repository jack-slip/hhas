#include <hhas/se2_grid_a_star.h>
#include <hhas/se2_viz.h>

int main()
{
    int width = 1000;
    int height = 1000;
    float density = 0.1;

    hhas::states::SE2GridState start = {0, 0, 0};
    hhas::states::SE2GridState goal = {925, 925, 90};

    auto grid = std::vector<std::vector<uint8_t>>(
        width, std::vector<uint8_t>(height, 0));

    // place obstacles at 2,2 and 3,3
    // Place random obstacles, but not at the start or goal
    // seed rand
    srand(time(NULL));

    int num_obstacles = density * width * height;

    for (int i = 0; i <= num_obstacles; i++)
    {
        int x = rand() % width;
        int y = rand() % height;
        if ((x == start.x && y == start.y) || (x == goal.x && y == goal.y))
        {
            continue;
        }

        grid[y][x] = 1;
    }

    hhas::SE2GridAStar planner(width, height, grid);
    // grid.clear();
    auto path = planner.plan(start, goal);

    for (auto &state : path)
    {
        std::cout << "(" << state.x << ", " << state.y << ", " << state.theta << ")" << std::endl;
        assert(grid[state.y][state.x] != 1);
    }

    // hhas::viz::drawGridAndPath(grid, path, 1);

    return 0;
}