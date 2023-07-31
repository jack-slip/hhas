#include <hhas/se2_grid_a_star_fp.h>
#include <hhas/se2_viz.h>

int main()
{
    int width = 250;
    int height = 250;
    float density = 0.00125;

    hhas::states::SE2GridState start = {0, 0, 0};
    hhas::states::SE2GridState goal = {200, 200, 0};

    auto grid = std::vector<std::vector<uint8_t>>(
        width, std::vector<uint8_t>(height, 0));

    // place obstacles at 2,2 and 3,3
    // Place random obstacles, but not at the start or goal
    // seed rand
    srand(time(NULL));

    int num_obstacles = density * static_cast<float>(width) * static_cast<float>(height);

    for (int i = 0; i <= num_obstacles; i++)
    {
        int x = rand() % width;
        int y = rand() % height;
        if ((x == start.x && y == start.y) || (x == goal.x && y == goal.y))
        {
            continue;
        }

        grid[y][x] = 1; // 1 is obstacle
    }

    hhas::geometry::Circle footprint(hhas::geometry::Vec2{0, 0}, 10);

    hhas::SE2GridAStarFootprint planner(width, height, grid, footprint);

    // grid.clear();
    int timeout_ms = 1000;
    auto path = planner.plan(start, goal, timeout_ms);

    for (auto &state : path)
    {
        // std::cout << "(" << state.x << ", " << state.y << ", " << state.theta << ")" << std::endl;
        assert(planner.getFeasibilityGrid()[state.y][state.x][state.theta / 5] != 1); // 1 is infesible
    }

    hhas::viz::drawGridAndPath(grid, path, 1);
    hhas::viz::visualizeFeasibleStates(planner.getFeasibilityGrid(), grid, start, goal, 1);
    hhas::viz::vizWaitKey();

    return 0;
}