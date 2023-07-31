#include <opencv2/opencv.hpp>
#include <hhas/se2_grid_a_star.h>

namespace hhas
{
    namespace viz
    {
        void vizWaitKey(int delay = 0)
        {
            cv::waitKey(delay);
        }

        inline void drawGridAndPath(const std::vector<std::vector<uint8_t>> &grid,
                                    const std::vector<hhas::states::SE2GridState> &path, int upsample_factor = 1)
        {
            if (path.empty())
            {
                std::cout << "No path found" << std::endl;
                return;
            }
            // create an image from the grid
            cv::Mat image(grid.size() * upsample_factor, grid[0].size() * upsample_factor, CV_8UC3, cv::Scalar(255, 255, 255));
            for (size_t i = 0; i < grid.size(); ++i)
            {
                for (size_t j = 0; j < grid[i].size(); ++j)
                {
                    if (grid[i][j] == 1)
                    { // if there's an obstacle, color the pixel black
                        image.at<cv::Vec3b>(i * upsample_factor, j * upsample_factor) = cv::Vec3b(0, 0, 0);
                    }
                }
            }

            // draw the path
            for (size_t i = 0; i < path.size(); ++i)
            {
                cv::Point start(path[i].x * upsample_factor, path[i].y * upsample_factor);

                // // Compute the direction of the arrow (since theta is just an int,
                // // we'll treat it as a rotation in degrees around the origin)
                // double radian_theta = path[i].theta * M_PI / 180.0;
                // cv::Point end(start.x + std::cos(radian_theta) * upsample_factor * 2, start.y + std::sin(radian_theta) * upsample_factor * 2);

                // cv::arrowedLine(image, start, end, cv::Scalar(0, 0, 255), 2);
                // just draw the point for now
                // cv::circle(image, start, upsample_factor, cv::Scalar(0, 0, 255), -1);
                image.at<cv::Vec3b>(start.y, start.x) = cv::Vec3b(0, 0, 255);
            }

            // Draw start and end states (green and blue circles, respectively)
            cv::circle(image, cv::Point(path.front().x * upsample_factor, path.front().y * upsample_factor), upsample_factor, cv::Scalar(0, 255, 0), -1);
            cv::circle(image, cv::Point(path.back().x * upsample_factor, path.back().y * upsample_factor), upsample_factor, cv::Scalar(255, 0, 0), -1);

            // show the image
            cv::imshow("Path", image);
        }

        template <typename State_T>
        void visualizeFeasibleStates(const std::vector<std::vector<std::vector<int8_t>>> &feas_map, const std::vector<std::vector<uint8_t>> &occ, const State_T &start, const State_T &goal, int upsample_factor = 1)
        {
            cv::Mat image(feas_map.size() * upsample_factor, feas_map[0].size() * upsample_factor, CV_8UC3, cv::Scalar(255, 255, 255));
            for (size_t i = 0; i < feas_map.size(); ++i)
            {
                for (size_t j = 0; j < feas_map[i].size(); ++j)
                {
                    if (occ[i][j] == 1)
                    { // if there's an obstacle
                        cv::rectangle(image,
                                      cv::Rect(j * upsample_factor, i * upsample_factor, upsample_factor, upsample_factor),
                                      cv::Scalar(0, 0, 0),
                                      cv::FILLED);
                    }
                    // 0 is feasible, 1 is infeasible
                    else if (feas_map[i][j][0] == 0)
                    {
                        cv::rectangle(image,
                                      cv::Rect(j * upsample_factor, i * upsample_factor, upsample_factor, upsample_factor),
                                      cv::Scalar(0, 255, 0),
                                      cv::FILLED);
                    }
                    else if (feas_map[i][j][0] == 1)
                    {
                        cv::rectangle(image,
                                      cv::Rect(j * upsample_factor, i * upsample_factor, upsample_factor, upsample_factor),
                                      cv::Scalar(0, 0, 255),
                                      cv::FILLED);
                    }
                }
            }
            // mark the start and goal states, make start white and goal blue
            cv::circle(image, cv::Point(start.x * upsample_factor, start.y * upsample_factor), upsample_factor, cv::Scalar(255, 255, 255), -1);
            cv::circle(image, cv::Point(goal.x * upsample_factor, goal.y * upsample_factor), upsample_factor, cv::Scalar(255, 0, 0), -1);

            cv::imshow("Feasible States", image);
        }

    } // namespace viz

} // namespace hhas
