#ifndef HHAS_SE2_VIZ_H
#define HHAS_SE2_VIZ_H
#include <opencv2/opencv.hpp>
#include <hhas/se2_grid_a_star.h>
#include <hhas/geometry.h>

namespace hhas
{
    namespace viz
    {
        void vizWaitKey(int delay = 0)
        {
            cv::waitKey(delay);
        }

        inline void drawGridAndPath(const std::vector<std::vector<uint8_t>> &grid,
                                    const std::vector<hhas::states::SE2GridState> &path,
                                    const hhas::geometry::Rectangle &footprint)
        {
            int rows = grid.size();
            int cols = grid[0].size();

            // Create an OpenCV image (3 channels for RGB)
            cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));

            // Draw the grid
            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    if (grid[i][j] == 1)
                    {
                        image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // Set pixel color to black
                    }
                }
            }

            // Draw the path with footprint
            int j = 0;
            int last_index = path.size() - 1;
            for (const auto &state : path)
            {
                //   draw the path
                image.at<cv::Vec3b>(state.y, state.x) = cv::Vec3b(0, 0, 255); // Set pixel color to red

                // convert theta to radians duh
                float theta = state.theta * M_PI / 180.0;
                // std::cout << "theta: " << theta << std::endl;
                auto fp = footprint;
                fp.move(state.x, state.y, theta);
                // print the state
                // std::cout << "Path State: (" << state.x << ", " << state.y << ", " << state.theta << ")" << std::endl;
                // // print the vertices for now
                // for (const auto &point : vertices)
                // {
                //     std::cout << "  (" << point.c[0] << ", " << point.c[1] << ")" << std::endl;
                // }
                // draw the footprint, ensure that the vertices are in the image, otherwise just skip that footprint
                //  only draw like every 5th point, make sure to always draw first and last
                if (j % 5 != 0 and j != 0 and j != last_index)
                {
                    j++;
                    continue;
                }
                j++;
                auto vertices = fp.getVertices();
                for (size_t i = 0; i < vertices.size(); ++i)
                {

                    auto point = vertices[i];
                    if (point.c[0] < 0 || point.c[0] >= cols || point.c[1] < 0 || point.c[1] >= rows)
                    {
                        continue;
                    }
                    // draw a line between this point and the next point
                    auto next_point = vertices[(i + 1) % vertices.size()];
                    cv::line(image, cv::Point(point.c[0], point.c[1]), cv::Point(next_point.c[0], next_point.c[1]), cv::Scalar(0, 0, 255), 1);
                }
            }

            // Resize for better visualization
            // cv::Mat resizedImage;
            // cv::resize(image, resizedImage, cv::Size(), 2, 2, cv::INTER_NEAREST); // Assuming a scale factor of 2 for now

            // Display the image
            cv::imshow("Grid and Path", image);
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

#endif // HHAS_SE2_VIZ_H