#ifndef HHAS_SE2_GRID_STATE_H
#define HHAS_SE2_GRID_STATE_H

#include <functional>
#include <unordered_map>
#include <vector>

namespace hhas
{
    namespace states
    {
        template <typename State>
        struct StateTraits;

        struct SE2GridState
        {
            int x, y, theta;

            bool operator==(const SE2GridState &other) const
            {
                return x == other.x && y == other.y && theta == other.theta;
            }
        };
        namespace detail
        {
            struct SE2GridStateHash
            {
                std::size_t operator()(const SE2GridState &state) const
                {
                    return std::hash<int>{}(state.x) ^ (std::hash<int>{}(state.y) << 1) ^ (std::hash<int>{}(state.theta) << 2);
                }
            };

            struct SE2GridStateEqual
            {
                bool operator()(const SE2GridState &lhs, const SE2GridState &rhs) const
                {
                    return lhs == rhs;
                }
            };

            struct SE2GridStatePQCompare
            {
                bool operator()(const std::pair<SE2GridState, double> &lhs,
                                const std::pair<SE2GridState, double> &rhs) const
                {
                    return lhs.second > rhs.second;
                }
            };
        } // namespace detail

        template <>
        struct StateTraits<SE2GridState>
        {
            using Hash = detail::SE2GridStateHash;
            using Equal = detail::SE2GridStateEqual;
            using PQCompare = detail::SE2GridStatePQCompare;
        };
    }

}

#endif // HHAS_SE2_GRID_STATE_H