#ifndef HHAS_STATES_H
#define HHAS_STATES_H

#include <functional>
#include <unordered_map>
#include <vector>

namespace hhas
{
    namespace states
    {
        template <typename State>
        struct StateTraits;

        struct T2GridState
        {
            int x, y;

            bool operator==(const T2GridState &other) const
            {
                return x == other.x && y == other.y;
            }
        };
        namespace detail
        {
            struct T2GridStateHash
            {
                std::size_t operator()(const T2GridState &state) const
                {
                    return std::hash<int>{}(state.x) ^ (std::hash<int>{}(state.y) << 1);
                }
            };

            struct T2GridStateEqual
            {
                bool operator()(const T2GridState &lhs, const T2GridState &rhs) const
                {
                    return lhs == rhs;
                }
            };

            struct T2GridStatePQCompare
            {
                bool operator()(const std::pair<T2GridState, double> &lhs,
                                const std::pair<T2GridState, double> &rhs) const
                {
                    return lhs.second > rhs.second;
                }
            };
        } // namespace detail

        template <>
        struct StateTraits<T2GridState>
        {
            using Hash = detail::T2GridStateHash;
            using Equal = detail::T2GridStateEqual;
            using PQCompare = detail::T2GridStatePQCompare;
        };

    }

}

#endif // HHAS_STATES_H