#ifndef HHAS_T2_KD_TREE_H
#define HHAS_T2_KD_TREE_H

#include <nanoflann.hpp>
#include <hhas/t2_kd_tree/t2_kd_tree_adapter.h>
#include <vector>

namespace hhas
{
    class T2KDTree
    {
    public:
        using DIMS = std::integral_constant<int, 2>;
        using KDData_t = std::vector<std::vector<float>>;
        using KDTree_t = detail::KDTreeVectorOfVectorsAdaptor<KDData_t, float, DIMS::value>;
        T2KDTree(const KDData_t &occupied_indices, const int leaf_max_size = 10, const int num_threads_build = 1) : occ_indices_(occupied_indices)
        {
            // Build the KD tree
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            kdtree_ = std::make_unique<KDTree_t>(DIMS::value, occ_indices_, leaf_max_size, num_threads_build);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "KDTree build time = "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]"
                      << std::endl;
        }

        std::vector<float> getIndex(const int i) const
        {
            return occ_indices_[i];
        }

        void radiusSearch(const float *query_pt, const float radius, std::vector<nanoflann::ResultItem<size_t, float>> &indices_dists, const nanoflann::SearchParameters &search_params = nanoflann::SearchParameters())
        {
            kdtree_->index->radiusSearch(query_pt, radius, indices_dists, search_params);
        }

    private:
        KDData_t occ_indices_; // stored as (i, j) pairs
        std::unique_ptr<KDTree_t> kdtree_{nullptr};
    };
} // namespace hhas

#endif // HHAS_T2_KD_TREE_H