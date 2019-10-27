#pragma once

#include <nav_msgs/OccupancyGrid.h>

namespace path_planning
{

// A 1D index.
using Index1D = int64_t;

// A 2D index, representing a column and a row.
struct Index2D
{
    // One-to-one constructor.
    Index2D(Index1D column_, Index1D row_) : column(std::move(column_)), row(std::move(row_))
    {
    }

    // The column for this index.
    Index1D column;

    // The row for this index.
    Index1D row;
};

// Converts a 1D index into a 2D index for the given map metadata.
Index2D index_1d_to_2d(const Index1D index_1d, const nav_msgs::MapMetaData &meta_data);

// Converts a 2D index into a point for the given map metadata.
geometry_msgs::Point index_2d_to_point(const Index2D &index_2d, const nav_msgs::MapMetaData &meta_data);

// Converts a 1D index into a point for the given map metadata.
geometry_msgs::Point index_1d_to_point(const Index1D &index_1d, const nav_msgs::MapMetaData &meta_data);

// Converts a point into a 2D index for the given map metadata.
Index2D point_to_index_2d(const geometry_msgs::Point &point, const nav_msgs::MapMetaData &meta_data);

// Converts a 2D index into a 1D index for the given map metadata.
Index1D index_2d_to_1d(const Index2D &index_2d, const nav_msgs::MapMetaData &meta_data);

// Converts a point into a 1D index for the given map metadata.
Index1D point_to_index_1d(const geometry_msgs::Point &point, const nav_msgs::MapMetaData &meta_data);

// Represents how two elements are connected within one another eight-neighborhood.
enum class EightConnectedNeighborType
{
    NONE,
    ADJACENT,
    DIAGONAL
};

// Get the connectivity between two elements.
EightConnectedNeighborType get_eight_connected_neighbor_type(const Index1D lhs, const Index1D rhs,
                                                             const nav_msgs::MapMetaData &meta_data);

// Get the elements in the given indices neighborhood.
std::vector<Index1D> get_eight_connected_neighbors(const Index1D index_1d, const nav_msgs::MapMetaData &meta_data);

// Computes a path on the given occupancy grid from `start` to `end`.
boost::optional<std::vector<geometry_msgs::Point>> a_star(const nav_msgs::OccupancyGrid &occupancy_grid,
                                                          const geometry_msgs::Point &start,
                                                          const geometry_msgs::Point &goal);

} // namespace path_planning
