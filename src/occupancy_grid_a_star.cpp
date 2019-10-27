#include <path_planning/occupancy_grid_a_star.h>

#include <path_planning/a_star.h>

namespace path_planning
{

// ######################################################################### //

Index2D index_1d_to_2d(const Index1D index_1d, const nav_msgs::MapMetaData &meta_data)
{
    const auto column = index_1d % meta_data.width;
    const auto row = index_1d / meta_data.width;
    return Index2D(column, row);
}

// ######################################################################### //

geometry_msgs::Point index_2d_to_point(const Index2D &index_2d, const nav_msgs::MapMetaData &meta_data)
{
    double x = meta_data.origin.position.x + index_2d.column * meta_data.resolution + meta_data.resolution / 2.0;
    double y = meta_data.origin.position.y + index_2d.row * meta_data.resolution + meta_data.resolution / 2.0;

    geometry_msgs::Point point;
    point.x = std::move(x);
    point.y = std::move(y);
    point.z = 0.0;

    return point;
}

// ######################################################################### //

geometry_msgs::Point index_1d_to_point(const Index1D &index_1d, const nav_msgs::MapMetaData &meta_data)
{
    return index_2d_to_point(index_1d_to_2d(index_1d, meta_data), meta_data);
}

// ######################################################################### //

Index2D point_to_index_2d(const geometry_msgs::Point &point, const nav_msgs::MapMetaData &meta_data)
{
    auto column = static_cast<Index1D>((point.x - meta_data.origin.position.x) / meta_data.resolution);
    auto row = static_cast<Index1D>((point.y - meta_data.origin.position.y) / meta_data.resolution);

    return Index2D(std::move(column), std::move(row));
}

// ######################################################################### //

Index1D index_2d_to_1d(const Index2D &index_2d, const nav_msgs::MapMetaData &meta_data)
{
    return index_2d.column + index_2d.row * meta_data.width;
}

// ######################################################################### //

EightConnectedNeighborType get_eight_connected_neighbor_type(const Index1D lhs, const Index1D rhs,
                                                             const nav_msgs::MapMetaData &meta_data)
{
    const auto lhs_index_2d = index_1d_to_2d(lhs, meta_data);
    const auto rhs_index_2d = index_1d_to_2d(rhs, meta_data);

    const auto delta_column = lhs_index_2d.column - rhs_index_2d.column;
    const auto delta_row = lhs_index_2d.row - rhs_index_2d.row;

    if ((delta_column == 0) && (delta_row == 0))
    {
        // The two indices are identical.
        return EightConnectedNeighborType::NONE;
    }
    else if ((std::abs(delta_column) > 1) || (std::abs(delta_row) > 1))
    {
        // The two indices are not neighbors.
        return EightConnectedNeighborType::NONE;
    }
    else if ((delta_column == 0) || (delta_row == 0))
    {
        // The two indices are adjacent to one another.
        return EightConnectedNeighborType::ADJACENT;
    }
    else
    {
        // The two indices are diagonal to one another.
        return EightConnectedNeighborType::DIAGONAL;
    }
}

// ######################################################################### //

std::vector<Index1D> get_eight_connected_neighbors(const Index1D index_1d, const nav_msgs::MapMetaData &meta_data)
{
    const auto index_2d = index_1d_to_2d(index_1d, meta_data);

    std::vector<Index2D> neighbors_2d;
    if (index_2d.column > 0)
    {
        // Left.
        neighbors_2d.emplace_back(index_2d.column - 1, index_2d.row);
    }
    if (index_2d.row > 0)
    {
        // Bottom.
        neighbors_2d.emplace_back(index_2d.column, index_2d.row - 1);
    }
    if (index_2d.column < (meta_data.width - 1))
    {
        // Right.
        neighbors_2d.emplace_back(index_2d.column + 1, index_2d.row);
    }
    if (index_2d.row < (meta_data.height - 1))
    {
        // Top.
        neighbors_2d.emplace_back(index_2d.column, index_2d.row + 1);
    }
    if ((index_2d.column > 0) && (index_2d.row > 0))
    {
        // Bottom-left.
        neighbors_2d.emplace_back(index_2d.column - 1, index_2d.row - 1);
    }
    if ((index_2d.column > 0) && (index_2d.row < (meta_data.height - 1)))
    {
        // Top-left.
        neighbors_2d.emplace_back(index_2d.column - 1, index_2d.row + 1);
    }
    if ((index_2d.column < (meta_data.width - 1)) && (index_2d.row > 0))
    {
        // Bottom-right.
        neighbors_2d.emplace_back(index_2d.column + 1, index_2d.row - 1);
    }
    if ((index_2d.column < (meta_data.width - 1)) && (index_2d.row < (meta_data.height - 1)))
    {
        // Top-right.
        neighbors_2d.emplace_back(index_2d.column + 1, index_2d.row + 1);
    }

    std::vector<Index1D> neighbors_1d(neighbors_2d.size());
    std::transform(std::begin(neighbors_2d), std::end(neighbors_2d), std::begin(neighbors_1d),
                   [&meta_data](const Index2D &index_2d) { return index_2d_to_1d(index_2d, meta_data); });

    return neighbors_1d;
}

// ######################################################################### //

Index1D point_to_index_1d(const geometry_msgs::Point &point, const nav_msgs::MapMetaData &meta_data)
{
    return index_2d_to_1d(point_to_index_2d(point, meta_data), meta_data);
}

// ######################################################################### //

boost::optional<std::vector<geometry_msgs::Point>> a_star(const nav_msgs::OccupancyGrid &occupancy_grid,
                                                          const geometry_msgs::Point &start,
                                                          const geometry_msgs::Point &goal)
{
    const auto start_index = point_to_index_1d(start, occupancy_grid.info);
    const auto goal_index = point_to_index_1d(goal, occupancy_grid.info);

    // Use the eight-connected neighborhood.
    const auto neighbor_functor = [&occupancy_grid](const Index1D index_1d) {
        return get_eight_connected_neighbors(index_1d, occupancy_grid.info);
    };

    // Cost to travel between two neighbors is the average of their individual occupancy costs.
    const auto travel_functor = [&occupancy_grid](const Index1D from, const Index1D to) {
        const auto neighbor_type = get_eight_connected_neighbor_type(from, to, occupancy_grid.info);
        if (neighbor_type == EightConnectedNeighborType::NONE)
        {
            throw std::runtime_error("attempted to compute neighbor type between two nodes that aren't in one anothers "
                                     "eight-connected neighborhood");
        }

        const double multiplier = (neighbor_type == EightConnectedNeighborType::DIAGONAL) ? sqrt(2.0) : 1.0;

        return multiplier *
               (static_cast<double>(occupancy_grid.data[from]) + static_cast<double>(occupancy_grid.data[to])) / 2.0;
    };

    // Heuristic cost to the goal is the euclidean distance to the goal.
    const auto heuristic_functor = [&occupancy_grid, &goal](const Index1D index_1d) {
        const auto &point = index_1d_to_point(index_1d, occupancy_grid.info);

        const double dx = goal.x - point.x;
        const double dy = goal.y - point.y;

        return sqrt((dx * dx) + (dy * dy));
    };

    // Run A*.
    auto index_path = a_star(start_index, goal_index, neighbor_functor, travel_functor, heuristic_functor);

    if (!index_path)
    {
        // We couldn't plan a path.
        return boost::none;
    }

    // Convert back to points.
    std::vector<geometry_msgs::Point> points_path(index_path->size());
    std::transform(
        std::begin(index_path.value()), std::end(index_path.value()), std::begin(points_path),
        [&occupancy_grid](const Index1D index_1d) { return index_1d_to_point(index_1d, occupancy_grid.info); });

    return boost::make_optional(std::move(points_path));
}

// ######################################################################### //

} // namespace path_planning
