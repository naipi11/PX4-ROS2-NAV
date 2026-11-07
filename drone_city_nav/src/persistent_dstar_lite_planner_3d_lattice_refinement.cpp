#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_lattice_level_zero.hpp"

// Resumable waypoint refinement of blocked level-zero edges: the fan one
// update may probe, the queue of fans an update cut short, and the edges a
// resumed fan promoted. See PlannerLattice3D::retryDeferredRefinements.

namespace drone_city_nav::detail {

std::optional<Point3> PlannerLattice3D::probeRefinementFan(
    const PersistentPlannerNode3D first, const PersistentPlannerNode3D second,
    std::size_t& next_probe, std::size_t& probes_spent, const std::size_t probe_limit,
    const std::chrono::steady_clock::time_point deadline, bool& fan_exhausted) {
  fan_exhausted = false;
  const auto offsets = static_cast<std::size_t>(config_->edge_refinement_offsets);
  if (offsets == 0U) {
    fan_exhausted = true;
    return std::nullopt;
  }
  const std::size_t total_probes = 4U * offsets;
  if (next_probe >= total_probes) {
    fan_exhausted = true;
    return std::nullopt;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
  const Vec3 along{second_point.x - first_point.x, second_point.y - first_point.y,
                   second_point.z - first_point.z};
  const double length =
      std::sqrt(along.x * along.x + along.y * along.y + along.z * along.z);
  if (!(length > 1.0e-6)) {
    fan_exhausted = true;
    return std::nullopt;
  }
  const Vec3 unit{along.x / length, along.y / length, along.z / length};
  // Two directions across the edge: one horizontal, one carrying whatever
  // vertical freedom the edge leaves. A doorway is missed across the edge, not
  // along it.
  const double horizontal_length = std::hypot(unit.x, unit.y);
  const Vec3 lateral =
      horizontal_length > 1.0e-6
          ? Vec3{-unit.y / horizontal_length, unit.x / horizontal_length, 0.0}
          : Vec3{1.0, 0.0, 0.0};
  const Vec3 vertical{lateral.y * unit.z - lateral.z * unit.y,
                      lateral.z * unit.x - lateral.x * unit.z,
                      lateral.x * unit.y - lateral.y * unit.x};
  const Point3 middle{0.5 * (first_point.x + second_point.x),
                      0.5 * (first_point.y + second_point.y),
                      0.5 * (first_point.z + second_point.z)};
  const double step_m =
      config_->minimum_horizontal_step_m / static_cast<double>(offsets + 1U);
  // Nearest first: the shortest detour that clears the body wins. The three
  // nested orders are flattened into one index so an attempt the update's
  // budget cut short resumes at the same candidate rather than starting the
  // fan again.
  while (next_probe < total_probes) {
    // The probe budget bounds the sweeps, not their cost: two raw swept-body
    // validations near evidence are milliseconds each, and a budget of
    // hundreds spent inside one fan is the whole update. The fan stops when
    // the update's deadline passes, exactly as the search stages around it
    // do, and keeps its cursor.
    if (probes_spent >= probe_limit || std::chrono::steady_clock::now() >= deadline) {
      return std::nullopt;
    }
    const std::size_t magnitude = next_probe / 4U + 1U;
    const bool lateral_axis = (next_probe / 2U) % 2U == 0U;
    const bool positive = next_probe % 2U == 0U;
    ++next_probe;
    const Vec3& axis = lateral_axis ? lateral : vertical;
    const double sign = positive ? 1.0 : -1.0;
    const double distance_m = static_cast<double>(magnitude) * step_m;
    const Point3 waypoint{middle.x + sign * distance_m * axis.x,
                          middle.y + sign * distance_m * axis.y,
                          middle.z + sign * distance_m * axis.z};
    if (!pointInsideFlightEnvelope(waypoint)) {
      continue;
    }
    ++probes_spent;
    if (routePiecesValid(first_point, waypoint) &&
        routePiecesValid(waypoint, second_point)) {
      return waypoint;
    }
  }
  fan_exhausted = true;
  return std::nullopt;
}

void PlannerLattice3D::deferRefinement(const PersistentPlannerEdge3D& edge,
                                       const std::size_t next_probe) {
  const auto offsets = static_cast<std::size_t>(config_->edge_refinement_offsets);
  if (offsets == 0U || config_->maximum_edge_refinement_probes == 0U ||
      next_probe >= 4U * offsets || deferred_refinement_members_.contains(edge)) {
    return;
  }
  const LevelZeroSlot slot = levelZeroSlot(edge);
  const auto bit = static_cast<std::uint16_t>(1U << slot.direction);
  if (!deferred_overflow_.empty() && (deferred_overflow_[slot.node] & bit) != 0U) {
    return;
  }
  if (deferred_refinements_.size() >= kMaximumDeferredRefinements) {
    if (deferred_overflow_.empty()) {
      deferred_overflow_.assign(nodeCount(), 0U);
    }
    deferred_overflow_[slot.node] |= bit;
    ++deferred_overflow_count_;
    return;
  }
  deferred_refinement_members_.insert(edge);
  deferred_refinements_.push_back(DeferredRefinement3D{
      .edge = edge, .next_probe = next_probe, .change_epoch = change_epoch_});
}

void PlannerLattice3D::forgetDeferredRefinement(const PersistentPlannerEdge3D& edge) {
  if (!deferred_overflow_.empty()) {
    const LevelZeroSlot slot = levelZeroSlot(edge);
    const auto bit = static_cast<std::uint16_t>(1U << slot.direction);
    if ((deferred_overflow_[slot.node] & bit) != 0U) {
      deferred_overflow_[slot.node] &= static_cast<std::uint16_t>(~bit);
      --deferred_overflow_count_;
    }
  }
  if (deferred_refinement_members_.erase(edge) != 0U) {
    std::erase_if(deferred_refinements_,
                  [&](const DeferredRefinement3D& item) { return item.edge == edge; });
  }
}

void PlannerLattice3D::refillDeferredRefinements(
    const std::chrono::steady_clock::time_point deadline) {
  while (deferred_overflow_count_ != 0U &&
         deferred_refinements_.size() < kMaximumDeferredRefinements &&
         std::chrono::steady_clock::now() < deadline) {
    const std::size_t index = deferred_overflow_cursor_;
    const std::uint16_t mask = deferred_overflow_[index];
    if (mask == 0U) {
      deferred_overflow_cursor_ = (index + 1U) % deferred_overflow_.size();
      continue;
    }
    const auto direction = static_cast<std::size_t>(std::countr_zero(mask));
    const auto bit = static_cast<std::uint16_t>(1U << direction);
    deferred_overflow_[index] &= static_cast<std::uint16_t>(~bit);
    --deferred_overflow_count_;
    const LevelZeroSlot slot{.node = index, .direction = direction};
    if (levelZeroState(slot) != kLevelZeroEdgeBlocked) {
      continue;
    }
    const PersistentPlannerNode3D first = nodeAt(index);
    const LevelZeroOffset3D offset = kLevelZeroOffsets[direction];
    const PersistentPlannerNode3D second{first.x + offset.x, first.y + offset.y,
                                         first.z + offset.z};
    if (!nodeInside(second)) {
      continue;
    }
    const PersistentPlannerEdge3D edge = canonicalEdge(first, second);
    if (!deferred_refinement_members_.insert(edge).second) {
      continue;
    }
    // Overflow does not keep a cursor or epoch. Rechecking the entire fan on
    // current placements is conservative after any intervening world change.
    placeNode(first);
    placeNode(second);
    deferred_refinements_.push_back(DeferredRefinement3D{
        .edge = edge, .next_probe = 0U, .change_epoch = change_epoch_});
  }
}

void PlannerLattice3D::retryDeferredRefinements(
    const std::size_t maximum_probes,
    const std::chrono::steady_clock::time_point deadline) {
  if (maximum_probes == 0U) {
    return;
  }
  refillDeferredRefinements(deadline);
  if (deferred_refinements_.empty()) {
    return;
  }
  std::size_t spent{0U};
  std::size_t examined{0U};
  const std::size_t pending = deferred_refinements_.size();
  // One edge must not take the whole pass: an edge queued behind a fan that
  // always stops on the budget would never be tried at all, which is the
  // starvation this queue exists to end. Each edge gets an even share of the
  // pass, and at least one probe, so every pass advances every fan it holds.
  const std::size_t per_edge_limit =
      std::max<std::size_t>(1U, maximum_probes / std::max<std::size_t>(1U, pending));
  while (!deferred_refinements_.empty() && spent < maximum_probes &&
         examined < pending && std::chrono::steady_clock::now() < deadline) {
    DeferredRefinement3D item = deferred_refinements_.front();
    deferred_refinements_.pop_front();
    deferred_refinement_members_.erase(item.edge);
    ++examined;
    if (!nodeInside(item.edge.first) || !nodeInside(item.edge.second) ||
        item.edge.first == item.edge.second ||
        level(item.edge.first, item.edge.second) != 0U) {
      continue;
    }
    const LevelZeroSlot slot = levelZeroSlot(item.edge);
    // Evidence that touched this edge re-derived it: a state that is no
    // longer blocked belongs to that derivation, and the fan cursor with the
    // world it was walking is spent.
    if (levelZeroState(slot) != kLevelZeroEdgeBlocked) {
      continue;
    }
    if (item.change_epoch != change_epoch_) {
      // A previous failed waypoint can become clear after an occupied change
      // beside either of its off-edge legs. Bound every possible waypoint and
      // body by a sphere about the canonical edge midpoint; an unrelated
      // changed chunk leaves the fan's progress intact.
      const Point3 first = canonicalPointFor(item.edge.first);
      const Point3 second = canonicalPointFor(item.edge.second);
      const Point3 midpoint{0.5 * (first.x + second.x), 0.5 * (first.y + second.y),
                            0.5 * (first.z + second.z)};
      const auto offsets = static_cast<double>(config_->edge_refinement_offsets);
      const auto& body = config_->physical_footprint;
      const double reach =
          0.5 * distance3D(first, second) + maximumNodePlacementOffsetM() +
          config_->minimum_horizontal_step_m * offsets / (offsets + 1.0) +
          std::max({body.radius_m, body.lower_extent_m, body.upper_extent_m});
      if (clearanceStale(midpoint, reach, item.change_epoch)) {
        item.next_probe = 0U;
        placeNode(item.edge.first);
        placeNode(item.edge.second);
      }
      item.change_epoch = change_epoch_;
    }
    bool fan_exhausted{false};
    // The limit is per edge here: a pass that splits its allowance between
    // several fans has to let each one spend its own share, measured from
    // where that fan starts.
    std::size_t edge_spent{0U};
    const std::optional<Point3> waypoint =
        probeRefinementFan(item.edge.first, item.edge.second, item.next_probe,
                           edge_spent, per_edge_limit, deadline, fan_exhausted);
    spent += edge_spent;
    if (waypoint.has_value()) {
      rememberRefinedWaypoint(item.edge, *waypoint);
      setLevelZeroState(slot, kLevelZeroEdgeRefined);
      promoted_edges_.push_back(item.edge);
      continue;
    }
    if (fan_exhausted) {
      // Every candidate was tried and none cleared the body: blocked stands.
      continue;
    }
    // The budget went again. The fan keeps its cursor at the back of the
    // queue, so the edges behind it get their turn.
    deferred_refinements_.push_back(item);
    deferred_refinement_members_.insert(item.edge);
  }
  refillDeferredRefinements(deadline);
}

std::vector<PersistentPlannerEdge3D> PlannerLattice3D::takePromotedEdges() {
  std::vector<PersistentPlannerEdge3D> taken;
  taken.swap(promoted_edges_);
  return taken;
}

} // namespace drone_city_nav::detail
