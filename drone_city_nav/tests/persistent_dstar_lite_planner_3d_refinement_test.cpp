#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_refinement_fixture.hpp"

// Edge refinement, departure waypoints, the start anchor and the escape
// search: how the planner reaches a route where no straight lattice segment
// admits the body.

namespace drone_city_nav {
namespace {

TEST(PersistentDStarLitePlanner3DTest, ARouteThroughARefinedEdgeCarriesItsWaypoint) {
  RefinedDoorwayFixture fixture;

  PersistentPlannerConfig3D unrefined_config = fixture.config;
  unrefined_config.edge_refinement_offsets = 0U;
  PersistentDStarLitePlanner3D unrefined{unrefined_config};
  const PlannerUpdate3D stuck = fixture.planUntilPublishable(
      unrefined, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_FALSE(stuck.publishable()) << "the fixture no longer needs a refined edge";

  PersistentDStarLitePlanner3D planner{fixture.config};
  const PlannerUpdate3D update = fixture.planUntilPublishable(
      planner, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_TRUE(update.publishable())
      << "edge refinement did not open the doorway: input="
      << static_cast<int>(update.input_status)
      << " expansions=" << update.telemetry.expansions
      << " time_expansions=" << update.telemetry.execution_time_search_expansions;
  const std::vector<Point3>& points = candidate(update).points;
  // The waypoint is off the lattice row and inside the door.
  const bool carries_waypoint = std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 7.5 && point.x < 8.5 && point.y > 3.1 && point.y < 4.0;
  });
  EXPECT_TRUE(carries_waypoint);
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest, ARejectedSegmentNamesTheEdgeThatAdmittedIt) {
  // The raw sweep is the authority on a candidate, and a leg it rejects has
  // to reach the cache entry that admitted it: a straight edge, or a refined
  // edge through its waypoint. Rejecting the edge forgets it and records it
  // for the persistent session's repair.
  RefinedDoorwayFixture fixture;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  ASSERT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value()) << "the doorway edge was not refined";
  EXPECT_NEAR(waypoint->x, 8.0, 1.0e-9);
  EXPECT_NEAR(waypoint->y, 3.5, 1.0e-9);
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);

  const std::vector<Point3> through_waypoint{fixture.start, west, *waypoint, east};
  for (const std::size_t leg : {2U, 3U}) {
    const std::optional<detail::PlannerLattice3D::PathSegmentEdge3D> priced =
        lattice.pricedEdgeForSegment(through_waypoint, leg);
    ASSERT_TRUE(priced.has_value()) << "leg " << leg;
    EXPECT_EQ(priced->from, fixture.door_west);
    EXPECT_EQ(priced->to, fixture.door_east);
  }
  // The departure from the exact start is not a lattice edge.
  EXPECT_FALSE(
      lattice.pricedEdgeForSegment({Point3{6.4, 3.2, 3.0}, west}, 1U).has_value());
  // A straight edge names itself.
  const std::optional<detail::PlannerLattice3D::PathSegmentEdge3D> straight =
      lattice.pricedEdgeForSegment({west, lattice.pointFor({2, 1, 1})}, 1U);
  ASSERT_TRUE(straight.has_value());
  EXPECT_EQ(straight->from, fixture.door_west);
  EXPECT_EQ(straight->to, (detail::PersistentPlannerNode3D{2, 1, 1}));

  const detail::PersistentPlannerEdge3D edge =
      detail::canonicalEdge(fixture.door_west, fixture.door_east);
  EXPECT_TRUE(lattice.rejectEdgeBySweep(edge));
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());
  EXPECT_FALSE(lattice.hasEdgeCost(edge));
  EXPECT_FALSE(lattice.rejectEdgeBySweep(edge)) << "nothing was cached to reject";
  const std::vector<detail::PersistentPlannerEdge3D> recorded =
      lattice.takeSweepRejectedEdges();
  ASSERT_EQ(recorded.size(), 1U);
  EXPECT_EQ(recorded.front(), edge);
  EXPECT_TRUE(lattice.takeSweepRejectedEdges().empty());
}

TEST(PersistentDStarLitePlanner3DTest,
     ARefinementTheProbeBudgetCutShortResumesOnALaterUpdate) {
  // The probe budget is per update, and near occupied evidence a search meets
  // more blocked edges than the budget can fan out. An edge whose fan stopped
  // on the budget was cached blocked for the life of the world, so a doorway
  // the budget reached too late stayed shut however long the vehicle waited:
  // the frozen r3510 revision 3520 probe found no route in 300 updates at the
  // 256-probe budget while the same map yielded a raw-valid route at a budget
  // large enough to finish the fans. The cut-short fan has to resume.
  RefinedDoorwayFixture fixture;
  fixture.config.maximum_edge_refinement_probes = 1U;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);

  // A row the wall blocks completely: its whole fan is tried and clears
  // nothing, and with one probe per update it is cut short on the first pass.
  const detail::PersistentPlannerNode3D blocked_west{3, 0, 1};
  const detail::PersistentPlannerNode3D blocked_east{4, 0, 1};
  lattice.beginEdgeRefinementBudget();
  EXPECT_FALSE(std::isfinite(lattice.rankedEdgeCost(blocked_west, blocked_east)))
      << "the fixture row no longer passes through the wall";
  // The budget is spent, so the doorway's own fan cannot start: it is cached
  // blocked with a cursor, not blocked forever.
  EXPECT_FALSE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());

  bool promoted = false;
  for (int update = 0; update < 8 && !promoted; ++update) {
    lattice.beginEdgeRefinementBudget();
    lattice.retryDeferredRefinements(fixture.config.maximum_edge_refinement_probes,
                                     std::chrono::steady_clock::time_point::max());
    for (const detail::PersistentPlannerEdge3D& edge : lattice.takePromotedEdges()) {
      promoted = promoted ||
                 edge == detail::canonicalEdge(fixture.door_west, fixture.door_east);
    }
  }
  ASSERT_TRUE(promoted) << "a fan the budget cut short was never resumed";
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value());
  EXPECT_NEAR(waypoint->x, 8.0, 1.0e-9);
  EXPECT_NEAR(waypoint->y, 3.5, 1.0e-9);
  EXPECT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  // Every candidate the fan tried cleared the body on both legs, so the
  // promoted edge is one the sweep admits.
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);
  EXPECT_TRUE(lattice.pathTraversable({west, *waypoint, east}));
}

TEST(PersistentDStarLitePlanner3DTest,
     AStalledFanBlockedEdgeStaysBlockedWhenEveryCandidateWasTried) {
  // The resume must not turn a fan that was tried whole into a search that
  // keeps re-probing the same candidates: an edge whose every waypoint the
  // sweep refuses is exhausted once and then left alone.
  RefinedDoorwayFixture fixture;
  fixture.config.maximum_edge_refinement_probes = 1U;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  const detail::PersistentPlannerNode3D blocked_west{3, 0, 1};
  const detail::PersistentPlannerNode3D blocked_east{4, 0, 1};
  EXPECT_FALSE(std::isfinite(lattice.rankedEdgeCost(blocked_west, blocked_east)));
  for (int update = 0; update < 12; ++update) {
    lattice.beginEdgeRefinementBudget();
    lattice.retryDeferredRefinements(fixture.config.maximum_edge_refinement_probes,
                                     std::chrono::steady_clock::time_point::max());
    EXPECT_TRUE(lattice.takePromotedEdges().empty())
        << "a fully refused fan was promoted on update " << update;
  }
  EXPECT_FALSE(std::isfinite(lattice.rankedEdgeCost(blocked_west, blocked_east)));
}

TEST(PersistentDStarLitePlanner3DTest,
     AResumedFanBehindAnExpiredDeadlineSpendsNothingAndKeepsItsPlace) {
  // The resumed fan runs outside any search, so the probe budget would be its
  // only bound: two raw swept-body validations near evidence are milliseconds
  // each and a budget of hundreds inside one fan is the whole update. A fan
  // whose deadline has passed must spend nothing, promote nothing, and keep
  // its place for the next update.
  RefinedDoorwayFixture fixture;
  fixture.config.maximum_edge_refinement_probes = 1U;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  const detail::PersistentPlannerNode3D blocked_west{3, 0, 1};
  const detail::PersistentPlannerNode3D blocked_east{4, 0, 1};
  lattice.beginEdgeRefinementBudget();
  ASSERT_FALSE(std::isfinite(lattice.rankedEdgeCost(blocked_west, blocked_east)));
  ASSERT_FALSE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));

  lattice.retryDeferredRefinements(fixture.config.maximum_edge_refinement_probes,
                                   std::chrono::steady_clock::now() -
                                       std::chrono::milliseconds{1});
  EXPECT_TRUE(lattice.takePromotedEdges().empty())
      << "a fan behind an expired deadline promoted an edge";
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());

  // The queue survived, so a later update with a real deadline still finds
  // the doorway.
  bool promoted = false;
  for (int update = 0; update < 8 && !promoted; ++update) {
    lattice.beginEdgeRefinementBudget();
    lattice.retryDeferredRefinements(fixture.config.maximum_edge_refinement_probes,
                                     std::chrono::steady_clock::time_point::max());
    for (const detail::PersistentPlannerEdge3D& edge : lattice.takePromotedEdges()) {
      promoted = promoted ||
                 edge == detail::canonicalEdge(fixture.door_west, fixture.door_east);
    }
  }
  EXPECT_TRUE(promoted) << "the expired pass discarded the fan instead of keeping it";
}

TEST(PersistentDStarLitePlanner3DTest,
     DeferredDoorwayRechecksAnEarlierWaypointAfterEvidenceRemoval) {
  RefinedDoorwayFixture fixture;
  fixture.config.maximum_edge_refinement_probes = 1U;
  const auto cells = RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const auto blocked = RefinedDoorwayFixture::withCells(*fixture.occupancy, cells);
  const PersistentPlannerWorld3D before = world(blocked, 1U);
  const PersistentPlannerWorld3D after =
      world(fixture.occupancy, 2U, RefinedDoorwayFixture::chunksOf(cells));
  detail::PlannerLattice3D lattice{fixture.config};
  lattice.configureGridGeometry(*before.bounds());
  lattice.installWorld(before);
  lattice.beginEdgeRefinementBudget();
  ASSERT_FALSE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));

  // The removed evidence touches the first, off-edge waypoint rather than
  // the straight segment. A cursor from the old world must not prevent that
  // newly available waypoint from being tested on the new world.
  lattice.installWorld(after);
  detail::DStarLiteSession3D session{fixture.config, lattice};
  std::size_t affected_states = 0U;
  session.scheduleAffectedVertices(after, cells, affected_states);
  for (int update = 0; update < 20; ++update) {
    lattice.retryDeferredRefinements(1U, std::chrono::steady_clock::time_point::max());
  }
  const auto waypoint = lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value());
  EXPECT_TRUE(lattice.pathTraversable({lattice.pointFor(fixture.door_west), *waypoint,
                                       lattice.pointFor(fixture.door_east)}));
}

TEST(PersistentDStarLitePlanner3DTest, APlannerWhoseBudgetStopsAFanStillOpensADoorway) {
  // The same doorway, asked of the planner with a budget smaller than one
  // fan: the 3-offset fan is twelve candidates, so a search that meets more
  // blocked edges than the budget can fan out leaves most of them cut short.
  // Before the resume every such edge was cached blocked for the life of the
  // world. The route has to exist anyway, however many updates it takes: the
  // fan resumes where it stopped.
  RefinedDoorwayFixture fixture;
  fixture.config.maximum_edge_refinement_probes = 8U;
  PersistentDStarLitePlanner3D planner{fixture.config};
  const PersistentPlannerRequest3D req =
      request(fixture.start, fixture.goal, world(fixture.occupancy, 1U));
  PlannerUpdate3D update;
  int updates = 0;
  for (; updates < 400; ++updates) {
    update = planner.plan(req);
    if (update.publishable()) {
      break;
    }
    ASSERT_NE(update.progress, SearchProgress3D::kNoRoute)
        << "the dispatcher must not terminate before a deferred doorway is tried";
  }
  ASSERT_TRUE(update.publishable())
      << "a budget below one fan never opened the doorway in " << updates
      << " updates: input=" << static_cast<int>(update.input_status);
  expectRawValid(candidate(update).points, *fixture.occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     DisabledRefinementBudgetCanTerminateWithoutRoute) {
  RefinedDoorwayFixture fixture;
  fixture.config.maximum_edge_refinement_probes = 0U;
  PersistentDStarLitePlanner3D planner{fixture.config};
  const PersistentPlannerRequest3D req =
      request(fixture.start, fixture.goal, world(fixture.occupancy, 1U));
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 400; ++attempt) {
    update = planner.plan(req);
    ASSERT_FALSE(update.publishable());
    if (update.progress == SearchProgress3D::kNoRoute) {
      return;
    }
  }
  FAIL() << "a disabled refinement budget cannot make deferred fans progress";
}

TEST(PersistentDStarLitePlanner3DTest, APathIsAcceptedOnlyAsItsRoutePiecesClear) {
  // A plate over a street, z in [7.75, 8.0), and a segment descending past
  // its edge that the body barely clears. Swept whole it passes; cut into the pieces
  // the route sampler makes of it, one piece is refused, as the activation
  // and the tracking tube sweep it. Accepted whole, r681 and r685 stood for
  // minutes while the activation refused the same path on every update.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{-30.0, -30.0, 0.0, 0.25, 160, 160, 80});
  for (double x = -18.375; x < -17.0; x += 0.25) {
    for (double y = 5.125; y < 9.0; y += 0.25) {
      const std::optional<GridIndex3D> cell =
          occupancy->worldToCell(Point3{x, y, 7.875});
      ASSERT_TRUE(cell.has_value());
      ASSERT_TRUE(occupancy->setState(*cell, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.physical_footprint = SweptFootprintConfig{.radius_m = 0.82,
                                                   .lower_extent_m = 0.23,
                                                   .upper_extent_m = 0.35,
                                                   .body_radius_m = 0.55,
                                                   .body_lower_extent_m = 0.23,
                                                   .body_upper_extent_m = 0.35,
                                                   .perimeter_samples = 12U,
                                                   .radial_rings = 2U,
                                                   .axial_samples = 3U,
                                                   .sweep_step_m = 0.25};
  config.route_sampling_step_m = 0.5;
  detail::PlannerLattice3D lattice{config};
  const PersistentPlannerWorld3D plate = world(occupancy, 1U);
  lattice.configureGridGeometry(*plate.bounds());
  lattice.installWorld(plate);
  const Point3 departure{-14.5, 8.623, 7.478};
  const Point3 first{-15.663, 8.623, 7.478};
  const Point3 second{-16.489, 9.213, 7.325};
  ASSERT_TRUE(lattice.rawSegmentValid(first, second)) << "the whole segment clears";
  EXPECT_FALSE(lattice.routePiecesValid(first, second));
  EXPECT_EQ(std::optional<std::size_t>{2U},
            lattice.firstInvalidSegment({departure, first, second}));
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeBesideARefinedEdgesWaypointForgetsTheEdge) {
  // The change scheduling tests the geometry the vehicle flies. A cell that
  // touches a refined edge's leg through its waypoint, but lies farther from
  // the straight node-to-node segment than the body margin, has to forget the
  // edge all the same; otherwise the cache keeps admitting a leg the sweep
  // rejects, and every search on the lattice rediscovers the same blocked
  // route for as long as the cell stays.
  RefinedDoorwayFixture fixture;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  ASSERT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value()) << "the doorway edge was not refined";

  const std::vector<GridIndex3D> cells =
      RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const PersistentPlannerWorld3D after =
      world(RefinedDoorwayFixture::withCells(*fixture.occupancy, cells), 2U,
            RefinedDoorwayFixture::chunksOf(cells));
  lattice.installWorld(after);
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);
  ASSERT_FALSE(lattice.pathTraversable({west, *waypoint, east}))
      << "the cells do not block the refined legs";
  ASSERT_TRUE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value())
      << "installing the world must not forget edges by itself";

  detail::DStarLiteSession3D session{fixture.config, lattice};
  std::size_t affected_states = 0U;
  session.scheduleAffectedVertices(after, cells, affected_states);
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());
  EXPECT_FALSE(
      lattice.hasEdgeCost(detail::canonicalEdge(fixture.door_west, fixture.door_east)));
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeBesideTheWaypointReroutesWithoutRestartingTheFeasibilitySearch) {
  // The doorway is refined on both lattice levels. Cells that block the upper
  // level's waypoint leave the lower one: the searches have to re-route
  // through it from their labels, not restart from nothing and rediscover
  // the blocked edge from the cache on every update.
  RefinedDoorwayFixture fixture;
  fixture.config.feasibility_first_enabled = true;
  PersistentDStarLitePlanner3D planner{fixture.config};
  PlannerUpdate3D update = fixture.planUntilPublishable(
      planner, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_TRUE(update.publishable());

  const std::vector<GridIndex3D> cells =
      RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const std::shared_ptr<ObservedOccupancyGrid3D> changed =
      RefinedDoorwayFixture::withCells(*fixture.occupancy, cells);
  PersistentPlannerRequest3D blocked =
      request(fixture.start, fixture.goal,
              world(changed, 2U, RefinedDoorwayFixture::chunksOf(cells)));
  // The consumer could not enter the route it held, so the feasibility
  // search runs.
  blocked.incumbent_rejection_sequence = 1U;
  std::size_t restarts = 0U;
  for (int attempt = 0; attempt < 12; ++attempt) {
    update = planner.plan(blocked);
    restarts = std::max(restarts, update.telemetry.feasibility_restarts);
    if (update.publishable()) {
      break;
    }
  }
  ASSERT_TRUE(update.publishable())
      << "no route below the blocked waypoint: restarts=" << restarts
      << " closest_goal_m=" << update.telemetry.feasibility_closest_goal_distance_m
      << " exhausted=" << update.telemetry.feasibility_frontier_exhausted;
  EXPECT_EQ(restarts, 0U) << "the search restarted instead of forgetting the edge";
  const std::vector<Point3>& points = candidate(update).points;
  expectRawValid(points, *changed, planner.config().physical_footprint);
  EXPECT_TRUE(std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 7.5 && point.x < 8.5 && point.z < 2.6;
  })) << "the route does not pass the door below the blocked cells";
}

TEST(PersistentDStarLitePlanner3DTest,
     RepairRunsWhileTheFeasibilitySearchLooksForAFirstRoute) {
  // No route is held and the world just changed: the feasibility search needs
  // most of the update, and the persistent session's repair queue is what
  // makes its labels true again. The repair has to get a share of the update
  // before the feasibility search takes the rest, or the session reports a
  // shortest path on labels the world has moved for as long as no route exists.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 40, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_compute_time_ms = 20.0;
  config.maximum_feasibility_compute_time_ms = 19.0;
  config.maximum_feasibility_expansions_per_update = 1U << 20U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 20.5, 1.5};
  const Point3 goal{38.5, 20.5, 1.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable())
      << "input=" << static_cast<int>(update.input_status)
      << " feasibility_expansions=" << update.telemetry.feasibility_expansions
      << " expansions=" << update.telemetry.expansions
      << " closest_goal_m=" << update.telemetry.feasibility_closest_goal_distance_m;

  // Enclose the goal: the labels around it move, and no route exists, so the
  // feasibility search runs to its deadline on every update.
  auto enclosed = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  std::vector<OccupancyChunkIndex3D> dirty;
  for (int x = 35; x < 40; ++x) {
    for (int y = 17; y < 24; ++y) {
      if (x == 35 || y == 17 || y == 23) {
        for (int z = 0; z < 3; ++z) {
          const GridIndex3D cell{x, y, z};
          ASSERT_TRUE(enclosed->setState(cell, ObservedVoxelState::kOccupied));
          dirty.push_back(ObservedOccupancyGrid3D::chunkIndex(cell));
        }
      }
    }
  }
  PersistentPlannerRequest3D blocked =
      request(start, goal, world(enclosed, 2U, std::move(dirty)));
  blocked.incumbent_rejection_sequence = 1U;
  update = planner.plan(blocked);
  ASSERT_TRUE(update.telemetry.feasibility_attempted);
  // The session's bookkeeping for the change runs before the search, and the
  // repair takes its share of the same update while the search carries on.
  EXPECT_GT(update.telemetry.repair_lattice_states_processed, 0U)
      << "pending=" << update.telemetry.repair_lattice_states_pending
      << " feasibility_ms=" << update.telemetry.feasibility_ms
      << " repair_ms=" << update.telemetry.repair_ms;
}

TEST(PersistentDStarLitePlanner3DTest, TheStartAnchorIsKeptWhileItStaysAdmissible) {
  // A vehicle drifting between two nodes keeps the anchor its searches were
  // seeded from; re-anchoring on every flip of the nearest node would restart
  // them for nothing. The walk over the anchors on exhaustion still wins.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 5, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  detail::PlannerLattice3D lattice{config};
  const PersistentPlannerWorld3D open = world(occupancy, 1U);
  lattice.configureGridGeometry(*open.bounds());
  lattice.installWorld(open);
  const detail::PersistentPlannerNode3D first{1, 1, 1};
  const detail::PersistentPlannerNode3D next{2, 1, 1};
  ASSERT_NEAR(lattice.pointFor(first).x, 3.0, 1.0e-9);
  ASSERT_NEAR(lattice.pointFor(next).x, 5.0, 1.0e-9);

  const Point3 nearer_the_next{4.2, 3.0, 1.5};
  const detail::PlannerLattice3D::DepartureConnection3D nearest =
      lattice.selectDepartureConnection(nearer_the_next);
  ASSERT_TRUE(nearest.available());
  EXPECT_EQ(*nearest.anchor, next);

  const detail::PlannerLattice3D::DepartureConnection3D kept =
      lattice.selectDepartureConnection(nearer_the_next, 0U, first);
  ASSERT_TRUE(kept.available());
  EXPECT_EQ(*kept.anchor, first) << "the anchor flipped to the nearest node";

  const detail::PlannerLattice3D::DepartureConnection3D walked =
      lattice.selectDepartureConnection(nearer_the_next, 1U, first);
  const detail::PlannerLattice3D::DepartureConnection3D walked_unpreferred =
      lattice.selectDepartureConnection(nearer_the_next, 1U);
  ASSERT_TRUE(walked.available());
  ASSERT_TRUE(walked_unpreferred.available());
  EXPECT_EQ(*walked.anchor, *walked_unpreferred.anchor)
      << "the preference must not alter the walk on exhaustion";

  // A preferred anchor the body no longer reaches is not kept.
  auto walled = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  for (int y = 0; y < 5; ++y) {
    for (int z = 0; z < 3; ++z) {
      ASSERT_TRUE(walled->setState({3, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  lattice.installWorld(world(walled, 2U));
  const detail::PlannerLattice3D::DepartureConnection3D released =
      lattice.selectDepartureConnection(nearer_the_next, 0U, first);
  ASSERT_TRUE(released.available());
  EXPECT_EQ(*released.anchor, next);
}

// Two rooms joined by a corridor 1.5 m wide whose centreline lies half a
// lattice step off the lattice rows: on a 2 m lattice over a 0.25 m map with a
// 0.5 m body, no node inside the corridor is valid and no straight lattice
// edge crosses it, so the graph has no path from one room to the other. The
// corridor is wide enough for the body, and a vehicle in the first room has a
// way out at the body's own scale.
// A corridor no lattice node stands in, placed or not. The body of radius
// 0.5 m fits the corridor y in [4.2, 5.4) only with its centre in
// [4.7, 4.9): the row of nodes on y = 5 fails by the wall above, and node
// placement, which probes a quarter and 0.45 of the 2 m step straight and
// diagonally, lands nearest at y = 4.646 -- still in the wall below. The
// escape fill, anchored on the vehicle at half-step cells, crosses the lane
// at y = 4.8.
struct NodelessCorridorFixture {
  static constexpr double kResolution = 0.2;
  static constexpr int kWidth = 80;  // 16 m
  static constexpr int kHeight = 30; // 6 m
  static constexpr int kDepth = 10;  // 2 m
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{3.0, 3.8, 1.0};
  Point3 goal{13.0, 3.0, 1.0};

  NodelessCorridorFixture() {
    // Walls between the rooms, x in [6, 10), except the corridor y in
    // [4.2, 5.4).
    for (int x = 30; x < 50; ++x) {
      for (int y = 0; y < kHeight; ++y) {
        if (y >= 21 && y < 27) {
          continue;
        }
        for (int z = 0; z < kDepth; ++z) {
          if (!occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied)) {
            throw std::logic_error{"fixture cell outside the grid"};
          }
        }
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.flight_envelope.maximum_target_z_m = 2.0;
    config.feasibility_first_enabled = true;
    config.maximum_compute_time_ms = 200.0;
    config.maximum_no_route_compute_time_ms = 200.0;
    config.maximum_feasibility_compute_time_ms = 100.0;
    config.departure_refinement_subdivisions = 4U;
    config.escape_search_radius_cells = 4U;
    config.escape_search_maximum_probes_per_update = 1U << 16U;
  }
};

TEST(PersistentDStarLitePlanner3DTest,
     AnExhaustedFrontierIsReportedAndWalksTheAnchors) {
  // Without an escape search a closed component is terminal, and the planner
  // has to say so: the exhaustion survives the search's own restart, and the
  // walk over the start's anchors advances on it.
  NodelessCorridorFixture fixture;
  fixture.config.escape_search_radius_cells = 0U;
  PersistentDStarLitePlanner3D planner{fixture.config};
  bool exhausted = false;
  std::size_t skip = 0U;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 12; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    exhausted = exhausted || update.telemetry.feasibility_frontier_exhausted;
    skip = std::max(skip, update.telemetry.departure_anchor_skip);
  }
  EXPECT_FALSE(update.publishable());
  EXPECT_TRUE(exhausted) << "the exhaustion was hidden by the restart";
  EXPECT_GT(skip, 0U) << "the anchor walk never advanced";
  EXPECT_FALSE(update.telemetry.escape_search_attempted);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheEscapeSearchLeavesAClosedComponentThroughTheCorridor) {
  NodelessCorridorFixture fixture;
  PersistentDStarLitePlanner3D planner{fixture.config};
  bool exhausted = false;
  bool escape_found = false;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 40; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    exhausted = exhausted || update.telemetry.feasibility_frontier_exhausted;
    escape_found = escape_found || update.telemetry.escape_search_found;
    if (update.publishable()) {
      break;
    }
  }
  EXPECT_TRUE(exhausted);
  EXPECT_TRUE(escape_found) << "explored="
                            << update.telemetry.escape_search_explored_cells
                            << " exhausted="
                            << update.telemetry.escape_search_exhausted;
  ASSERT_TRUE(update.publishable()) << "no route through the corridor";
  EXPECT_TRUE(update.telemetry.escape_connection_active);
  EXPECT_TRUE(update.telemetry.departure_waypoint_used);
  EXPECT_GT(update.telemetry.departure_waypoint_count, 1U);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 2U);
  EXPECT_NEAR(points.front().x, fixture.start.x, 1.0e-9);
  EXPECT_NEAR(points.back().x, fixture.goal.x, 1.0e-9);
  // The chain through the corridor is shortcut into one segment, so the
  // crossing is read off the segment that spans the wall.
  bool crosses_corridor = false;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const Point3& first = points[index - 1U];
    const Point3& second = points[index];
    if (std::min(first.x, second.x) > 6.0 || std::max(first.x, second.x) < 10.0 ||
        first.x == second.x) {
      continue;
    }
    const double ratio = (8.0 - first.x) / (second.x - first.x);
    const double y_at_wall = std::lerp(first.y, second.y, ratio);
    crosses_corridor = crosses_corridor || (y_at_wall > 4.6 && y_at_wall < 5.0);
  }
  EXPECT_TRUE(crosses_corridor) << "the route does not pass the corridor";
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

// The escape fill is anchored on the vehicle, but its cells are absolute
// points and the chain an exit yields is validated from wherever the vehicle
// stands when it is used: a vehicle hovering through a fraction of a lattice
// step of drift keeps the fill it has. Re-anchored at one fine step instead,
// the fill started over on every update and never reached its own radius.
TEST(PersistentDStarLitePlanner3DTest,
     AVehicleDriftingWithinALatticeStepKeepsTheEscapeFill) {
  NodelessCorridorFixture fixture;
  ASSERT_GE(fixture.config.departure_refinement_subdivisions, 2U);
  // A few probes an update: the fill takes many updates, so the drift lands
  // in the middle of it.
  fixture.config.escape_search_maximum_probes_per_update = 8U;
  PersistentDStarLitePlanner3D planner{fixture.config};
  PlannerUpdate3D update;
  std::size_t explored_before{0U};
  for (int attempt = 0; attempt < 60 && explored_before == 0U; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    if (update.telemetry.escape_search_attempted &&
        !update.telemetry.escape_search_found) {
      explored_before = update.telemetry.escape_search_explored_cells;
    }
  }
  ASSERT_GT(explored_before, 0U) << "the escape fill never started";

  const Point3 drifted{fixture.start.x + 0.4 * fixture.config.minimum_horizontal_step_m,
                       fixture.start.y, fixture.start.z};
  update = planner.plan(request(drifted, fixture.goal, world(fixture.occupancy, 1U)));

  EXPECT_TRUE(update.telemetry.escape_search_attempted ||
              update.telemetry.escape_connection_active);
  EXPECT_GE(update.telemetry.escape_search_explored_cells, explored_before)
      << "the fill started over on a drift within a lattice step";

  bool escape_found = update.telemetry.escape_search_found;
  for (int attempt = 0; attempt < 200 && !escape_found; ++attempt) {
    update = planner.plan(request(attempt % 2 == 0 ? drifted : fixture.start,
                                  fixture.goal, world(fixture.occupancy, 1U)));
    escape_found = update.telemetry.escape_search_found;
  }
  EXPECT_TRUE(escape_found) << "explored="
                            << update.telemetry.escape_search_explored_cells
                            << " exhausted="
                            << update.telemetry.escape_search_exhausted;
}

// Two nodeless corridors leave the first room: the nearer one ends in a
// walled pocket with a valid node of its own, the farther one leads to the
// goal's room. The fill reaches the pocket first; the search from that exit
// exhausts the pocket, which is no way out, so the exit is retired and the
// fill resumes until it finds the corridor that leads on.
struct DeadEndExitFixture {
  static constexpr double kResolution = 0.2;
  static constexpr int kWidth = 80;  // 16 m
  static constexpr int kHeight = 40; // 8 m
  static constexpr int kDepth = 10;  // 2 m
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{4.0, 2.3, 1.0};
  Point3 goal{13.0, 7.0, 1.0};

  DeadEndExitFixture() {
    const auto fill = [&](const int x, const int y) {
      for (int z = 0; z < kDepth; ++z) {
        if (!occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied)) {
          throw std::logic_error{"fixture cell outside the grid"};
        }
      }
    };
    // The wall between the rooms, x in [6, 10), pierced by the dead-end
    // corridor y in [2.2, 3.4) and the corridor on, y in [6.2, 7.4): the body
    // fits either only with its centre in the 0.2 m under the lattice row on
    // the odd metre above, where node placement does not land (see
    // NodelessCorridorFixture), so neither carries a node. The escape fill
    // from the start crosses both lanes, at y = 2.8 and y = 6.8.
    for (int x = 30; x < 50; ++x) {
      for (int y = 0; y < kHeight; ++y) {
        if ((y >= 11 && y < 17) || (y >= 31 && y < 37)) {
          continue;
        }
        fill(x, y);
      }
    }
    // The divider behind the wall, y in [3.8, 4.8), closes the pocket the
    // dead-end corridor opens into off the goal's room.
    for (int x = 50; x < kWidth; ++x) {
      for (int y = 19; y < 24; ++y) {
        fill(x, y);
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.flight_envelope.maximum_target_z_m = 2.0;
    config.feasibility_first_enabled = true;
    config.maximum_compute_time_ms = 200.0;
    config.maximum_no_route_compute_time_ms = 200.0;
    config.maximum_feasibility_compute_time_ms = 100.0;
    config.departure_refinement_subdivisions = 4U;
    config.escape_search_radius_cells = 4U;
    config.escape_search_maximum_probes_per_update = 1U << 16U;
  }
};

TEST(PersistentDStarLitePlanner3DTest, AnEscapeIntoADeadEndIsRetiredForTheWayOn) {
  DeadEndExitFixture fixture;
  PersistentDStarLitePlanner3D planner{fixture.config};
  std::size_t escapes_found = 0U;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 80; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    escapes_found += update.telemetry.escape_search_found ? 1U : 0U;
    if (update.publishable()) {
      break;
    }
  }
  ASSERT_TRUE(update.publishable()) << "no route on past the dead end";
  EXPECT_GE(escapes_found, 2U) << "the dead-end exit was never retired";
  EXPECT_TRUE(update.telemetry.escape_connection_active);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 2U);
  std::string described;
  for (const Point3& point : points) {
    described += "(" + std::to_string(point.x) + "," + std::to_string(point.y) + "," +
                 std::to_string(point.z) + ") ";
  }
  EXPECT_NEAR(points.back().x, fixture.goal.x, 1.0e-9);
  // The chain through the corridor is shortcut into one segment, so the
  // crossing is read off the segment that spans the wall.
  bool crosses_corridor_on = false;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const Point3& first = points[index - 1U];
    const Point3& second = points[index];
    if (std::min(first.x, second.x) > 6.0 || std::max(first.x, second.x) < 10.0 ||
        first.x == second.x) {
      continue;
    }
    const double ratio = (8.0 - first.x) / (second.x - first.x);
    const double y_at_wall = std::lerp(first.y, second.y, ratio);
    crosses_corridor_on = crosses_corridor_on || (y_at_wall > 6.6 && y_at_wall < 7.0);
  }
  EXPECT_TRUE(crosses_corridor_on)
      << "the route does not pass the corridor on: " << described;
  EXPECT_FALSE(std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 10.0 && point.y < 3.8;
  })) << "the route enters the dead end";
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

} // namespace

// The departure envelope decides first; the hull decides only where the
// envelope admits no anchor at all. A vehicle resting in a slot narrower than
// the envelope still leaves it, and an ordinary departure keeps the envelope.
TEST(PersistentDStarLitePlanner3DTest, TheDepartureFallsBackToTheHullOnlyWhenItMust) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 0.25, 80, 40, 24});
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  config.minimum_vertical_step_m = 2.0;
  config.physical_footprint.radius_m = 0.8;
  config.physical_footprint.body_radius_m = 0.4;
  config.physical_footprint.perimeter_samples = 8U;
  config.physical_footprint.radial_rings = 1U;
  config.physical_footprint.axial_samples = 1U;
  config.physical_footprint.sweep_step_m = 0.1;
  const Point3 start{3.0, 3.0, 3.0};
  const Point3 goal{17.0, 3.0, 3.0};

  // Open world: the envelope admits the departure and the hull is not asked.
  PersistentDStarLitePlanner3D open_planner{config};
  PlannerUpdate3D open_update =
      open_planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !open_update.publishable(); ++attempt) {
    open_update = open_planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(open_update.publishable());
  EXPECT_FALSE(open_update.telemetry.departure_hull_fallback);

  // A wall the vehicle has come to rest against, closer than the envelope the
  // legs answer to and further than the hull it measures. The lattice rows the
  // legs reach stand clear of it, so only the leg is in question.
  for (int x = 0; x < 80; ++x) {
    for (int z = 0; z < 24; ++z) {
      ASSERT_TRUE(occupancy->setState({x, 7, z}, ObservedVoxelState::kOccupied));
    }
  }
  const Point3 boxed_start{3.0, 2.55, 3.0};
  PersistentDStarLitePlanner3D boxed_planner{config};
  PlannerUpdate3D boxed_update =
      boxed_planner.plan(request(boxed_start, goal, world(occupancy, 2U)));
  for (int attempt = 0; attempt < 8 && !boxed_update.publishable(); ++attempt) {
    boxed_update = boxed_planner.plan(request(boxed_start, goal, world(occupancy, 2U)));
  }

  // The departure exists at all only because the hull was asked for it, and
  // the route that leaves through it is published: a path validation that
  // still demanded the envelope on the same leg would refuse every candidate.
  EXPECT_TRUE(boxed_update.telemetry.departure_hull_fallback);
  EXPECT_NE(boxed_update.input_status, PlannerInputStatus3D::kStartUnavailable);
  for (int attempt = 0; attempt < 12 && !boxed_update.publishable(); ++attempt) {
    boxed_update = boxed_planner.plan(request(boxed_start, goal, world(occupancy, 2U)));
  }
  EXPECT_TRUE(boxed_update.publishable());
}

TEST(PersistentDStarLitePlanner3DTest, TheClearanceFieldReproducesTheRawNodeClearance) {
  // A lattice node sits on a voxel corner, where the field's bracketing
  // centres give the raw clearance exactly; elsewhere the field gives a lower
  // bound within half a voxel diagonal. The launch-support contact cells the
  // field leaves out count as they do on the raw grid, and a point outside
  // the field's window falls back to the raw grid.
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 64, 64, 32};
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(bounds);
  for (int y = 0; y < bounds.height_cells; ++y) {
    for (int z = 0; z < bounds.depth_cells; ++z) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{40, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  ASSERT_TRUE(
      occupancy->setState(GridIndex3D{20, 44, 12}, ObservedVoxelState::kOccupied));
  const GridIndex3D support_cell{20, 20, 3};
  ASSERT_TRUE(occupancy->setState(support_cell, ObservedVoxelState::kOccupied));
  LaunchSupportContact3D contact;
  contact.contact_cells.push_back(
      AxisAlignedBox3D{.minimum = {5.0, 5.0, 0.75}, .maximum = {5.25, 5.25, 1.0}});
  const GridBounds3D window{2.0, 2.0, 1.0, 0.25, 40, 40, 20};
  const std::vector<GridIndex3D> suppressed{support_cell};
  const std::shared_ptr<const KnownObstacleDistance3D> field =
      buildKnownObstacleDistance3D(*occupancy, window, 7.0, suppressed).field;
  ASSERT_NE(field, nullptr);
  ASSERT_TRUE(field->valid());

  PersistentPlannerConfig3D config = testConfig();
  config.clearance_ranking_weight = 1.0;
  config.clearance_ranking_distance_m = 6.0;
  PersistentPlannerWorld3D raw_world = world(occupancy, 1U);
  raw_world.launch_support_contact = contact;
  PersistentPlannerWorld3D field_world = raw_world;
  field_world.observed_clearance_field = field;
  detail::PlannerLattice3D raw_lattice{config};
  raw_lattice.configureGridGeometry(bounds);
  raw_lattice.installWorld(raw_world);
  detail::PlannerLattice3D field_lattice{config};
  field_lattice.configureGridGeometry(bounds);
  field_lattice.installWorld(field_world);

  // The wall, the pillar's corner, and the contact cell each decide a node.
  EXPECT_NEAR(raw_lattice.nodeClearanceM(detail::PersistentPlannerNode3D{9, 5, 2}), 0.5,
              1.0e-9);
  EXPECT_NEAR(raw_lattice.nodeClearanceM(detail::PersistentPlannerNode3D{4, 10, 2}),
              std::sqrt(0.75), 1.0e-9);
  EXPECT_NEAR(raw_lattice.nodeClearanceM(detail::PersistentPlannerNode3D{4, 4, 1}),
              std::sqrt(0.75), 1.0e-9);
  for (int x = 2; x <= 13; ++x) {
    for (int y = 2; y <= 13; ++y) {
      for (int z = 1; z <= 7; ++z) {
        const detail::PersistentPlannerNode3D node{x, y, z};
        EXPECT_NEAR(field_lattice.nodeClearanceM(node),
                    raw_lattice.nodeClearanceM(node), 1.0e-5)
            << "node " << x << ' ' << y << ' ' << z;
      }
    }
  }
  const Point3 off_corner{7.3, 6.1, 2.2};
  const double raw_m = raw_lattice.pointClearanceM(off_corner);
  const double field_m = field_lattice.pointClearanceM(off_corner);
  EXPECT_LE(field_m, raw_m + 1.0e-6);
  EXPECT_GE(field_m, raw_m - 0.125 * std::sqrt(3.0) - 1.0e-6);
}

// Every search runs out: the frontier exhausts on the far side of a doorway
// no lattice edge crosses, and the fill's own reach ends short of it. The
// vehicle flew in through that doorway, so its trail is a corridor its body
// has already swept, and the retreat walks back along it instead of standing
// there.
TEST(PersistentDStarLitePlanner3DTest,
     TheRetreatWalksBackAlongTheTrailWhenEverySearchRunsOut) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 0.25, 80, 48, 24});
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  config.minimum_vertical_step_m = 2.0;
  config.feasibility_first_enabled = true;
  config.physical_footprint.radius_m = 0.4;
  config.physical_footprint.body_radius_m = 0.4;
  config.physical_footprint.perimeter_samples = 8U;
  config.physical_footprint.radial_rings = 1U;
  config.physical_footprint.axial_samples = 1U;
  config.physical_footprint.sweep_step_m = 0.1;
  config.flight_envelope.maximum_target_z_m = 6.0;
  config.maximum_compute_time_ms = 60.0;
  config.maximum_feasibility_compute_time_ms = 25.0;
  // The fill reaches one lattice step; the doorway is three metres behind the
  // vehicle, so only the trail leads out.
  config.escape_search_radius_cells = 1U;
  config.escape_search_maximum_probes_per_update = 1U << 16U;
  // A wall at x = 8 with a doorway at y in [5.25, 6.25): the lattice rows lie
  // at y = 1, 3, 5 and 7, and the body clears none of them inside it, so no
  // lattice edge crosses the wall.
  for (int y = 0; y < 48; ++y) {
    for (int z = 0; z < 24; ++z) {
      if (y >= 21 && y < 25) {
        continue;
      }
      ASSERT_TRUE(occupancy->setState({32, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  const Point3 goal{3.0, 5.75, 3.0};
  PersistentDStarLitePlanner3D planner{config};
  // The vehicle flies east through the doorway; every update records where it
  // stands.
  for (double x = 5.0; x <= 11.0; x += 0.5) {
    static_cast<void>(
        planner.plan(request(Point3{x, 5.75, 3.0}, goal, world(occupancy, 1U))));
  }

  // The consumer refuses the route it was handed -- the vehicle cannot enter
  // it from where it now stands -- so the searches start again from here.
  const Point3 trapped{11.0, 5.75, 3.0};
  bool retreat_taken = false;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 60 && !retreat_taken; ++attempt) {
    PersistentPlannerRequest3D trapped_request =
        request(trapped, goal, world(occupancy, 1U));
    trapped_request.incumbent_rejection_sequence = 1U;
    update = planner.plan(trapped_request);
    retreat_taken = update.telemetry.escape_connection_active;
  }
  EXPECT_TRUE(retreat_taken)
      << "the vehicle stands where its own trail leads out: frontier_exhausted="
      << update.telemetry.feasibility_frontier_exhausted
      << " escape_exhausted=" << update.telemetry.escape_search_exhausted;
}

} // namespace drone_city_nav
