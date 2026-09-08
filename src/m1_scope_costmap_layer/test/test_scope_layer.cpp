#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "m1_scope_costmap_layer/scope_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace allocation_probe
{

std::atomic<bool> enabled{false};
std::atomic<size_t> count{0};

}  // namespace allocation_probe

void * operator new(std::size_t size)
{
  if (allocation_probe::enabled.load(std::memory_order_relaxed)) {
    allocation_probe::count.fetch_add(1, std::memory_order_relaxed);
  }
  if (void * memory = std::malloc(size == 0 ? 1 : size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size)
{
  return ::operator new(size);
}

void operator delete(void * memory) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::size_t) noexcept
{
  std::free(memory);
}

namespace m1_scope_costmap_layer
{

class ScopeLayerTestPeer
{
public:
  static void prediction(
    ScopeLayer & layer, const nav_msgs::msg::OccupancyGrid & message,
    const rclcpp::Time & received_at)
  {
    layer.bufferPrediction(message, received_at);
  }

  static void uncertainty(
    ScopeLayer & layer, const nav_msgs::msg::OccupancyGrid & message,
    const rclcpp::Time & received_at)
  {
    layer.bufferUncertainty(message, received_at);
  }

  static bool hasSnapshot(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return static_cast<bool>(layer.latest_snapshot_);
  }

  static int64_t snapshotStamp(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.latest_snapshot_ ? layer.latest_snapshot_->stamp.nanoseconds() : -1;
  }

  static size_t pendingCount(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.pending_predictions_.size() + layer.pending_uncertainties_.size();
  }

  static uint64_t generation(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.subscription_generation_;
  }

  static void predictionForGeneration(
    ScopeLayer & layer, const nav_msgs::msg::OccupancyGrid & message,
    const rclcpp::Time & received_at, uint64_t generation)
  {
    layer.bufferGrid(message, received_at, true, generation);
  }

  static void uncertaintyForGeneration(
    ScopeLayer & layer, const nav_msgs::msg::OccupancyGrid & message,
    const rclcpp::Time & received_at, uint64_t generation)
  {
    layer.bufferGrid(message, received_at, false, generation);
  }

  static std::string configuredPredictionTopic(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.config_.prediction_topic;
  }

  static std::string configuredUncertaintyTopic(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.config_.uncertainty_topic;
  }

  static std::string predictionSubscriptionTopic(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.prediction_subscription_->get_topic_name();
  }

  static std::string uncertaintySubscriptionTopic(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.uncertainty_subscription_->get_topic_name();
  }

  static double staleTimeout(const ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.config_.stale_timeout;
  }

  static void makeSnapshotStale(ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    ASSERT_TRUE(layer.latest_snapshot_);
    layer.latest_snapshot_->received_at = layer.clock_->now() - rclcpp::Duration::from_seconds(2.0);
  }

  static void makeSnapshotFromFuture(ScopeLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    ASSERT_TRUE(layer.latest_snapshot_);
    layer.latest_snapshot_->received_at = layer.clock_->now() + rclcpp::Duration::from_seconds(2.0);
  }
};

namespace
{

constexpr double kPi = 3.14159265358979323846;

nav_msgs::msg::OccupancyGrid makeGrid(
  int64_t stamp_ns, const std::vector<int8_t> & data,
  unsigned int width = 1, unsigned int height = 1, double resolution = 0.05,
  double origin_x = 0.0, double origin_y = 0.0, double yaw = 0.0,
  const std::string & frame = "odom")
{
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  message.header.frame_id = frame;
  message.info.width = width;
  message.info.height = height;
  message.info.resolution = resolution;
  message.info.origin.position.x = origin_x;
  message.info.origin.position.y = origin_y;
  message.info.origin.orientation.z = std::sin(yaw * 0.5);
  message.info.origin.orientation.w = std::cos(yaw * 0.5);
  message.data = data;
  return message;
}

std::set<std::pair<unsigned int, unsigned int>> cellsWithCost(
  const nav2_costmap_2d::Costmap2D & map, unsigned char cost)
{
  std::set<std::pair<unsigned int, unsigned int>> cells;
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
      if (map.getCost(x, y) == cost) {
        cells.emplace(x, y);
      }
    }
  }
  return cells;
}

struct BaseCell
{
  double world_x;
  double world_y;
  unsigned char cost;
};

class FixedBaseLayer : public nav2_costmap_2d::Layer
{
public:
  explicit FixedBaseLayer(std::vector<BaseCell> cells)
  : cells_(std::move(cells))
  {
    current_ = true;
    enabled_ = true;
  }

  void updateBounds(
    double, double, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    for (const auto & cell : cells_) {
      *min_x = std::min(*min_x, cell.world_x - 0.025);
      *min_y = std::min(*min_y, cell.world_y - 0.025);
      *max_x = std::max(*max_x, cell.world_x + 0.025);
      *max_y = std::max(*max_y, cell.world_y + 0.025);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    for (const auto & cell : cells_) {
      unsigned int map_x = 0;
      unsigned int map_y = 0;
      if (master.worldToMap(cell.world_x, cell.world_y, map_x, map_y) &&
        static_cast<int>(map_x) >= min_i && static_cast<int>(map_x) < max_i &&
        static_cast<int>(map_y) >= min_j && static_cast<int>(map_y) < max_j)
      {
        master.setCost(map_x, map_y, cell.cost);
      }
    }
  }

  void reset() override {}
  bool isClearable() override {return false;}

private:
  std::vector<BaseCell> cells_;
};

class LayeredHarness
{
public:
  LayeredHarness(
    bool rolling, std::vector<BaseCell> base_cells,
    unsigned int size_x = 20, unsigned int size_y = 20,
    double origin_x = 0.0, double origin_y = 0.0)
  {
    const auto id = counter_.fetch_add(1);
    node = std::make_shared<nav2_util::LifecycleNode>(
      "scope_layer_integration_" + std::to_string(id));
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    layered = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", rolling, false);
    layered->resizeMap(size_x, size_y, 0.05, origin_x, origin_y);
    base = std::make_shared<FixedBaseLayer>(std::move(base_cells));
    layered->addPlugin(base);
    base->initialize(layered.get(), "base", tf.get(), node, callback_group);
    scope = std::make_shared<ScopeLayer>();
    layered->addPlugin(scope);
    scope->initialize(layered.get(), "scope", tf.get(), node, callback_group);
  }

  void pairAt(
    const nav_msgs::msg::OccupancyGrid & prediction,
    const nav_msgs::msg::OccupancyGrid & uncertainty,
    const rclcpp::Time & received_at)
  {
    ScopeLayerTestPeer::prediction(*scope, prediction, received_at);
    ScopeLayerTestPeer::uncertainty(*scope, uncertainty, received_at);
  }

  void pair(
    const nav_msgs::msg::OccupancyGrid & prediction,
    const nav_msgs::msg::OccupancyGrid & uncertainty)
  {
    pairAt(prediction, uncertainty, node->get_clock()->now());
  }

  static std::atomic<unsigned int> counter_;
  nav2_util::LifecycleNode::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered;
  std::shared_ptr<FixedBaseLayer> base;
  std::shared_ptr<ScopeLayer> scope;
};

std::atomic<unsigned int> LayeredHarness::counter_{0};

TEST(RasterStatsDiagnostics, FormatsStableParseableCounts)
{
  const RasterStats stats{17, 3, 5, 2};

  const auto diagnostic = formatRasterStats(stats);

  EXPECT_STREQ(
    diagnostic.data(),
    "SCOPE raster stats valid=17 clipped=3 medium=5 lethal=2");
}

class ScopeLayerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto id = counter_.fetch_add(1);
    node_ = std::make_shared<nav2_util::LifecycleNode>(
      "scope_layer_test_" + std::to_string(id));
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    layered_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, false);
    layered_->resizeMap(40, 40, 0.05, 0.0, 0.0);
    layer_ = std::make_shared<ScopeLayer>();
    layered_->addPlugin(layer_);
    layer_->initialize(layered_.get(), "scope", tf_.get(), node_, callback_group_);
  }

  void pair(
    const nav_msgs::msg::OccupancyGrid & prediction,
    const nav_msgs::msg::OccupancyGrid & uncertainty)
  {
    const auto now = node_->get_clock()->now();
    ScopeLayerTestPeer::prediction(*layer_, prediction, now);
    ScopeLayerTestPeer::uncertainty(*layer_, uncertainty, now);
  }

  void update(nav2_costmap_2d::Costmap2D & master)
  {
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
    layer_->updateCosts(
      master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
      static_cast<int>(master.getSizeInCellsY()));
  }

  static std::atomic<unsigned int> counter_;
  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_;
  std::shared_ptr<ScopeLayer> layer_;
};

std::atomic<unsigned int> ScopeLayerFixture::counter_{0};

TEST_F(ScopeLayerFixture, FusesPredictionPlusOneSigmaAtExactThresholds)
{
  const auto prediction = makeGrid(100, {20, 20, 30, 40}, 4, 1);
  const auto uncertainty = makeGrid(100, {29, 30, 60, 40}, 4, 1);
  pair(prediction, uncertainty);

  nav2_costmap_2d::Costmap2D master(
    8, 2, static_cast<double>(prediction.info.resolution), 0.0, 0.0, 0);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), 0);    // 0.345 is transparent.
  EXPECT_EQ(master.getCost(1, 0), 200);  // 0.350 is medium.
  EXPECT_EQ(master.getCost(2, 0), 254);  // 0.600 is lethal.
  EXPECT_EQ(master.getCost(3, 0), 254);  // p=0.4 plus sigma=0.2.
  const auto stats = layer_->getLastStats();
  EXPECT_EQ(stats.valid_source_cells, 4u);
  EXPECT_EQ(stats.medium_master_cells, 1u);
  EXPECT_EQ(stats.lethal_master_cells, 2u);
}

TEST_F(ScopeLayerFixture, PairsAsynchronouslyByExactStampAndGeometry)
{
  const auto now = node_->get_clock()->now();
  const auto prediction = makeGrid(100, {80});
  ScopeLayerTestPeer::prediction(*layer_, prediction, now);
  ScopeLayerTestPeer::uncertainty(*layer_, makeGrid(200, {0}), now);
  EXPECT_FALSE(ScopeLayerTestPeer::hasSnapshot(*layer_));

  ScopeLayerTestPeer::uncertainty(
    *layer_, makeGrid(100, {0}, 1, 1, 0.10), now);
  EXPECT_FALSE(ScopeLayerTestPeer::hasSnapshot(*layer_));

  ScopeLayerTestPeer::uncertainty(*layer_, makeGrid(100, {0}), now);
  EXPECT_TRUE(ScopeLayerTestPeer::hasSnapshot(*layer_));
  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);
}

TEST_F(ScopeLayerFixture, RejectsFrameMismatchAndInvalidMessagesWithoutReplacingSnapshot)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));
  ASSERT_TRUE(ScopeLayerTestPeer::hasSnapshot(*layer_));

  pair(
    makeGrid(200, {80}, 1, 1, 0.05, 0.0, 0.0, 0.0, "map"),
    makeGrid(200, {0}, 1, 1, 0.05, 0.0, 0.0, 0.0, "map"));
  pair(makeGrid(300, {101}), makeGrid(300, {0}));
  pair(makeGrid(400, {}, 1, 1), makeGrid(400, {0}));
  auto nonfinite = makeGrid(500, {80});
  nonfinite.info.origin.position.x = std::numeric_limits<double>::quiet_NaN();
  pair(nonfinite, makeGrid(500, {0}));

  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);
}

TEST_F(ScopeLayerFixture, TreatsUnknownCellsAsTransparentAndBoundsPendingQueues)
{
  const auto now = node_->get_clock()->now();
  for (int64_t stamp = 1; stamp <= 20; ++stamp) {
    ScopeLayerTestPeer::prediction(*layer_, makeGrid(stamp, {-1}), now);
  }
  EXPECT_LE(ScopeLayerTestPeer::pendingCount(*layer_), 8u);

  ScopeLayerTestPeer::uncertainty(*layer_, makeGrid(20, {100}), now);
  ASSERT_TRUE(ScopeLayerTestPeer::hasSnapshot(*layer_));
  nav2_costmap_2d::Costmap2D master(2, 2, 0.05, 0.0, 0.0, 77);
  update(master);
  EXPECT_EQ(master.getCost(0, 0), 77);
}

TEST_F(ScopeLayerFixture, RejectsNegativeValuesOtherThanMinusOneWithoutReplacingSnapshot)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));
  ASSERT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);

  pair(makeGrid(200, {-2}), makeGrid(200, {0}));
  pair(makeGrid(300, {80}), makeGrid(300, {-128}));

  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);
}

TEST_F(ScopeLayerFixture, ConservativelyRasterizesEncodedPointOneCell)
{
  pair(makeGrid(100, {80}, 1, 1, 0.10), makeGrid(100, {0}, 1, 1, 0.10));
  nav2_costmap_2d::Costmap2D master(6, 6, 0.05, 0.0, 0.0, 0);
  update(master);

  const std::set<std::pair<unsigned int, unsigned int>> expected{
    {0, 0}, {1, 0}, {2, 0},
    {0, 1}, {1, 1}, {2, 1},
    {0, 2}, {1, 2}, {2, 2}};
  EXPECT_EQ(cellsWithCost(master, nav2_costmap_2d::LETHAL_OBSTACLE), expected);
}

TEST_F(ScopeLayerFixture, RasterizesNinetyDegreeRotatedOrigin)
{
  pair(
    makeGrid(100, {80}, 1, 1, 0.10, 0.10, 0.0, kPi / 2.0),
    makeGrid(100, {0}, 1, 1, 0.10, 0.10, 0.0, kPi / 2.0));
  nav2_costmap_2d::Costmap2D master(6, 6, 0.05, 0.0, 0.0, 0);
  update(master);

  const std::set<std::pair<unsigned int, unsigned int>> expected{
    {0, 0}, {1, 0}, {2, 0},
    {0, 1}, {1, 1}, {2, 1},
    {0, 2}, {1, 2}, {2, 2}};
  EXPECT_EQ(cellsWithCost(master, nav2_costmap_2d::LETHAL_OBSTACLE), expected);
}

TEST_F(ScopeLayerFixture, RasterizesArbitraryYawByPositiveAreaIntersection)
{
  pair(
    makeGrid(100, {80}, 1, 1, 0.10, 0.20, 0.20, kPi / 4.0),
    makeGrid(100, {0}, 1, 1, 0.10, 0.20, 0.20, kPi / 4.0));
  nav2_costmap_2d::Costmap2D master(10, 10, 0.05, 0.0, 0.0, 0);
  update(master);

  const std::set<std::pair<unsigned int, unsigned int>> expected{
    {2, 4}, {3, 4}, {4, 4}, {2, 5}, {3, 5}, {4, 5}, {5, 5}, {3, 6}, {4, 6}};
  EXPECT_EQ(cellsWithCost(master, nav2_costmap_2d::LETHAL_OBSTACLE), expected);
}

TEST_F(ScopeLayerFixture, ClipsRasterizationToMasterAndUpdateWindow)
{
  pair(
    makeGrid(100, {80}, 1, 1, 0.10, -0.025, -0.025),
    makeGrid(100, {0}, 1, 1, 0.10, -0.025, -0.025));
  nav2_costmap_2d::Costmap2D master(2, 2, 0.05, 0.0, 0.0, 0);

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  layer_->updateCosts(master, 1, 0, 2, 2);

  EXPECT_EQ(
    cellsWithCost(master, nav2_costmap_2d::LETHAL_OBSTACLE),
    (std::set<std::pair<unsigned int, unsigned int>>{{1, 0}, {1, 1}}));
  EXPECT_EQ(layer_->getLastStats().clipped_source_cells, 1u);
}

TEST_F(ScopeLayerFixture, KeepsRepresentableTinyPositiveOverlap)
{
  constexpr double tiny_overlap = 1e-9;
  pair(
    makeGrid(100, {80}, 1, 1, 0.05, -0.05 + tiny_overlap, 0.0),
    makeGrid(100, {0}, 1, 1, 0.05, -0.05 + tiny_overlap, 0.0));
  nav2_costmap_2d::Costmap2D master(1, 1, 0.05, 0.0, 0.0, 0);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ScopeLayerFixture, KeepsRepresentablePositiveOverlapAtLargeCoordinates)
{
  const double source_origin = std::nextafter(999.95, std::numeric_limits<double>::infinity());
  pair(
    makeGrid(100, {80}, 1, 1, 0.05, source_origin, 0.0),
    makeGrid(100, {0}, 1, 1, 0.05, source_origin, 0.0));
  nav2_costmap_2d::Costmap2D master(1, 1, 0.05, 1000.0, 0.0, 0);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ScopeLayerFixture, KeepsPositiveOverlapWhenCandidateBoundRoundsToInteger)
{
  constexpr double exact_binary_resolution = 0.0625;
  const double source_edge = std::nextafter(0.45, std::numeric_limits<double>::infinity());
  const double source_origin = source_edge - exact_binary_resolution;
  pair(
    makeGrid(
      100, {80}, 1, 1, exact_binary_resolution,
      source_origin, 0.0),
    makeGrid(
      100, {0}, 1, 1, exact_binary_resolution,
      source_origin, 0.0));
  nav2_costmap_2d::Costmap2D master(10, 1, 0.05, 0.0, 0.0, 0);
  update(master);

  EXPECT_EQ(master.getCost(9, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ScopeLayerFixture, TreatsExactBoundaryOnlyContactAsTransparent)
{
  constexpr double exact_binary_resolution = 0.0625;
  pair(
    makeGrid(
      100, {80}, 1, 1, exact_binary_resolution,
      -exact_binary_resolution, 0.0),
    makeGrid(
      100, {0}, 1, 1, exact_binary_resolution,
      -exact_binary_resolution, 0.0));
  nav2_costmap_2d::Costmap2D master(1, 1, 0.05, 0.0, 0.0, 0);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ScopeLayerFixture, KeepsPositiveOverlapFromRepresentableSmallYaw)
{
  constexpr double exact_binary_resolution = 0.0625;
  const double small_negative_yaw = -8.0 * std::numeric_limits<double>::epsilon();
  pair(
    makeGrid(
      100, {80}, 1, 1, exact_binary_resolution,
      -exact_binary_resolution, 0.0, small_negative_yaw),
    makeGrid(
      100, {0}, 1, 1, exact_binary_resolution,
      -exact_binary_resolution, 0.0, small_negative_yaw));
  nav2_costmap_2d::Costmap2D master(1, 1, 0.05, 0.0, 0.0, 0);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ScopeLayerFixture, CropsFarOutSourceBeforeRasterIndexConversion)
{
  pair(
    makeGrid(100, {80}, 1, 1, 0.10, 1e9, -1e9),
    makeGrid(100, {0}, 1, 1, 0.10, 1e9, -1e9));
  nav2_costmap_2d::Costmap2D master(2, 2, 0.05, 0.0, 0.0, 0);
  update(master);

  EXPECT_TRUE(cellsWithCost(master, nav2_costmap_2d::LETHAL_OBSTACLE).empty());
  EXPECT_EQ(layer_->getLastStats().valid_source_cells, 1u);
  EXPECT_EQ(layer_->getLastStats().clipped_source_cells, 1u);
}

TEST_F(ScopeLayerFixture, CountsClippedValidCellsEvenWhenRiskIsTransparent)
{
  pair(
    makeGrid(100, {0}, 1, 1, 0.10, -0.025, -0.025),
    makeGrid(100, {0}, 1, 1, 0.10, -0.025, -0.025));
  nav2_costmap_2d::Costmap2D master(2, 2, 0.05, 0.0, 0.0, 0);
  update(master);

  const auto stats = layer_->getLastStats();
  EXPECT_EQ(stats.valid_source_cells, 1u);
  EXPECT_EQ(stats.clipped_source_cells, 1u);
  EXPECT_EQ(stats.medium_master_cells, 0u);
  EXPECT_EQ(stats.lethal_master_cells, 0u);
}

TEST_F(ScopeLayerFixture, CountsOnlyScopeCostsActuallyWrittenToMaster)
{
  pair(makeGrid(100, {40, 80}, 2, 1), makeGrid(100, {0, 0}, 2, 1));
  nav2_costmap_2d::Costmap2D master(2, 1, 0.05, 0.0, 0.0, 0);
  master.setCost(0, 0, 230);
  master.setCost(1, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), 230);
  EXPECT_EQ(master.getCost(1, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(layer_->getLastStats().medium_master_cells, 0u);
  EXPECT_EQ(layer_->getLastStats().lethal_master_cells, 0u);
}

TEST_F(ScopeLayerFixture, TransparentAndMaxCombinationNeverClearMasterObstacles)
{
  pair(makeGrid(100, {10, 80}, 2, 1), makeGrid(100, {0, 0}, 2, 1));
  nav2_costmap_2d::Costmap2D master(4, 2, 0.05, 0.0, 0.0, 0);
  master.setCost(0, 0, 230);
  master.setCost(1, 0, nav2_costmap_2d::NO_INFORMATION);
  update(master);

  EXPECT_EQ(master.getCost(0, 0), 230);
  EXPECT_EQ(master.getCost(1, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ScopeLayerFixture, StaleTransitionRequestsOldBoundsAndWithdrawsOnRecomposition)
{
  pair(makeGrid(100, {80}, 1, 1, 0.10), makeGrid(100, {0}, 1, 1, 0.10));
  nav2_costmap_2d::Costmap2D master(4, 4, 0.05, 0.0, 0.0, 0);
  update(master);
  ASSERT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  ScopeLayerTestPeer::makeSnapshotStale(*layer_);
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_LE(min_x, 0.0);
  EXPECT_LE(min_y, 0.0);
  EXPECT_GE(max_x, 0.10);
  EXPECT_GE(max_y, 0.10);

  // This reset plus base-layer rewrite models LayeredCostmap's recomposition.
  master.resetMap(0, 0, 4, 4);
  master.setCost(0, 0, 123);
  layer_->updateCosts(master, 0, 0, 4, 4);
  EXPECT_EQ(master.getCost(0, 0), 123);
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(ScopeLayerFixture, NewFrameBoundsCoverOldAndNewOrigins)
{
  pair(makeGrid(100, {80}, 1, 1, 0.10), makeGrid(100, {0}, 1, 1, 0.10));
  nav2_costmap_2d::Costmap2D master(20, 4, 0.05, 0.0, 0.0, 0);
  update(master);

  pair(
    makeGrid(200, {80}, 1, 1, 0.10, 0.50, 0.0),
    makeGrid(200, {0}, 1, 1, 0.10, 0.50, 0.0));
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_LE(min_x, 0.0);
  EXPECT_GE(max_x, 0.60);
}

TEST_F(ScopeLayerFixture, ClockRollbackWithdrawsAndForgetsFutureReceiptSnapshot)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));
  ScopeLayerTestPeer::makeSnapshotFromFuture(*layer_);

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  EXPECT_FALSE(ScopeLayerTestPeer::hasSnapshot(*layer_));
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(ScopeLayerFixture, ClockRollbackAllowsNewPairWithLowerHeaderStamp)
{
  const auto future_receipt = node_->get_clock()->now() + rclcpp::Duration::from_seconds(2.0);
  ScopeLayerTestPeer::prediction(*layer_, makeGrid(100, {80}), future_receipt);
  ScopeLayerTestPeer::uncertainty(*layer_, makeGrid(100, {0}), future_receipt);
  ASSERT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);

  const auto new_epoch_receipt = node_->get_clock()->now();
  ScopeLayerTestPeer::prediction(*layer_, makeGrid(10, {40}), new_epoch_receipt);
  ScopeLayerTestPeer::uncertainty(*layer_, makeGrid(10, {0}), new_epoch_receipt);

  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 10);
}

TEST_F(ScopeLayerFixture, EnabledTransitionWithdrawsAndReenableReappliesFreshSnapshot)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));
  nav2_costmap_2d::Costmap2D master(4, 4, 0.05, 0.0, 0.0, 0);
  update(master);
  ASSERT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  auto result = node_->set_parameter(rclcpp::Parameter("scope.enabled", false));
  ASSERT_TRUE(result.successful) << result.reason;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_LE(min_x, 0.0);
  EXPECT_GE(max_x, 0.05);
  master.resetMap(0, 0, 4, 4);
  master.setCost(0, 0, 111);
  layer_->updateCosts(master, 0, 0, 4, 4);
  EXPECT_EQ(master.getCost(0, 0), 111);
  EXPECT_FALSE(layer_->isEnabled());

  result = node_->set_parameter(rclcpp::Parameter("scope.enabled", true));
  ASSERT_TRUE(result.successful) << result.reason;
  update(master);
  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(layer_->isEnabled());
}

TEST_F(ScopeLayerFixture, TopicSwapRejectsInvalidNameWithoutMutatingActiveState)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));
  const auto old_generation = ScopeLayerTestPeer::generation(*layer_);
  const auto old_prediction_config = ScopeLayerTestPeer::configuredPredictionTopic(*layer_);
  const auto old_uncertainty_config = ScopeLayerTestPeer::configuredUncertaintyTopic(*layer_);
  const auto old_prediction_subscription =
    ScopeLayerTestPeer::predictionSubscriptionTopic(*layer_);
  const auto old_uncertainty_subscription =
    ScopeLayerTestPeer::uncertaintySubscriptionTopic(*layer_);

  rcl_interfaces::msg::SetParametersResult result;
  EXPECT_NO_THROW(
    result = node_->set_parameter(
      rclcpp::Parameter("scope.prediction_topic", "invalid topic")));

  EXPECT_FALSE(result.successful);
  EXPECT_EQ(ScopeLayerTestPeer::generation(*layer_), old_generation);
  EXPECT_EQ(ScopeLayerTestPeer::configuredPredictionTopic(*layer_), old_prediction_config);
  EXPECT_EQ(ScopeLayerTestPeer::configuredUncertaintyTopic(*layer_), old_uncertainty_config);
  EXPECT_EQ(
    ScopeLayerTestPeer::predictionSubscriptionTopic(*layer_), old_prediction_subscription);
  EXPECT_EQ(
    ScopeLayerTestPeer::uncertaintySubscriptionTopic(*layer_), old_uncertainty_subscription);
  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);
}

TEST_F(ScopeLayerFixture, TopicSwapIsGenerationSafeAndWithdrawsOldSnapshot)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));
  nav2_costmap_2d::Costmap2D master(4, 4, 0.05, 0.0, 0.0, 0);
  update(master);
  const auto old_generation = ScopeLayerTestPeer::generation(*layer_);

  const auto result = node_->set_parameters_atomically(
      {
        rclcpp::Parameter("scope.prediction_topic", "/scope/new_prediction"),
        rclcpp::Parameter("scope.uncertainty_topic", "/scope/new_uncertainty")
      });
  ASSERT_TRUE(result.successful) << result.reason;
  const auto new_generation = ScopeLayerTestPeer::generation(*layer_);

  EXPECT_GT(new_generation, old_generation);
  EXPECT_FALSE(ScopeLayerTestPeer::hasSnapshot(*layer_));
  EXPECT_EQ(
    ScopeLayerTestPeer::predictionSubscriptionTopic(*layer_), "/scope/new_prediction");
  EXPECT_EQ(
    ScopeLayerTestPeer::uncertaintySubscriptionTopic(*layer_), "/scope/new_uncertainty");

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_LE(min_x, 0.0);
  EXPECT_GE(max_x, 0.05);
  master.resetMap(0, 0, 4, 4);
  layer_->updateCosts(master, 0, 0, 4, 4);
  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::FREE_SPACE);

  const auto now = node_->get_clock()->now();
  ScopeLayerTestPeer::predictionForGeneration(
    *layer_, makeGrid(200, {80}), now, old_generation);
  ScopeLayerTestPeer::uncertaintyForGeneration(
    *layer_, makeGrid(200, {0}), now, new_generation);
  EXPECT_FALSE(ScopeLayerTestPeer::hasSnapshot(*layer_));
  ScopeLayerTestPeer::uncertaintyForGeneration(
    *layer_, makeGrid(300, {0}), now, old_generation);
  ScopeLayerTestPeer::predictionForGeneration(
    *layer_, makeGrid(300, {80}), now, new_generation);
  EXPECT_FALSE(ScopeLayerTestPeer::hasSnapshot(*layer_));

  ScopeLayerTestPeer::predictionForGeneration(
    *layer_, makeGrid(400, {80}), now, new_generation);
  ScopeLayerTestPeer::uncertaintyForGeneration(
    *layer_, makeGrid(400, {0}), now, new_generation);
  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 400);
}

TEST_F(ScopeLayerFixture, RejectsInvalidTimeAndExtremeGeometryWithoutThrowing)
{
  pair(makeGrid(100, {80}), makeGrid(100, {0}));

  auto negative_stamp_prediction = makeGrid(200, {80});
  auto negative_stamp_uncertainty = makeGrid(200, {0});
  negative_stamp_prediction.header.stamp.sec = -1;
  negative_stamp_uncertainty.header.stamp.sec = -1;
  EXPECT_NO_THROW(pair(negative_stamp_prediction, negative_stamp_uncertainty));

  auto invalid_nanoseconds_prediction = makeGrid(300, {80});
  auto invalid_nanoseconds_uncertainty = makeGrid(300, {0});
  invalid_nanoseconds_prediction.header.stamp.nanosec = 1000000000u;
  invalid_nanoseconds_uncertainty.header.stamp.nanosec = 1000000000u;
  EXPECT_NO_THROW(pair(invalid_nanoseconds_prediction, invalid_nanoseconds_uncertainty));

  auto collapsed_prediction = makeGrid(
    400, {80}, 1, 1, 0.05, std::numeric_limits<double>::max(), 0.0);
  auto collapsed_uncertainty = makeGrid(
    400, {0}, 1, 1, 0.05, std::numeric_limits<double>::max(), 0.0);
  EXPECT_NO_THROW(pair(collapsed_prediction, collapsed_uncertainty));

  auto overflow_prediction = makeGrid(
    500, {80, 80}, 2, 1, std::numeric_limits<float>::max(),
    std::numeric_limits<double>::max(), 0.0);
  auto overflow_uncertainty = makeGrid(
    500, {0, 0}, 2, 1, std::numeric_limits<float>::max(),
    std::numeric_limits<double>::max(), 0.0);
  EXPECT_NO_THROW(pair(overflow_prediction, overflow_uncertainty));

  EXPECT_EQ(ScopeLayerTestPeer::snapshotStamp(*layer_), 100);
}

TEST_F(ScopeLayerFixture, RejectsUnsafeStaleTimeoutWithoutMutationOrException)
{
  const double old_timeout = ScopeLayerTestPeer::staleTimeout(*layer_);
  rcl_interfaces::msg::SetParametersResult result;
  EXPECT_NO_THROW(
    result = node_->set_parameter(
      rclcpp::Parameter("scope.stale_timeout", 1.0e20)));

  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(ScopeLayerTestPeer::staleTimeout(*layer_), old_timeout);
}

TEST_F(ScopeLayerFixture, DenseRasterizationUsesBoundedScratchAllocations)
{
  constexpr unsigned int side = 64;
  constexpr double exact_binary_resolution = 0.0625;
  const size_t cell_count = static_cast<size_t>(side) * side;
  pair(
    makeGrid(
      100, std::vector<int8_t>(cell_count, 80), side, side,
      exact_binary_resolution),
    makeGrid(
      100, std::vector<int8_t>(cell_count, 0), side, side,
      exact_binary_resolution));
  nav2_costmap_2d::Costmap2D master(
    side, side, exact_binary_resolution, 0.0, 0.0, 0);

  allocation_probe::count.store(0, std::memory_order_relaxed);
  allocation_probe::enabled.store(true, std::memory_order_relaxed);
  const auto start = std::chrono::steady_clock::now();
  update(master);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  allocation_probe::enabled.store(false, std::memory_order_relaxed);

  const auto stats = layer_->getLastStats();
  EXPECT_EQ(stats.valid_source_cells, cell_count);
  EXPECT_EQ(stats.lethal_master_cells, cell_count);
  EXPECT_EQ(master.getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(side - 1, side - 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_LE(allocation_probe::count.load(std::memory_order_relaxed), 4u);
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}

TEST_F(ScopeLayerFixture, DeclaresTheCompletePublicParameterSet)
{
  for (const auto & name : {
        "enabled", "prediction_topic", "uncertainty_topic", "low_threshold",
        "lethal_threshold", "uncertainty_gain", "uncertainty_encoding_scale",
        "medium_cost", "stale_timeout"})
  {
    EXPECT_TRUE(node_->has_parameter("scope." + std::string(name))) << name;
  }
}

TEST(ScopeLayerIntegration, OptionalLayerIsCurrentBeforeAnyPairArrives)
{
  LayeredHarness harness(false, {{0.025, 0.025, 123}});

  EXPECT_TRUE(harness.layered->isCurrent());
  harness.layered->updateMap(0.25, 0.25, 0.0);
  EXPECT_TRUE(harness.layered->isCurrent());
  EXPECT_EQ(harness.layered->getCostmap()->getCost(0, 0), 123);
}

TEST(ScopeLayerIntegration, StaleWithdrawalRecomposesBaseAndRemainsCurrent)
{
  LayeredHarness harness(false, {{0.025, 0.025, 123}});
  harness.pair(makeGrid(100, {80}), makeGrid(100, {0}));
  harness.layered->updateMap(0.25, 0.25, 0.0);
  ASSERT_EQ(
    harness.layered->getCostmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  ScopeLayerTestPeer::makeSnapshotStale(*harness.scope);
  harness.layered->updateMap(0.25, 0.25, 0.0);

  EXPECT_EQ(harness.layered->getCostmap()->getCost(0, 0), 123);
  EXPECT_TRUE(harness.scope->isCurrent());
  EXPECT_TRUE(harness.layered->isCurrent());
}

TEST(ScopeLayerIntegration, NewFrameClearsOldContributionAndRendersNewOrigin)
{
  LayeredHarness harness(false, {{0.025, 0.025, 123}});
  harness.pair(makeGrid(100, {80}), makeGrid(100, {0}));
  harness.layered->updateMap(0.25, 0.25, 0.0);
  ASSERT_EQ(
    harness.layered->getCostmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  harness.pair(
    makeGrid(200, {80}, 1, 1, 0.05, 0.20, 0.0),
    makeGrid(200, {0}, 1, 1, 0.05, 0.20, 0.0));
  harness.layered->updateMap(0.25, 0.25, 0.0);

  EXPECT_EQ(harness.layered->getCostmap()->getCost(0, 0), 123);
  EXPECT_EQ(
    harness.layered->getCostmap()->getCost(4, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(ScopeLayerIntegration, DisabledWithdrawalRecomposesBaseAndRemainsCurrent)
{
  LayeredHarness harness(false, {{0.025, 0.025, 123}});
  harness.pair(makeGrid(100, {80}), makeGrid(100, {0}));
  harness.layered->updateMap(0.25, 0.25, 0.0);
  ASSERT_EQ(
    harness.layered->getCostmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto result = harness.node->set_parameter(rclcpp::Parameter("scope.enabled", false));
  ASSERT_TRUE(result.successful) << result.reason;
  harness.layered->updateMap(0.25, 0.25, 0.0);

  EXPECT_EQ(harness.layered->getCostmap()->getCost(0, 0), 123);
  EXPECT_TRUE(harness.layered->isCurrent());
}

TEST(ScopeLayerIntegration, RollingOriginAndNewFrameLeaveNoOldScopeGhosts)
{
  LayeredHarness harness(true, {}, 8, 6, -0.20, -0.15);
  harness.pair(
    makeGrid(100, {80}, 1, 1, 0.10, -0.10, -0.05),
    makeGrid(100, {0}, 1, 1, 0.10, -0.10, -0.05));
  harness.layered->updateMap(0.0, 0.0, 0.0);

  harness.pair(
    makeGrid(200, {80}, 1, 1, 0.10, 0.10, -0.05),
    makeGrid(200, {0}, 1, 1, 0.10, 0.10, -0.05));
  harness.layered->updateMap(0.05, 0.0, 0.0);

  const auto * master = harness.layered->getCostmap();
  unsigned int old_x = 0;
  unsigned int old_y = 0;
  ASSERT_TRUE(master->worldToMap(-0.075, -0.025, old_x, old_y));
  EXPECT_NE(master->getCost(old_x, old_y), nav2_costmap_2d::LETHAL_OBSTACLE);
  unsigned int new_x = 0;
  unsigned int new_y = 0;
  ASSERT_TRUE(master->worldToMap(0.125, -0.025, new_x, new_y));
  EXPECT_EQ(master->getCost(new_x, new_y), nav2_costmap_2d::LETHAL_OBSTACLE);
  for (unsigned int y = 0; y < master->getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < master->getSizeInCellsX(); ++x) {
      if (master->getCost(x, y) != nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      double world_x = 0.0;
      double world_y = 0.0;
      master->mapToWorld(x, y, world_x, world_y);
      EXPECT_GT(world_x, 0.05) << "lethal cost remained in the old-frame-only region";
    }
  }
}

TEST(ScopeLayerPlugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto instance = loader.createSharedInstance("m1_scope_costmap_layer::ScopeLayer");
  ASSERT_NE(instance, nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<ScopeLayer>(instance), nullptr);
}

}  // namespace
}  // namespace m1_scope_costmap_layer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
