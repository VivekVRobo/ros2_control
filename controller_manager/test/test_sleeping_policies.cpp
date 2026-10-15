// Copyright 2026 Felix Exner
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "rosgraph_msgs/msg/clock.hpp"
#include "rclcpp/clock.hpp"

#include "controller_manager/sleeping_policies.hpp"
#include "controller_manager_test_common.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kMinimumCycleTimeS = 0.001;
constexpr int kUpdateRateHz = 100;
constexpr auto kTimingTolerance = 500us;

template <typename Func>
std::chrono::nanoseconds measure_execution(Func && func)
{
  const auto start = std::chrono::steady_clock::now();
  func();
  return std::chrono::steady_clock::now() - start;
}

class SleepingPoliciesTest : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  SleepingPoliciesTest()
  : ControllerManagerFixture(
      ros2_control_test_assets::minimal_robot_urdf, "",
      {rclcpp::Parameter("update_rate", kUpdateRateHz)})
  {
  }

protected:
  void SetUp() override
  {
    ControllerManagerFixture::SetUp();
    pass_robot_description_to_cm_and_rm();
    executor_->add_node(cm_);
    executor_spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_spin_future_.valid())
    {
      executor_spin_future_.wait();
    }
    executor_->remove_node(cm_->get_node_base_interface());
    ControllerManagerFixture::TearDown();
  }

  std::future<void> executor_spin_future_;
};

}  // namespace

TEST_F(SleepingPoliciesTest, sleep_for_blocking_read_write_does_not_sleep_when_cycle_is_long_enough)
{
  const controller_manager::ControlLoopTimingConfig config{
    .expect_blocking_read_write = true, .minimum_cycle_time = kMinimumCycleTimeS};
  controller_manager::ControlLoopState state;
  state.previous_time = rclcpp::Time(1, 0);
  state.cycle_end_time = state.previous_time + rclcpp::Duration::from_seconds(kMinimumCycleTimeS);

  const auto elapsed =
    measure_execution([&]() { sleep_for_blocking_read_write(cm_, config, state); });

  EXPECT_LT(elapsed, kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_blocking_read_write_sleeps_when_cycle_is_too_short)
{
  const controller_manager::ControlLoopTimingConfig config{
    .expect_blocking_read_write = true, .minimum_cycle_time = kMinimumCycleTimeS};
  controller_manager::ControlLoopState state;
  state.previous_time = rclcpp::Time(1, 0);
  state.cycle_end_time =
    state.previous_time + rclcpp::Duration::from_seconds(kMinimumCycleTimeS / 2);

  const auto elapsed =
    measure_execution([&]() { sleep_for_blocking_read_write(cm_, config, state); });

  EXPECT_GE(
    elapsed, std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::duration<double>(kMinimumCycleTimeS)) -
               kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_periodic_cycle_sleeps_until_next_iteration)
{
  controller_manager::ControlLoopTimingConfig config{.manage_overruns = true};
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRateHz);
  state.next_iteration_time = std::chrono::steady_clock::now();
  auto expected_next_iteration_time = state.next_iteration_time + state.period;

  const auto elapsed = measure_execution([&]() { sleep_for_periodic_cycle(cm_, config, state); });

  EXPECT_GE(elapsed, state.period - kTimingTolerance);
  EXPECT_LE(elapsed, state.period + kTimingTolerance);
  EXPECT_GE(state.next_iteration_time, expected_next_iteration_time - kTimingTolerance);
  EXPECT_LE(state.next_iteration_time, expected_next_iteration_time + kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_periodic_cycle_recovers_from_overrun)
{
  controller_manager::ControlLoopTimingConfig config{.manage_overruns = true};
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRateHz);
  // "next iteration time" is more or less the expected time at which THIS control cycle should
  // have been started. Since we overran, this is in the past
  state.next_iteration_time = std::chrono::steady_clock::now() - 200ms;

  const auto elapsed = measure_execution([&]() { sleep_for_periodic_cycle(cm_, config, state); });

  EXPECT_LT(elapsed, 100ms);
  EXPECT_GE(state.next_iteration_time, std::chrono::steady_clock::now() - kTimingTolerance);
}

TEST_F(
  SleepingPoliciesTest, sleep_for_periodic_cycle_does_not_recover_when_overrun_management_disabled)
{
  controller_manager::ControlLoopTimingConfig config{.manage_overruns = false};
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRateHz);
  // "next iteration time" is more or less the expected time at which THIS control cycle should
  // have been started. Since we overran, this is in the past
  state.next_iteration_time = std::chrono::steady_clock::now() - 200ms;

  auto expected_next_iteration_time = state.next_iteration_time + state.period;

  const auto elapsed = measure_execution([&]() { sleep_for_periodic_cycle(cm_, config, state); });

  EXPECT_LT(elapsed, kTimingTolerance);  // Since the next iteration time is in the past, we should
                                         // not sleep at all

  EXPECT_LE(state.next_iteration_time, expected_next_iteration_time + kTimingTolerance);
  EXPECT_GE(state.next_iteration_time, expected_next_iteration_time - kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_sim_time_returns_true_and_sleeps_for_period)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::milliseconds(20);
  state.previous_time = cm_->get_clock()->now();

  bool sleep_result = false;
  const auto elapsed = measure_execution([&]() { sleep_result = sleep_for_sim_time(cm_, state); });

  EXPECT_TRUE(sleep_result);
  EXPECT_LE(elapsed, state.period + kTimingTolerance);
  EXPECT_GE(elapsed, state.period - kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_sim_time_returns_false_when_clock_types_mismatch)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::milliseconds(20);
  state.previous_time = cm_->get_trigger_clock()->now();

  EXPECT_FALSE(sleep_for_sim_time(cm_, state));
}


class SimTimeSleepingPoliciesTest
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  SimTimeSleepingPoliciesTest()
  : ControllerManagerFixture(
      ros2_control_test_assets::minimal_robot_urdf, "",
      {rclcpp::Parameter("update_rate", kUpdateRateHz),
       rclcpp::Parameter("use_sim_time", true)})
  {
  }

protected:
  void SetUp() override
  {
    ControllerManagerFixture::SetUp();
    pass_robot_description_to_cm_and_rm();
    clock_publisher_node_ = std::make_shared<rclcpp::Node>("clock_publisher");
    clock_publisher_ =
      clock_publisher_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    executor_->add_node(cm_);
    executor_->add_node(clock_publisher_node_);
    executor_spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
    ASSERT_TRUE(wait_for_clock(0ns));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_spin_future_.valid())
    {
      executor_spin_future_.wait();
    }
    executor_->remove_node(cm_->get_node_base_interface());
    executor_->remove_node(clock_publisher_node_->get_node_base_interface());
    ControllerManagerFixture::TearDown();
  }

  void publish_clock(std::chrono::nanoseconds time)
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock.sec = static_cast<int32_t>(time.count() / 1'000'000'000);
    msg.clock.nanosec = static_cast<uint32_t>(time.count() % 1'000'000'000);
    clock_publisher_->publish(msg);
  }

  bool wait_for_clock(std::chrono::nanoseconds expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline)
    {
      publish_clock(expected);
      if (cm_->get_clock()->now().nanoseconds() == expected.count())
      {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  rclcpp::Node::SharedPtr clock_publisher_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  std::future<void> executor_spin_future_;
};

TEST_F(SimTimeSleepingPoliciesTest, backward_clock_jump_interrupts_pending_sim_time_sleep)
{
  ASSERT_TRUE(wait_for_clock(10s));
  ASSERT_TRUE(cm_->get_clock()->started());

  controller_manager::ControlLoopState state;
  state.period = 10s;
  state.previous_time = cm_->get_trigger_clock()->now();

  auto jump_handler = register_sim_time_jump_handler(cm_);

  auto sleep_future = std::async(
    std::launch::async, [&]() { return sleep_for_sim_time(cm_, state); });

  // Let the worker enter sleep_until() before rewinding simulated time.
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(wait_for_clock(11s));

  const auto reset_start = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait_for_clock(1s));

  ASSERT_EQ(sleep_future.wait_for(1s), std::future_status::ready)
    << "Backward /clock jump did not interrupt the pending simulated-time sleep";
  EXPECT_TRUE(sleep_future.get());
  EXPECT_LT(std::chrono::steady_clock::now() - reset_start, 1s);
  EXPECT_EQ(cm_->get_trigger_clock()->now().nanoseconds(), 1'000'000'000);
}
