// Copyright 2025 Aarav Gupta
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

#include "test_magnetic_field_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "sensor_msgs/msg/magnetic_field.hpp"

void MagneticFieldSensorTest::TearDown() { magnetic_field_sensor_.reset(nullptr); }

TEST_F(MagneticFieldSensorTest, validate_all)
{
  // Create the magnetic field sensor
  magnetic_field_sensor_ = std::make_unique<TestableMagneticFieldSensor>(sensor_name_);

  // validate the component name
  ASSERT_EQ(magnetic_field_sensor_->name_, sensor_name_);

  // Validate unavailable data is represented as NaN until a sample is available.
  ASSERT_TRUE(
    std::all_of(
      magnetic_field_sensor_->data_.begin(), magnetic_field_sensor_->data_.end(),
      [](const double value) { return std::isnan(value); }));

  // Validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(magnetic_field_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(magnetic_field_sensor_->state_interfaces_.capacity(), size_);

  // Validate the default interface_names
  ASSERT_TRUE(
    std::equal(
      magnetic_field_sensor_->interface_names_.begin(),
      magnetic_field_sensor_->interface_names_.end(), full_interface_names_.begin(),
      full_interface_names_.end()));

  // Get the interface names
  std::vector<std::string> interface_names = magnetic_field_sensor_->get_state_interface_names();

  // Assign values
  auto magnetic_field_x = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[0]);
  std::ignore = magnetic_field_x->set_value(magnetic_field_values_[0]);
  auto magnetic_field_y = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[1]);
  std::ignore = magnetic_field_y->set_value(magnetic_field_values_[1]);
  auto magnetic_field_z = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[2]);
  std::ignore = magnetic_field_z->set_value(magnetic_field_values_[2]);

  // Create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(10);

  // Insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(magnetic_field_y);
  temp_state_interfaces.emplace_back(magnetic_field_z);
  temp_state_interfaces.emplace_back(magnetic_field_x);

  // Now call the function to make them in order like interface_names
  magnetic_field_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // Validate the count of state_interfaces
  ASSERT_EQ(magnetic_field_sensor_->state_interfaces_.size(), size_);

  sensor_msgs::msg::MagneticField temp_message;

  // Simulate all state reads being unavailable. The sensor must not report the
  // zero-initialized or stale cache as a current measurement.
  {
    std::unique_lock<std::shared_mutex> x_lock(magnetic_field_x->get_mutex());
    std::unique_lock<std::shared_mutex> y_lock(magnetic_field_y->get_mutex());
    std::unique_lock<std::shared_mutex> z_lock(magnetic_field_z->get_mutex());

    ASSERT_TRUE(magnetic_field_sensor_->get_values_as_message(temp_message));
    ASSERT_TRUE(std::isnan(temp_message.magnetic_field.x));
    ASSERT_TRUE(std::isnan(temp_message.magnetic_field.y));
    ASSERT_TRUE(std::isnan(temp_message.magnetic_field.z));
  }

  // Validate a partial read. An unavailable axis must not be mixed with a
  // previous sample while the other axes report the current sample.
  std::ignore = magnetic_field_x->set_value(10.0);
  std::ignore = magnetic_field_y->set_value(20.0);
  std::ignore = magnetic_field_z->set_value(30.0);

  {
    std::unique_lock<std::shared_mutex> y_lock(magnetic_field_y->get_mutex());

    ASSERT_TRUE(magnetic_field_sensor_->get_values_as_message(temp_message));
    ASSERT_DOUBLE_EQ(temp_message.magnetic_field.x, 10.0);
    ASSERT_TRUE(std::isnan(temp_message.magnetic_field.y));
    ASSERT_DOUBLE_EQ(temp_message.magnetic_field.z, 30.0);
  }

  // Validate a complete read after the unavailable sample.
  ASSERT_TRUE(magnetic_field_sensor_->get_values_as_message(temp_message));
  ASSERT_DOUBLE_EQ(temp_message.magnetic_field.x, 10.0);
  ASSERT_DOUBLE_EQ(temp_message.magnetic_field.y, 20.0);
  ASSERT_DOUBLE_EQ(temp_message.magnetic_field.z, 30.0);

  // release the state_interfaces
  magnetic_field_sensor_->release_interfaces();

  // validate the count of state_interfaces
  ASSERT_EQ(magnetic_field_sensor_->state_interfaces_.size(), 0u);
}
