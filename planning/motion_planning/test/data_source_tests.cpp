///
/// @file
/// @brief Contains unit tests for Road Model Data Source.
/// @copyright Copyright (c) 2021. All Rights Reserved.
///
#include "planning/motion_planning/data_source.h"
#include "planning/motion_planning/test/support/builders/sensor_fusion_builder.h"
#include "support/builders/object_fusion_builder.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace planning
{
namespace
{
using namespace units::literals;

using ::testing::AllOf;
using ::testing::Contains;
using ::testing::Field;

class DataSourceFixture : public ::testing::Test
{
  protected:
    DataSource data_source_;
};

template <typename T>
class DataSourceFixtureT : public DataSourceFixture, public ::testing::WithParamInterface<T>
{
};

TEST_F(DataSourceFixture, SetVehicleDynamics_GivenTypicalVehicleDynamics_ExpectSame)
{
    // Given
    VehicleDynamics vehicle_dynamics{};
    vehicle_dynamics.velocity = 10.0_mps;
    vehicle_dynamics.yaw = 10.0_rad;

    // When
    data_source_.SetVehicleDynamics(vehicle_dynamics);

    // Then
    EXPECT_THAT(data_source_.GetVehicleDynamics(),
                AllOf(Field(&VehicleDynamics::velocity, vehicle_dynamics.velocity),
                      Field(&VehicleDynamics::yaw, vehicle_dynamics.yaw)));
}

TEST_F(DataSourceFixture, SetMapCoordinates_GivenTypicalMapCoordinates_ExpectSame)
{
    // Given
    const MapCoordinates map_coordinates{GlobalCoordinates{784.6001, 1135.571},
                                         FrenetCoordinates{0, 0, -0.02359831, -0.9997216}};
    MapCoordinatesList map_coordinates_list_{};
    map_coordinates_list_.push_back(map_coordinates);

    // When
    data_source_.SetMapCoordinates(map_coordinates_list_);

    // Then
    EXPECT_THAT(data_source_.GetMapCoordinates(),
                Contains(AllOf(Field(&MapCoordinates::global_coords,
                                     AllOf(Field(&GlobalCoordinates::x, map_coordinates.global_coords.x),
                                           Field(&GlobalCoordinates::y, map_coordinates.global_coords.y))),
                               Field(&MapCoordinates::frenet_coords,
                                     AllOf(Field(&FrenetCoordinates::s, map_coordinates.frenet_coords.s),
                                           Field(&FrenetCoordinates::d, map_coordinates.frenet_coords.d),
                                           Field(&FrenetCoordinates::dx, map_coordinates.frenet_coords.dx),
                                           Field(&FrenetCoordinates::dy, map_coordinates.frenet_coords.dy))))));
}

TEST_F(DataSourceFixture, SetPreviousPath_GivenTypicalPreviousPath_ExpectSame)
{
    // Given
    const GlobalCoordinates global_coordinate{10.0, 12.0};
    PreviousPathGlobal previous_path_global{};
    previous_path_global.push_back(global_coordinate);

    // When
    data_source_.SetPreviousPath(previous_path_global);

    // Then
    EXPECT_THAT(data_source_.GetPreviousPathInGlobalCoords(),
                Contains(AllOf(Field(&GlobalCoordinates::x, global_coordinate.x),
                               Field(&GlobalCoordinates::y, global_coordinate.y))));
}

TEST_F(DataSourceFixture, SetPreviousPathEnd_GivenTypicalPreviousPathEnd_ExpectSame)
{
    // Given
    const FrenetCoordinates frenet_coordinate{10.0, 12.0, 0.0, 0.0};

    // When
    data_source_.SetPreviousPathEnd(frenet_coordinate);

    // Then
    EXPECT_THAT(
        data_source_.GetPreviousPathEnd(),
        AllOf(Field(&FrenetCoordinates::s, frenet_coordinate.s), Field(&FrenetCoordinates::d, frenet_coordinate.d)));
}

TEST_F(DataSourceFixture, SetSensorFusion_GivenTypicalSensorFusion_ExpectSame)
{
    // Given
    const units::velocity::meters_per_second_t velocity{10.0};
    const ObjectFusion object_fusion = ObjectFusionBuilder().WithVelocity(velocity).Build();
    const SensorFusion sensor_fusion = SensorFusionBuilder().WithObjectFusion(object_fusion).Build();

    // When
    data_source_.SetSensorFusion(sensor_fusion);

    // Then
    EXPECT_THAT(data_source_.GetSensorFusion(),
                AllOf(Field(&SensorFusion::objs, Contains(Field(&ObjectFusion::velocity, velocity)))));
}

TEST_F(DataSourceFixture, SetSpeedLimit_GivenTypicalSpeedLimit_ExpectSame)
{
    // Given
    const units::velocity::kilometers_per_hour_t speed_limit{100.0};

    // When
    data_source_.SetSpeedLimit(speed_limit);

    // Then
    EXPECT_EQ(data_source_.GetSpeedLimit(), speed_limit);
}

struct TestFrenetCoordinateParam
{
    // Given
    FrenetCoordinates frenet_coordinates;

    // Then
    GlobalLaneId global_lane_id;
};

using DataSourceFixture_WithFrenetCoordinates = DataSourceFixtureT<TestFrenetCoordinateParam>;

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    DataSource,
    DataSourceFixture_WithFrenetCoordinates,
    ::testing::Values(
      //                        frenet_coordinates          , (expected) global_lane_id
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 2.0} , GlobalLaneId::kLeft      },
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 0.0} , GlobalLaneId::kInvalid   },
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 4.0} , GlobalLaneId::kInvalid   },
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 6.0} , GlobalLaneId::kCenter    },
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 8.0} , GlobalLaneId::kInvalid   },
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 10.0}, GlobalLaneId::kRight     },
      TestFrenetCoordinateParam{FrenetCoordinates{0.0, 12.0}, GlobalLaneId::kInvalid   }
));
// clang-format on

TEST_P(DataSourceFixture_WithFrenetCoordinates, GetGlobalLaneId_GivenTypicalFrenetCoordinates_ExpectGlobalLaneId)
{
    // Given
    const auto param = GetParam();

    // When
    const auto actual = data_source_.GetGlobalLaneId(param.frenet_coordinates);

    // Then
    EXPECT_EQ(actual, param.global_lane_id);
}

TEST_P(DataSourceFixture_WithFrenetCoordinates, GetGlobalLaneId_GivenTypicalVehicleDynamics_ExpectGlobalLaneId)
{
    // Given
    const auto param = GetParam();
    VehicleDynamics vehicle_dynamics{};
    vehicle_dynamics.frenet_coords = param.frenet_coordinates;

    // When
    data_source_.SetVehicleDynamics(vehicle_dynamics);

    // Then
    EXPECT_EQ(data_source_.GetGlobalLaneId(), param.global_lane_id);
}

}  // namespace
}  // namespace planning
