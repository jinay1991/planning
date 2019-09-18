///
/// @file
///

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ostream>
#include <sstream>

#include <motion_planning/i_data_source.h>
#include <motion_planning/roadmodel_data_source.h>

#define private public
#include <motion_planning/lane_evaluator/lane_evaluator.h>

using namespace motion_planning;

namespace
{
class ObjectFusionBuilder
{
  public:
    ObjectFusionBuilder() {}
    ObjectFusionBuilder& WithIndex(const std::int32_t idx)
    {
        object_fusion_.idx = idx;
        return *this;
    }
    ObjectFusionBuilder& WithGlobalCoordinates(const GlobalCoordinates& coords)
    {
        object_fusion_.global_coords = coords;
        return *this;
    }
    ObjectFusionBuilder& WithFrenetCoordinates(const FrenetCoordinates& coords)
    {
        object_fusion_.frenet_coords = coords;
        return *this;
    }

    ObjectFusionBuilder& WithVelocity(const units::velocity::meters_per_second_t& velocity)
    {
        object_fusion_.velocity = velocity;
        return *this;
    }

    ObjectFusion Build() const { return object_fusion_; }

  private:
    ObjectFusion object_fusion_{};
};
class LaneEvaluatorSpec
{
  public:
    LaneEvaluatorSpec()
    {
        data_source_ = std::make_shared<RoadModelDataSource>();

        unit_ = std::make_unique<LaneEvaluator>(data_source_);
    }

    void SetEgoVehicleGlobalLaneId(const LaneInformation::GlobalLaneId& global_lane_id)
    {
        switch (global_lane_id)
        {
            case LaneInformation::GlobalLaneId::kCenter:
                data_source_->SetPreviousPathEnd(FrenetCoordinates{24, 6});
                break;
            case LaneInformation::GlobalLaneId::kLeft:
                data_source_->SetPreviousPathEnd(FrenetCoordinates{24, 2});
                break;
            case LaneInformation::GlobalLaneId::kRight:
                data_source_->SetPreviousPathEnd(FrenetCoordinates{24, 10});
                break;
            case LaneInformation::GlobalLaneId::kInvalid:
            default:
                data_source_->SetPreviousPathEnd(FrenetCoordinates{24, 14});
                break;
        }
        auto vehicle_dynamics = VehicleDynamics{};
        vehicle_dynamics.global_lane_id = global_lane_id;
        vehicle_dynamics.velocity = units::velocity::meters_per_second_t{17.0};
        data_source_->SetVehicleDynamics(vehicle_dynamics);
        auto previous_path_global = PreviousPathGlobal{};
        previous_path_global.resize(50);
        data_source_->SetPreviousPath(previous_path_global);
    }

    void SetObjectVehicleGlobalLaneId(const LaneInformation::GlobalLaneId& global_lane_id)
    {
        auto object_fusion = ObjectFusion{};
        switch (global_lane_id)
        {
            case LaneInformation::GlobalLaneId::kCenter:
                object_fusion = ObjectFusionBuilder()
                                    .WithFrenetCoordinates(FrenetCoordinates{25, 6})
                                    .WithVelocity(units::velocity::meters_per_second_t{12.0})
                                    .Build();
                break;
            case LaneInformation::GlobalLaneId::kLeft:
                object_fusion = ObjectFusionBuilder()
                                    .WithFrenetCoordinates(FrenetCoordinates{25, 2})
                                    .WithVelocity(units::velocity::meters_per_second_t{12.0})
                                    .Build();
                break;
            case LaneInformation::GlobalLaneId::kRight:
                object_fusion = ObjectFusionBuilder()
                                    .WithFrenetCoordinates(FrenetCoordinates{25, 10})
                                    .WithVelocity(units::velocity::meters_per_second_t{12.0})
                                    .Build();
                break;
            case LaneInformation::GlobalLaneId::kInvalid:
            default:
                object_fusion = ObjectFusionBuilder()
                                    .WithFrenetCoordinates(FrenetCoordinates{25, 14})
                                    .WithVelocity(units::velocity::meters_per_second_t{12.0})
                                    .Build();
                break;
        }
        data_source_->SetSensorFusion(SensorFusion{std::vector<ObjectFusion>{object_fusion}});
    }

  protected:
    std::unique_ptr<LaneEvaluator> unit_;
    std::shared_ptr<IDataSource> data_source_;
};

struct LocalLaneConversionTestParam
{
    LaneInformation::GlobalLaneId ego_lane;
    LaneInformation::GlobalLaneId obj_lane;

    LaneInformation::LaneId expected_lane;
};
class LaneEvaluatorSpecConversionFixture : public LaneEvaluatorSpec,
                                           public ::testing::TestWithParam<LocalLaneConversionTestParam>
{
};
struct GlobalLaneConversionTestParam
{
    FrenetCoordinates frenet_coords;
    LaneInformation::GlobalLaneId expected_lane;
};
class LaneEvaluatorSpecGlobalLaneConversionFixture : public LaneEvaluatorSpec,
                                                     public ::testing::TestWithParam<GlobalLaneConversionTestParam>
{
};
struct IsObjectNearTestParam
{
    FrenetCoordinates ego_coords;
    FrenetCoordinates obj_coords;

    bool expected_value;
};
class LaneEvalautorSpecIsObjectNearFixture : public LaneEvaluatorSpec,
                                             public ::testing::TestWithParam<IsObjectNearTestParam>
{
};
struct IsDrivableLaneTestParam
{
    LaneInformation::GlobalLaneId ego_lane;
    LaneInformation::GlobalLaneId obj_lane;

    bool expected_value;
};
inline std::ostream& operator<<(std::ostream& out, const IsDrivableLaneTestParam& param)
{
    out << "{ego: " << param.ego_lane << ", obj: " << param.obj_lane << ", is_drivable: " << std::boolalpha
        << param.expected_value << std::endl;
    return out;
}
class LaneEvaluatorSpecFixture : public LaneEvaluatorSpec, public ::testing::TestWithParam<IsDrivableLaneTestParam>
{
};

TEST_P(LaneEvaluatorSpecConversionFixture, GivenTypicalGlobalLaneId_WhenGetLocalLaneId_ThenReturnLocalLaneId)
{
    // Prepare
    const auto params = GetParam();
    SetEgoVehicleGlobalLaneId(params.ego_lane);

    // Run
    const auto actual = unit_->GetLocalLaneId(params.obj_lane);

    // Assert
    EXPECT_EQ(actual, params.expected_lane);
}
INSTANTIATE_TEST_CASE_P(
    LocalLaneConversion, LaneEvaluatorSpecConversionFixture,
    ::testing::Values(
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kCenter,
                                     LaneInformation::LaneId::kEgo},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kLeft,
                                     LaneInformation::LaneId::kLeft},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kRight,
                                     LaneInformation::LaneId::kRight},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kCenter,
                                     LaneInformation::LaneId::kRight},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kLeft,
                                     LaneInformation::LaneId::kEgo},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kRight,
                                     LaneInformation::LaneId::kInvalid},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kCenter,
                                     LaneInformation::LaneId::kLeft},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kLeft,
                                     LaneInformation::LaneId::kInvalid},
        LocalLaneConversionTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kRight,
                                     LaneInformation::LaneId::kEgo}));

TEST_P(LaneEvaluatorSpecGlobalLaneConversionFixture,
       GivenTypicalFrenetCoordinates_WhenGetGlobalLaneId_ThenReturnGlobalLaneId)
{
    const auto params = GetParam();

    const auto actual = unit_->GetGlobalLaneId(params.frenet_coords);

    EXPECT_EQ(actual, params.expected_lane);
}
INSTANTIATE_TEST_CASE_P(
    GlobalLaneConversion, LaneEvaluatorSpecGlobalLaneConversionFixture,
    ::testing::Values(GlobalLaneConversionTestParam{FrenetCoordinates{0, 0}, LaneInformation::GlobalLaneId::kInvalid},
                      GlobalLaneConversionTestParam{FrenetCoordinates{0, 2}, LaneInformation::GlobalLaneId::kLeft},
                      GlobalLaneConversionTestParam{FrenetCoordinates{0, 4}, LaneInformation::GlobalLaneId::kInvalid},
                      GlobalLaneConversionTestParam{FrenetCoordinates{0, 6}, LaneInformation::GlobalLaneId::kCenter},
                      GlobalLaneConversionTestParam{FrenetCoordinates{0, 8}, LaneInformation::GlobalLaneId::kInvalid},
                      GlobalLaneConversionTestParam{FrenetCoordinates{0, 10}, LaneInformation::GlobalLaneId::kRight},
                      GlobalLaneConversionTestParam{FrenetCoordinates{0, 12},
                                                    LaneInformation::GlobalLaneId::kInvalid}));

TEST_P(LaneEvalautorSpecIsObjectNearFixture, GivenTypicalVehiclePositions_WhenEvaluatedIsObjectNear_ThenReturnBoolean)
{
}
INSTANTIATE_TEST_CASE_P(
    IsObjectNearValidation, LaneEvalautorSpecIsObjectNearFixture,
    ::testing::Values(
        IsObjectNearTestParam{FrenetCoordinates{0, 0}, FrenetCoordinates{0, 0}, true},
        IsObjectNearTestParam{FrenetCoordinates{gkFarDistanceThreshold.value() - 15, 0}, FrenetCoordinates{0, 0}, true},
        IsObjectNearTestParam{FrenetCoordinates{gkFarDistanceThreshold.value(), 0}, FrenetCoordinates{0, 0}, false},
        IsObjectNearTestParam{FrenetCoordinates{gkFarDistanceThreshold.value() + 15, 0}, FrenetCoordinates{0, 0},
                              false},
        IsObjectNearTestParam{FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value() - 15, 0}, true},
        IsObjectNearTestParam{FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value(), 0}, false},
        IsObjectNearTestParam{FrenetCoordinates{0, 0}, FrenetCoordinates{gkFarDistanceThreshold.value() + 15, 0},
                              false}));

TEST_P(LaneEvaluatorSpecFixture, GivenTypicalSensorFusion_WhenCheckedIsDrivableLane_ThenReturnBasedOnCollisionAvoidance)
{
    const auto param = GetParam();
    SetEgoVehicleGlobalLaneId(param.ego_lane);
    SetObjectVehicleGlobalLaneId(param.obj_lane);

    const auto actual = unit_->IsDrivableLane(LaneInformation::LaneId::kEgo);

    EXPECT_EQ(actual, param.expected_value);
}
INSTANTIATE_TEST_CASE_P(
    IsDrivableLaneValidation, LaneEvaluatorSpecFixture,
    ::testing::Values(
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kCenter, false},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kCenter, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kCenter, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kInvalid, LaneInformation::GlobalLaneId::kCenter, false},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kLeft, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kLeft, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kLeft, false},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kInvalid, LaneInformation::GlobalLaneId::kLeft, false},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kRight, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kRight, false},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kRight, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kInvalid, LaneInformation::GlobalLaneId::kRight, false},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kCenter, LaneInformation::GlobalLaneId::kInvalid, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kRight, LaneInformation::GlobalLaneId::kInvalid, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kLeft, LaneInformation::GlobalLaneId::kInvalid, true},
        IsDrivableLaneTestParam{LaneInformation::GlobalLaneId::kInvalid, LaneInformation::GlobalLaneId::kInvalid,
                                false}));

}  // namespace
