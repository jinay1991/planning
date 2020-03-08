///
/// @file lane_evaluator.h
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "motion_planning/lane_evaluator/lane_evaluator.h"
#include "logging/logging.h"

namespace motion_planning
{
LaneEvaluator::LaneEvaluator(std::shared_ptr<IDataSource>& data_source) : data_source_(data_source) {}

bool LaneEvaluator::IsObjectNear(const FrenetCoordinates& ego_position, const FrenetCoordinates& obj_position) const
{
    const auto is_near = (std::fabs(obj_position.s - ego_position.s) < gkFarDistanceThreshold.value());
    return is_near;
}

LaneId LaneEvaluator::GetLocalLaneId(const GlobalLaneId& global_lane_id) const
{
    const auto ego_global_lane_id = data_source_->GetGlobalLaneId();

    if (ego_global_lane_id == global_lane_id)
    {
        return LaneId::kEgo;
    }
    else if (ego_global_lane_id - 1 == global_lane_id)
    {
        return LaneId::kLeft;
    }
    else if (ego_global_lane_id + 1 == global_lane_id)
    {
        return LaneId::kRight;
    }
    return LaneId::kInvalid;
}

bool LaneEvaluator::IsDrivableLane(const LaneId& lane_id) const
{
    bool car_in_front = false;
    bool car_to_left = false;
    bool car_to_right = false;
    const auto sensor_fusion = data_source_->GetSensorFusion();
    const auto previous_path_size = data_source_->GetPreviousPathInGlobalCoords().size();

    // Ego Properties
    const auto ego_velocity = data_source_->GetVehicleDynamics().velocity;
    const auto ego_position = data_source_->GetPreviousPathEnd();
    const auto ego_global_lane_id = data_source_->GetGlobalLaneId();
    const auto ego_position_predicted =
        FrenetCoordinates{ego_position.s + (previous_path_size * 0.02 * ego_velocity.value()), ego_position.d};

    for (const auto& obj : sensor_fusion.objs)
    {
        // Object Properties
        const auto obj_velocity = obj.velocity;
        const auto obj_position = obj.frenet_coords;
        const auto obj_global_lane_id = data_source_->GetGlobalLaneId(obj_position);
        const auto obj_lane_id = GetLocalLaneId(obj_global_lane_id);
        const auto obj_position_predicted =
            FrenetCoordinates{obj_position.s + (previous_path_size * 0.02 * obj_velocity.value()), obj_position.d};

        // Object is in query lane
        if (obj_lane_id == LaneId::kEgo)
        {
            car_in_front |= (ego_position_predicted.s > obj_position_predicted.s) &&
                            IsObjectNear(ego_position_predicted, obj_position_predicted);
        }
        else if (obj_lane_id == LaneId::kLeft)
        {
            car_to_left |= (ego_position_predicted.s - gkFarDistanceThreshold.value()) < obj_position_predicted.s &&
                           IsObjectNear(ego_position_predicted, obj_position_predicted);
        }
        else if (obj_lane_id == LaneId::kRight)
        {
            car_to_right |= (ego_position_predicted.s - gkFarDistanceThreshold.value()) < obj_position_predicted.s &&
                            IsObjectNear(ego_position_predicted, obj_position_predicted);
        }
        else
        {
            /* do nothing */
        }
    }
    const auto is_ego_in_valid_lane = (ego_global_lane_id != GlobalLaneId::kInvalid);
    bool is_drivable = false;
    switch (lane_id)
    {
        case LaneId::kEgo:
            is_drivable = IsValidLane(LaneId::kEgo) && is_ego_in_valid_lane && !car_in_front;
            break;
        case LaneId::kLeft:
            is_drivable = IsValidLane(LaneId::kLeft) && is_ego_in_valid_lane && !car_to_left;
            break;
        case LaneId::kRight:
            is_drivable = IsValidLane(LaneId::kRight) && is_ego_in_valid_lane && !car_to_right;
            break;
        case LaneId::kInvalid:
        default:
            is_drivable = false;
            break;
    }

    // LOG(DEBUG) << "Is {" << lane_id << "} drivable? " << std::boolalpha << is_drivable;
    return is_drivable;
}

bool LaneEvaluator::IsValidLane(const LaneId& lane_id) const
{
    const auto ego_global_lane_id = data_source_->GetGlobalLaneId();

    switch (lane_id)
    {
        case LaneId::kEgo:
            return true;
        case LaneId::kLeft:
            return (ego_global_lane_id - 1) != GlobalLaneId::kInvalid;
        case LaneId::kRight:
            return (ego_global_lane_id + 1) != GlobalLaneId::kInvalid;
        case LaneId::kInvalid:
        default:
            return false;
    }
}

}  // namespace motion_planning
