///
/// @file
///
#include <logging/logging.h>
#include <motion_planning/lane_evaluator/lane_evaluator.h>
#include <chrono>
#include <sstream>

namespace motion_planning
{
LaneEvaluator::LaneEvaluator(std::shared_ptr<IDataSource>& data_source) : data_source_(data_source) {}

LaneInformation::GlobalLaneId LaneEvaluator::GetGlobalLaneId(const FrenetCoordinates& coords) const
{
    if (coords.d > 0 && coords.d < 4)  // left lane (near to double solid lane marking)
    {
        return LaneInformation::GlobalLaneId::kLeft;
    }
    else if (coords.d > 4 && coords.d < 8)  // center lane
    {
        return LaneInformation::GlobalLaneId::kCenter;
    }
    else if (coords.d > 8 && coords.d < 12)  // right lane (near to the edge of the road)
    {
        return LaneInformation::GlobalLaneId::kRight;
    }
    else
    {
        return LaneInformation::GlobalLaneId::kInvalid;
    }
}

bool LaneEvaluator::IsObjectNear(const FrenetCoordinates& ego_position, const FrenetCoordinates& obj_position) const
{
    const auto is_near = (std::fabs(obj_position.s - ego_position.s) < gkFarDistanceThreshold.value());
    return is_near;
}

LaneInformation::LaneId LaneEvaluator::GetLocalLaneId(const LaneInformation::GlobalLaneId& global_lane_id) const
{
    const auto ego_global_lane_id = data_source_->GetGlobalLaneId();

    if (ego_global_lane_id == global_lane_id)
    {
        return LaneInformation::LaneId::kEgo;
    }
    else if (ego_global_lane_id - 1 == global_lane_id)
    {
        return LaneInformation::LaneId::kLeft;
    }
    else if (ego_global_lane_id + 1 == global_lane_id)
    {
        return LaneInformation::LaneId::kRight;
    }
    return LaneInformation::LaneId::kInvalid;
}

bool LaneEvaluator::IsDrivableLane(const LaneInformation::LaneId& lane_id) const
{
    std::stringstream log_stream;

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
        const auto obj_global_lane_id = GetGlobalLaneId(obj_position);
        const auto obj_lane_id = GetLocalLaneId(obj_global_lane_id);
        const auto obj_position_predicted =
            FrenetCoordinates{obj_position.s + (previous_path_size * 0.02 * obj_velocity.value()), obj_position.d};

        // Object is in query lane
        if (obj_lane_id == LaneInformation::LaneId::kEgo)
        {
            car_in_front |= (ego_position_predicted.s > obj_position_predicted.s) &&
                            IsObjectNear(ego_position_predicted, obj_position_predicted);
        }
        else if (obj_lane_id == LaneInformation::LaneId::kLeft)
        {
            car_to_left |= (ego_position_predicted.s - gkFarDistanceThreshold.value()) < obj_position_predicted.s &&
                           IsObjectNear(ego_position_predicted, obj_position_predicted);
        }
        else if (obj_lane_id == LaneInformation::LaneId::kRight)
        {
            car_to_right |= (ego_position_predicted.s - gkFarDistanceThreshold.value()) < obj_position_predicted.s &&
                            IsObjectNear(ego_position_predicted, obj_position_predicted);
        }
        else
        {
            /* do nothing */
        }
    }
    const auto is_ego_in_valid_lane = (ego_global_lane_id != LaneInformation::GlobalLaneId::kInvalid);
    bool is_drivable = false;
    switch (lane_id)
    {
        case LaneInformation::LaneId::kEgo:
            is_drivable = IsValidLane(LaneInformation::LaneId::kEgo) && is_ego_in_valid_lane && !car_in_front;
            break;
        case LaneInformation::LaneId::kLeft:
            is_drivable = IsValidLane(LaneInformation::LaneId::kLeft) && is_ego_in_valid_lane && !car_to_left;
            break;
        case LaneInformation::LaneId::kRight:
            is_drivable = IsValidLane(LaneInformation::LaneId::kRight) && is_ego_in_valid_lane && !car_to_right;
            break;
        case LaneInformation::LaneId::kInvalid:
        default:
            is_drivable = false;
            break;
    }

    log_stream << "Is {" << lane_id << "} drivable? " << std::boolalpha << is_drivable << std::endl;
    LOG_DEBUG("LaneEvaluator", log_stream.str());
    return is_drivable;
}

bool LaneEvaluator::IsValidLane(const LaneInformation::LaneId& lane_id) const
{
    const auto ego_global_lane_id = data_source_->GetGlobalLaneId();

    switch (lane_id)
    {
        case LaneInformation::LaneId::kEgo:
            return true;
        case LaneInformation::LaneId::kLeft:
            return (ego_global_lane_id - 1) != LaneInformation::GlobalLaneId::kInvalid;
        case LaneInformation::LaneId::kRight:
            return (ego_global_lane_id + 1) != LaneInformation::GlobalLaneId::kInvalid;
        case LaneInformation::LaneId::kInvalid:
        default:
            return false;
    }
}

}  // namespace motion_planning
