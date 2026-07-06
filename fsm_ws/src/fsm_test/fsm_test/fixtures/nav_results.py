from wall_destacking_strategy.data import NavResult, PoseData


def success_result() -> NavResult:
    return NavResult(success=True, actual_base_pose=PoseData.zero(frame_id="map"))
