import types
from typing import Any

class LockedPlanningSceneContextManagerRO:
    def __init__(self, *args, **kwargs) -> None: ...
    def __enter__(self) -> Any: ...
    def __exit__(self, type, value, traceback) -> Any: ...

class LockedPlanningSceneContextManagerRW:
    def __init__(self, *args, **kwargs) -> None: ...
    def __enter__(self) -> Any: ...
    def __exit__(self, type, value, traceback) -> Any: ...

class MoveItPy:
    def __init__(self, *args, **kwargs) -> None: ...
    def execute(self, *args, **kwargs) -> Any: ...
    def get_planning_component(self, *args, **kwargs) -> Any: ...
    def get_planning_scene_monitor(self, *args, **kwargs) -> Any: ...
    def get_robot_model(self, *args, **kwargs) -> Any: ...
    def get_trajactory_execution_manager(self, *args, **kwargs) -> Any: ...
    def shutdown(self, *args, **kwargs) -> Any: ...

class MultiPipelinePlanRequestParameters:
    def __init__(self, *args, **kwargs) -> None: ...
    @property
    def multi_plan_request_parameters(self) -> Any: ...

class PlanRequestParameters:
    max_acceleration_scaling_factor: Any
    max_velocity_scaling_factor: Any
    planner_id: Any
    planning_attempts: Any
    planning_pipeline: Any
    planning_time: Any
    def __init__(self, *args, **kwargs) -> None: ...

class PlanningComponent:
    def __init__(self, *args, **kwargs) -> None: ...
    def get_named_target_state_values(self, *args, **kwargs) -> Any: ...
    def get_start_state(self, *args, **kwargs) -> Any: ...
    def plan(self, *args, **kwargs) -> Any: ...
    def set_goal_state(self, *args, **kwargs) -> Any: ...
    def set_path_constraints(self, *args, **kwargs) -> Any: ...
    def set_start_state(self, *args, **kwargs) -> Any: ...
    def set_start_state_to_current_state(self, *args, **kwargs) -> Any: ...
    def set_workspace(self, *args, **kwargs) -> Any: ...
    def unset_workspace(self, *args, **kwargs) -> Any: ...
    @property
    def named_target_states(self) -> Any: ...
    @property
    def planning_group_name(self) -> Any: ...

class PlanningSceneMonitor:
    def __init__(self, *args, **kwargs) -> None: ...
    def clear_octomap(self, *args, **kwargs) -> Any: ...
    def process_attached_collision_object(self, *args, **kwargs) -> Any: ...
    def process_collision_object(self, *args, **kwargs) -> Any: ...
    def read_only(self, *args, **kwargs) -> Any: ...
    def read_write(self, *args, **kwargs) -> Any: ...
    def request_planning_scene_state(self, *args, **kwargs) -> Any: ...
    def start_scene_monitor(self, *args, **kwargs) -> Any: ...
    def start_state_monitor(self, *args, **kwargs) -> Any: ...
    def stop_scene_monitor(self, *args, **kwargs) -> Any: ...
    def stop_state_monitor(self, *args, **kwargs) -> Any: ...
    def update_frame_transforms(self, *args, **kwargs) -> Any: ...
    def wait_for_current_robot_state(self, *args, **kwargs) -> Any: ...
    @property
    def name(self) -> Any: ...

class TrajectoryExecutionManager:
    def __init__(self, *args, **kwargs) -> None: ...
    def are_controllers_active(self, *args, **kwargs) -> Any: ...
    def clear(self, *args, **kwargs) -> Any: ...
    def enable_execution_duration_monitoring(self, *args, **kwargs) -> Any: ...
    def ensure_active_controller(self, *args, **kwargs) -> Any: ...
    def ensure_active_controllers(self, *args, **kwargs) -> Any: ...
    def ensure_active_controllers_for_group(self, *args, **kwargs) -> Any: ...
    def ensure_active_controllers_for_joints(self, *args, **kwargs) -> Any: ...
    def execute(self, *args, **kwargs) -> Any: ...
    def execute_and_wait(self, *args, **kwargs) -> Any: ...
    def get_last_execution_status(self, *args, **kwargs) -> Any: ...
    def is_controller_active(self, *args, **kwargs) -> Any: ...
    def is_managing_controllers(self, *args, **kwargs) -> Any: ...
    def process_event(self, *args, **kwargs) -> Any: ...
    def push(self, *args, **kwargs) -> Any: ...
    def set_allowed_execution_duration_scaling(self, *args, **kwargs) -> Any: ...
    def set_allowed_goal_duration_margin(self, *args, **kwargs) -> Any: ...
    def set_allowed_start_tolerance(self, *args, **kwargs) -> Any: ...
    def set_execution_velocity_scaling(self, *args, **kwargs) -> Any: ...
    def set_wait_for_trajectory_completion(self, *args, **kwargs) -> Any: ...
    def stop_execution(self, *args, **kwargs) -> Any: ...
    def wait_for_execution(self, *args, **kwargs) -> Any: ...
