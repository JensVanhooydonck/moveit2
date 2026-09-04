/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter David Fagan */

#include "planning_scene_monitor.hpp"

namespace moveit_py
{
namespace bind_planning_scene_monitor
{

bool processCollisionObject(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                            moveit_msgs::msg::CollisionObject& collision_object_msg,
                            std::optional<moveit_msgs::msg::ObjectColor> color_msg)
{
  moveit_msgs::msg::CollisionObject::ConstSharedPtr const_ptr =
      std::make_shared<const moveit_msgs::msg::CollisionObject>(collision_object_msg);
  if (color_msg)
  {
    return planning_scene_monitor->processCollisionObjectMsg(const_ptr, *std::move(color_msg));
  }
  return planning_scene_monitor->processCollisionObjectMsg(const_ptr);
}

bool processAttachedCollisionObjectMsg(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                       moveit_msgs::msg::AttachedCollisionObject& attached_collision_object_msg)
{
  moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr const_ptr =
      std::make_shared<const moveit_msgs::msg::AttachedCollisionObject>(attached_collision_object_msg);
  return planning_scene_monitor->processAttachedCollisionObjectMsg(const_ptr);
}

LockedPlanningSceneContextManagerRO
readOnly(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  return LockedPlanningSceneContextManagerRO(planning_scene_monitor);
};

LockedPlanningSceneContextManagerRW
readWrite(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  return LockedPlanningSceneContextManagerRW(planning_scene_monitor);
};

const planning_scene::PlanningSceneConstPtr& LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoEnter() const
{
  return static_cast<const planning_scene_monitor::PlanningSceneMonitor*>(planning_scene_monitor_.get())
      ->getPlanningScene();
}

const planning_scene::PlanningScenePtr& LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwEnter()
{
  return planning_scene_monitor_->getPlanningScene();
}

void LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoExit(const py::object& /*type*/,
                                                                    const py::object& /*value*/,
                                                                    const py::object& /*traceback*/)
{
  ls_ro_.reset();
}

void LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwExit(const py::object& /*type*/,
                                                                    const py::object& /*value*/,
                                                                    const py::object& /*traceback*/)
{
  ls_rw_.reset();
}

void initPlanningSceneMonitor(py::module& m)
{
  py::class_<planning_scene_monitor::PlanningSceneMonitor, planning_scene_monitor::PlanningSceneMonitorPtr>(
      m, "PlanningSceneMonitor", R"(
      Maintains the internal state of the planning scene.
      )")

      .def_property("name", &planning_scene_monitor::PlanningSceneMonitor::getName, nullptr,
                    R"(
                    str: The name of this planning scene monitor.
                    )")

      .def("update_frame_transforms", &planning_scene_monitor::PlanningSceneMonitor::updateFrameTransforms,
           R"(
           Update the transforms for the frames that are not part of the kinematic model using tf.

           Examples of these frames are the "map" and "odom_combined" transforms. This function is automatically called
           when data that uses transforms is received.
           However, this function should also be called before starting a planning request, for example.
           )")

      .def("start_scene_monitor", &planning_scene_monitor::PlanningSceneMonitor::startSceneMonitor,
           R"(
           Starts the scene monitor.
           )")

      .def("stop_scene_monitor", &planning_scene_monitor::PlanningSceneMonitor::stopSceneMonitor,
           R"(
           Stops the scene monitor.
           )")

      .def("start_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::startStateMonitor,
           R"(
	   Starts the state monitor.
	   )")

      .def("stop_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::stopStateMonitor,
           R"(
	       Stops the state monitor.
	   )")
      .def("request_planning_scene_state", &planning_scene_monitor::PlanningSceneMonitor::requestPlanningSceneState,
           py::arg("service_name"),
           R"(
	       Request the planning scene.

            Args:
               service_name (str): The name of the service to call.
	   )")

      .def("wait_for_current_robot_state", &planning_scene_monitor::PlanningSceneMonitor::waitForCurrentRobotState,
           py::call_guard<py::gil_scoped_release>(),
           R"(
	   Waits for the current robot state to be received.
	   )")

      .def("clear_octomap", &planning_scene_monitor::PlanningSceneMonitor::clearOctomap,
           R"(
           Clears the octomap.
           )")
      .def("process_collision_object", &moveit_py::bind_planning_scene_monitor::processCollisionObject,
           py::arg("collision_object_msg"), py::arg("color_msg") = nullptr,
           R"(
           Apply a collision object to the planning scene.

	      Args:
               collision_object_msg (moveit_msgs.msg.CollisionObject): The collision object to apply to the planning scene.
           )")
      .def("process_attached_collision_object",
           &moveit_py::bind_planning_scene_monitor::processAttachedCollisionObjectMsg,
           py::arg("attached_collision_object_msg"),
           R"(
           Apply an attached collision object msg to the planning scene.

	      Args:
               attached_collision_object_msg (moveit_msgs.msg.AttachedCollisionObject): The attached collision object to apply to the planning scene.
           )")

      .def("new_planning_scene_message", &planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneMessage,
           py::arg("scene"),
           R"(
           Called to update the planning scene with a new message.

	      Args:
               scene (moveit_msgs.msg.PlanningScene): The new planning scene message.
           )")

      // These two BLOCK: the returned context manager's constructor builds a
      // LockedPlanningScene{RO,RW}, which acquires the scene mutex. Without a call_guard
      // that wait happens while holding the GIL, so a Python thread waiting for the scene
      // monitor to finish an update freezes EVERY thread in the process — the scene is
      // written hardest during motion (/joint_states floods in), which is exactly when
      // other threads are reading it. Measured: ~0.7-1.4s process-wide stalls, which
      // inflate job durations and make trajectory triggers fire late.
      //
      // Releasing here matches what the other blocking bindings already do
      // (wait_for_execution, execute_and_wait, wait_for_current_robot_state, plan). It
      // also removes a deadlock: a thread blocked on the mutex while holding the GIL
      // cannot be freed by a mutex holder that needs the GIL to make progress.
      //
      // The guard is destroyed before the return value is converted, so the GIL is held
      // again by the time pybind11 casts the context manager back to Python.
      // __enter__/__exit__ need no guard: __enter__ only returns the already-locked
      // scene, and __exit__ only drops the pointer.
      .def("read_only", &moveit_py::bind_planning_scene_monitor::readOnly,
           py::call_guard<py::gil_scoped_release>(),
           R"(
           Returns a read-only context manager for the planning scene.
           )")

      .def("read_write", &moveit_py::bind_planning_scene_monitor::readWrite,
           py::call_guard<py::gil_scoped_release>(),
           R"(
           Returns a read-write context manager for the planning scene.
           )");
}

void initContextManagers(py::module& m)
{
  // In Python we lock the planning scene using a with statement as this allows us to have control over resources.
  // To this end each of the below manager classes binds special methods __enter__ and __exit__.
  // LockedPlanningSceneContextManagerRO
  py::class_<moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO>(
      m, "LockedPlanningSceneContextManagerRO", R"(
      A context manager that locks the planning scene for reading.
      )")

      .def("__enter__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoEnter,
           R"(
           Special method that is used with the with statement, provides access to a locked plannning scene instance.
           Returns:
               :py:class:`moveit_py.core.PlanningScene`: The locked planning scene.
        )")
      .def("__exit__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO::lockedPlanningSceneRoExit,
           R"(
           Special method that is used with the with statement, releases the lock on the planning scene.
           )");

  // LockedPlanningSceneContextManagerRW
  py::class_<moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW>(
      m, "LockedPlanningSceneContextManagerRW", R"(
      A context manager that locks the planning scene for reading and writing.
      )")

      .def("__enter__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwEnter,
           py::return_value_policy::take_ownership,
           R"(
           Special method that is used with the with statement, provides access to a locked plannning scene instance.
           Returns:
               :py:class:`moveit_py.core.PlanningScene`: The locked planning scene.
           )")

      .def("__exit__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW::lockedPlanningSceneRwExit,
           R"(
           Special method that is used with the with statement, releases the lock on the planning scene.
           )");
}
}  // namespace bind_planning_scene_monitor
}  // namespace moveit_py
