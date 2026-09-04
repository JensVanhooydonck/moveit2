/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/controller_manager/controller_manager.hpp>
#include <moveit/macros/class_forward.hpp>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>

namespace moveit_simple_controller_manager
{
using namespace std::chrono_literals;

/*
 * This exist solely to inject addJoint/getJoints into base non-templated class.
 */
class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandleBase(const std::string& name, const std::string& logger_name)
    : moveit_controller_manager::MoveItControllerHandle(name), logger_(rclcpp::get_logger(logger_name))
  {
  }

  virtual void addJoint(const std::string& name) = 0;
  virtual void getJoints(std::vector<std::string>& joints) = 0;
  // TODO(JafarAbdi): Revise parameter lookup
  //  virtual void configure(XmlRpc::XmlRpcValue& /* config */)
  //  {
  //  }

protected:
  const rclcpp::Logger logger_;
};

MOVEIT_CLASS_FORWARD(
    ActionBasedControllerHandleBase);  // Defines ActionBasedControllerHandleBasePtr, ConstPtr, WeakPtr... etc

/**
 * @brief Base class for controller handles that interact with a controller through a ROS action server.
 */
template <typename T>
class ActionBasedControllerHandle : public ActionBasedControllerHandleBase
{
public:
  ActionBasedControllerHandle(const rclcpp::Node::SharedPtr& node, const std::string& name, const std::string& ns,
                              const std::string& logger_name)
    : ActionBasedControllerHandleBase(name, logger_name), node_(node), done_(true), namespace_(ns)
  {
    // Creating the action client does not ensure that the action server is actually running. Executing trajectories
    // through the controller handle will fail if the server is not running when an action goal message is sent.
    controller_action_client_ = rclcpp_action::create_client<T>(node, getActionName());
    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  /**
   * @brief Cancels trajectory execution by triggering the controller action server's cancellation callback.
   * @return True if cancellation was accepted, false if cancellation failed.
   */
  bool cancelExecution() override
  {
    if (!controller_action_client_)
      return false;
    if (!done_)
    {
      if (!current_goal_)
      {
        // Nothing accepted by the server yet (or the goal was abandoned): there is no goal handle to cancel.
        RCLCPP_WARN_STREAM(logger_, "Cancel requested for " << name_ << " but no goal handle is held; nothing to cancel");
        last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
        done_ = true;
        return true;
      }
      RCLCPP_INFO_STREAM(logger_, "Cancelling execution for " << name_);
      auto cancel_result_future = controller_action_client_->async_cancel_goal(current_goal_);

      // Bounded wait: an unresponsive action server (crashed/hung controller)
      // otherwise blocks this .get() forever — and cancelExecution runs inside
      // stopExecutionInternal() while the TEM holds execution_state_mutex_, so
      // an infinite wait here wedges every motion on the manager.
      if (cancel_result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      {
        RCLCPP_ERROR(logger_, "Cancel response not received within 5s; abandoning cancel wait");
      }
      else
      {
        const auto& result = cancel_result_future.get();
        if (!result)
          RCLCPP_ERROR(logger_, "Failed to cancel goal");
      }

      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      done_ = true;
    }
    return true;
  }

  /**
   * @brief Callback function to call when the result is received from the controller action server.
   * @param wrapped_result
   */
  virtual void
  controllerDoneCallback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult& wrapped_result) = 0;

  /**
   * @brief Blocks waiting for the action result to be received.
   *
   * The result callback is registered exactly once per goal, inside sendGoal(); this function only waits on the
   * flag that callback sets. It is therefore safe to call repeatedly with short timeouts (the trajectory execution
   * manager polls it once per second). The previous implementation re-registered a result callback with
   * rclcpp_action on every call, which raced with result delivery: rclcpp_action invokes and clears the registered
   * callback once, so a poll that arrived between delivery and goal-handle erasure registered a callback that could
   * never fire and then waited on it forever (FryStock wedge, 2026-09-03).
   *
   * @param timeout Duration to wait for a result before failing. A negative value means no timeout.
   * @return True if a result was received, false on timeout.
   */
  bool waitForExecution(const rclcpp::Duration& timeout = rclcpp::Duration::from_seconds(-1.0)) override
  {
    std::unique_lock<std::mutex> lock(result_mutex_);
    if (timeout < std::chrono::nanoseconds(0))
    {
      result_cv_.wait(lock, [this] { return result_received_; });
      return true;
    }
    return result_cv_.wait_for(lock, timeout.to_chrono<std::chrono::nanoseconds>(),
                               [this] { return result_received_; });
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return last_exec_;
  }

  void addJoint(const std::string& name) override
  {
    joints_.push_back(name);
  }

  void getJoints(std::vector<std::string>& joints) override
  {
    joints = joints_;
  }

protected:
  /**
   * @brief A pointer to the node, required to read parameters and get the time.
   */
  const rclcpp::Node::SharedPtr node_;

  /**
   * @brief Check if the controller's action server is ready to receive action goals.
   * @return True if the action server is ready, false if it is not ready or does not exist.
   */
  bool isConnected() const
  {
    return controller_action_client_->action_server_is_ready();
  }

  /**
   * @brief Get the full name of the action using the action namespace and base name.
   * @return The action name.
   */
  std::string getActionName() const
  {
    if (namespace_.empty())
    {
      return name_;
    }
    else
    {
      return name_ + "/" + namespace_;
    }
  }

  /**
   * @brief Send a goal to the action server and wait (bounded) for it to be accepted or rejected.
   *
   * Registers the result callback once, through SendGoalOptions, so that rclcpp_action delivers the result to
   * controllerDoneCallback() exactly once and waitForExecution() never has to touch the action client again.
   * The wait for the goal response is bounded: an action server that never answers used to block the trajectory
   * execution manager's worker thread forever (and with it every motion on the manager). If the server accepts the
   * goal after we gave up, the response callback cancels it so the controller does not move unowned.
   *
   * @param goal The goal to send.
   * @param send_goal_options Options; goal_response_callback and feedback_callback are honoured, result_callback is
   *        overwritten.
   * @return True if the goal was accepted, false if it was rejected or no response arrived in time.
   */
  bool sendGoal(const typename T::Goal& goal, typename rclcpp_action::Client<T>::SendGoalOptions send_goal_options)
  {
    const uint64_t seq = ++goal_seq_;
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      result_received_ = false;
      goal_abandoned_ = false;
    }
    current_goal_.reset();
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;

    send_goal_options.result_callback =
        [this, seq](const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult& wrapped_result) {
          if (seq != goal_seq_)
          {
            RCLCPP_WARN_STREAM(logger_, "Ignoring late result for a superseded goal on " << name_);
            return;
          }
          controllerDoneCallback(wrapped_result);
          {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_received_ = true;
          }
          result_cv_.notify_all();
        };

    auto user_response_cb = send_goal_options.goal_response_callback;
    send_goal_options.goal_response_callback =
        [this, seq, user_response_cb](const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr& goal_handle) {
          if (user_response_cb)
            user_response_cb(goal_handle);
          bool abandoned;
          {
            std::lock_guard<std::mutex> lock(result_mutex_);
            abandoned = goal_abandoned_ && seq == goal_seq_;
          }
          if (abandoned && goal_handle)
          {
            RCLCPP_ERROR_STREAM(logger_, "Goal for " << name_ << " was accepted after we stopped waiting for the "
                                                          "response; cancelling it so the controller does not run "
                                                          "an unowned goal");
            controller_action_client_->async_cancel_goal(goal_handle);
          }
        };

    auto goal_handle_future = controller_action_client_->async_send_goal(goal, send_goal_options);
    if (goal_handle_future.wait_for(GOAL_RESPONSE_TIMEOUT) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(logger_, "No goal response from " << getActionName() << " within "
                                                             << std::chrono::duration_cast<std::chrono::seconds>(
                                                                    GOAL_RESPONSE_TIMEOUT)
                                                                    .count()
                                                             << "s; abandoning the goal");
      {
        std::lock_guard<std::mutex> lock(result_mutex_);
        goal_abandoned_ = true;
        result_received_ = true;  // nothing will ever be delivered for this goal to a waiter
      }
      // The response may have landed between our timeout and the flag above; if so, cancel it here.
      if (goal_handle_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        auto late_handle = goal_handle_future.get();
        if (late_handle)
          controller_action_client_->async_cancel_goal(late_handle);
      }
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      done_ = true;
      return false;
    }
    current_goal_ = goal_handle_future.get();
    if (!current_goal_)
    {
      RCLCPP_ERROR_STREAM(logger_, "Goal was rejected by server: " << getActionName());
      {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_received_ = true;
      }
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      done_ = true;
      return false;
    }
    return true;
  }

  /**
   * @brief Indicate that the controller handle is done executing the trajectory and set the controller manager handle's
   * ExecutionStatus based on the received action ResultCode.
   * @param rclcpp_action::ResultCode to convert to moveit_controller_manager::ExecutionStatus.
   */
  void finishControllerExecution(const rclcpp_action::ResultCode& state)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Controller " << name_ << " is done with state " << static_cast<int>(state));
    if (state == rclcpp_action::ResultCode::SUCCEEDED)
    {
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    }
    else if (state == rclcpp_action::ResultCode::ABORTED)
    {
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    }
    else if (state == rclcpp_action::ResultCode::CANCELED)
    {
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    }
    else if (state == rclcpp_action::ResultCode::UNKNOWN)
    {
      last_exec_ = moveit_controller_manager::ExecutionStatus::UNKNOWN;
    }
    else
    {
      last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    }
    done_ = true;
  }

  /**
   * @brief Status after last trajectory execution.
   */
  moveit_controller_manager::ExecutionStatus last_exec_;

  /**
   * @brief Indicates whether the controller handle is done executing its current trajectory.
   */
  bool done_;

  /**
   * @brief The controller namespace. The controller action server's topics will map to name/ns/goal, name/ns/result, etc.
   */
  std::string namespace_;

  /**
   * @brief The joints controlled by this controller.
   */
  std::vector<std::string> joints_;

  /**
   * @brief Action client to send trajectories to the controller's action server.
   */
  typename rclcpp_action::Client<T>::SharedPtr controller_action_client_;

  /**
   * @brief Current goal that has been sent to the action server.
   */
  typename rclcpp_action::ClientGoalHandle<T>::SharedPtr current_goal_;

  /**
   * @brief How long sendGoal() waits for the action server to accept or reject a goal.
   */
  static constexpr std::chrono::seconds GOAL_RESPONSE_TIMEOUT{ 10 };

  /**
   * @brief Guards result_received_ and goal_abandoned_; paired with result_cv_.
   */
  std::mutex result_mutex_;
  std::condition_variable result_cv_;

  /**
   * @brief Set (once, by the result callback registered in sendGoal) when the result of the current goal arrived, or
   * when there is no goal whose result could still arrive. Initially true: nothing is pending.
   */
  bool result_received_ = true;

  /**
   * @brief Set when sendGoal() gave up waiting for the goal response; a late acceptance is then cancelled.
   */
  bool goal_abandoned_ = false;

  /**
   * @brief Monotonic goal counter so late callbacks of a superseded goal are ignored.
   */
  std::atomic<uint64_t> goal_seq_{ 0 };
};

}  // namespace moveit_simple_controller_manager
