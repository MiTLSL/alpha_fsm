#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to robot_msgs__msg__BehaviorState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub system_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub wall_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub active_substate: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub last_error_code: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub summary_json: std::string::String,

}



impl Default for BehaviorState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::BehaviorState::default())
  }
}

impl rosidl_runtime_rs::Message for BehaviorState {
  type RmwMsg = super::msg::rmw::BehaviorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        system_state: msg.system_state.as_str().into(),
        task_state: msg.task_state.as_str().into(),
        wall_state: msg.wall_state.as_str().into(),
        active_substate: msg.active_substate.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        last_error_code: msg.last_error_code,
        summary_json: msg.summary_json.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        system_state: msg.system_state.as_str().into(),
        task_state: msg.task_state.as_str().into(),
        wall_state: msg.wall_state.as_str().into(),
        active_substate: msg.active_substate.as_str().into(),
        task_id: msg.task_id.as_str().into(),
      last_error_code: msg.last_error_code,
        summary_json: msg.summary_json.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      system_state: msg.system_state.to_string(),
      task_state: msg.task_state.to_string(),
      wall_state: msg.wall_state.to_string(),
      active_substate: msg.active_substate.to_string(),
      task_id: msg.task_id.to_string(),
      last_error_code: msg.last_error_code,
      summary_json: msg.summary_json.to_string(),
    }
  }
}


// Corresponds to robot_msgs__msg__NavDebug

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavDebug {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_type: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_pose: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_error: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub details_json: std::string::String,

}



impl Default for NavDebug {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavDebug::default())
  }
}

impl rosidl_runtime_rs::Message for NavDebug {
  type RmwMsg = super::msg::rmw::NavDebug;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        source: msg.source.as_str().into(),
        goal_type: msg.goal_type.as_str().into(),
        target_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.target_pose)).into_owned(),
        distance_remaining: msg.distance_remaining,
        yaw_error: msg.yaw_error,
        status: msg.status.as_str().into(),
        details_json: msg.details_json.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        source: msg.source.as_str().into(),
        goal_type: msg.goal_type.as_str().into(),
        target_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target_pose)).into_owned(),
      distance_remaining: msg.distance_remaining,
      yaw_error: msg.yaw_error,
        status: msg.status.as_str().into(),
        details_json: msg.details_json.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      source: msg.source.to_string(),
      goal_type: msg.goal_type.to_string(),
      target_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.target_pose),
      distance_remaining: msg.distance_remaining,
      yaw_error: msg.yaw_error,
      status: msg.status.to_string(),
      details_json: msg.details_json.to_string(),
    }
  }
}


