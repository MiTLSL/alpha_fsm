#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__msg__BehaviorState() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__msg__BehaviorState__init(msg: *mut BehaviorState) -> bool;
    fn robot_msgs__msg__BehaviorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BehaviorState>, size: usize) -> bool;
    fn robot_msgs__msg__BehaviorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BehaviorState>);
    fn robot_msgs__msg__BehaviorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BehaviorState>, out_seq: *mut rosidl_runtime_rs::Sequence<BehaviorState>) -> bool;
}

// Corresponds to robot_msgs__msg__BehaviorState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub system_state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub wall_state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub active_substate: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub last_error_code: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub summary_json: rosidl_runtime_rs::String,

}



impl Default for BehaviorState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__msg__BehaviorState__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__msg__BehaviorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BehaviorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__BehaviorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__BehaviorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__BehaviorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BehaviorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BehaviorState where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/msg/BehaviorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__msg__BehaviorState() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__msg__NavDebug() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__msg__NavDebug__init(msg: *mut NavDebug) -> bool;
    fn robot_msgs__msg__NavDebug__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavDebug>, size: usize) -> bool;
    fn robot_msgs__msg__NavDebug__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavDebug>);
    fn robot_msgs__msg__NavDebug__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavDebug>, out_seq: *mut rosidl_runtime_rs::Sequence<NavDebug>) -> bool;
}

// Corresponds to robot_msgs__msg__NavDebug
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavDebug {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_type: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_pose: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_error: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub details_json: rosidl_runtime_rs::String,

}



impl Default for NavDebug {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__msg__NavDebug__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__msg__NavDebug__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavDebug {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__NavDebug__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__NavDebug__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__NavDebug__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavDebug {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavDebug where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/msg/NavDebug";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__msg__NavDebug() }
  }
}


