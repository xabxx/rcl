// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>

#include "rcl_action/action_client.h"
#include "rcl_action/action_server.h"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "test_msgs/action/fibonacci.h"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

// TODO(jacobperron): Add action client to complete tests
class CLASSNAME (TestActionCommunication, RMW_IMPLEMENTATION) : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rcl_ret_t ret = rcl_init(0, nullptr, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    this->node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&this->node, "test_action_communication_node", "", &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    const rosidl_action_type_support_t * ts = ROSIDL_GET_ACTION_TYPE_SUPPORT(
      test_msgs, Fibonacci);
    const char * action_name = "test_action_commmunication_name";
    const rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
    this->action_server = rcl_action_get_zero_initialized_server();
    ret = rcl_action_server_init(
      &this->action_server, &this->node, ts, action_name, &server_options);
    ASSERT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
    const rcl_action_client_options_t client_options = rcl_action_client_get_default_options();
    this->action_client = rcl_action_get_zero_initialized_client();
    ret = rcl_action_client_init(
      &this->action_client, &this->node, ts, action_name, &client_options);
    ASSERT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  }

  void TearDown() override
  {
    // Finalize
    rcl_ret_t ret = rcl_action_server_fini(&this->action_server, &this->node);
    EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
    rcl_reset_error();
    ret = rcl_action_client_fini(&this->action_client, &this->node);
    EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
    rcl_reset_error();
    ret = rcl_shutdown();
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    rcl_reset_error();
  }

  void init_test_uuid0(uint8_t * uuid)
  {
    for (uint8_t i = 0; i < 16; ++i) {
      uuid[i] = i;
    }
  }

  void init_test_uuid1(uint8_t * uuid)
  {
    for (uint8_t i = 0; i < 16; ++i) {
      uuid[i] = 15 - i;
    }
  }

  rcl_action_client_t action_client;
  rcl_action_server_t action_server;
  rcl_node_t node;
};  // class TestActionCommunication

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_goal_request_comm)
{
  test_msgs__action__Fibonacci_Goal_Request outgoing_goal_request;
  test_msgs__action__Fibonacci_Goal_Request incoming_goal_request;
  test_msgs__action__Fibonacci_Goal_Request__init(&outgoing_goal_request);

  // Send goal request with null action client
  rcl_ret_t ret = rcl_action_send_goal_request(nullptr, &outgoing_goal_request);
  ASSERT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID);
  rcl_reset_error();

  // Send goal request with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_send_goal_request(&invalid_action_client, &outgoing_goal_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID);
  rcl_reset_error();

  // Send goal request with null message
  ret = rcl_action_send_goal_request(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send goal request with valid arguments
  ret = rcl_action_send_goal_request(&this->action_client, &outgoing_goal_request);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take goal request with null action server
  ret = rcl_action_take_goal_request(nullptr, &incoming_goal_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take goal request with null message
  ret = rcl_action_take_goal_request(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take goal request with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_take_goal_request(&invalid_action_server, &incoming_goal_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take goal request with valid arguments
  ret = rcl_action_take_goal_request(&this->action_server, &incoming_goal_request);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  test_msgs__action__Fibonacci_Goal_Request__fini(&outgoing_goal_request);
  test_msgs__action__Fibonacci_Goal_Request__init(&incoming_goal_request);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_goal_response_comm)
{
  test_msgs__action__Fibonacci_Goal_Response outgoing_goal_response;
  test_msgs__action__Fibonacci_Goal_Response incoming_goal_response;
  test_msgs__action__Fibonacci_Goal_Response__init(&outgoing_goal_response);
  test_msgs__action__Fibonacci_Goal_Response__init(&incoming_goal_response);

  // Send goal response with null action server
  rcl_ret_t ret = rcl_action_send_goal_response(nullptr, &outgoing_goal_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send goal response with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_send_goal_response(&invalid_action_server, &outgoing_goal_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send goal response with null message
  ret = rcl_action_send_goal_response(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send goal response with valid arguments
  ret = rcl_action_send_goal_response(&this->action_server, &outgoing_goal_response);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take goal response with null action client
  ret = rcl_action_take_goal_response(nullptr, &incoming_goal_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID);
  rcl_reset_error();

  // Take goal response with null message
  ret = rcl_action_take_goal_response(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT);
  rcl_reset_error();

  // Take goal response with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_take_goal_response(&invalid_action_client, &incoming_goal_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take goal response with valid arguments
  ret = rcl_action_take_goal_response(&this->action_client, &incoming_goal_response);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  test_msgs__action__Fibonacci_Goal_Response__fini(&incoming_goal_response);
  test_msgs__action__Fibonacci_Goal_Response__fini(&outgoing_goal_response);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_cancel_request_comm)
{
  action_msgs__srv__CancelGoal_Request outgoing_cancel_request;
  action_msgs__srv__CancelGoal_Request incoming_cancel_request;
  action_msgs__srv__CancelGoal_Request__init(&outgoing_cancel_request);
  action_msgs__srv__CancelGoal_Request__init(&incoming_cancel_request);

  // Send cancel request with null action client
  rcl_ret_t ret = rcl_action_send_cancel_request(nullptr, &outgoing_cancel_request);
  ASSERT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send cancel request with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_send_cancel_request(&invalid_action_client, &outgoing_cancel_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send cancel request with null message
  ret = rcl_action_send_cancel_request(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send cancel request with valid arguments
  ret = rcl_action_send_cancel_request(&this->action_client, &outgoing_cancel_request);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel request with null action server
  ret = rcl_action_take_cancel_request(nullptr, &incoming_cancel_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel request with null message
  ret = rcl_action_take_cancel_request(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel request with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_take_cancel_request(&invalid_action_server, &incoming_cancel_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel request with valid arguments
  ret = rcl_action_take_cancel_request(&this->action_server, &incoming_cancel_request);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  action_msgs__srv__CancelGoal_Request__fini(&incoming_cancel_request);
  action_msgs__srv__CancelGoal_Request__fini(&outgoing_cancel_request);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_cancel_response_comm)
{
  action_msgs__srv__CancelGoal_Response outgoing_cancel_response;
  action_msgs__srv__CancelGoal_Response incoming_cancel_response;
  action_msgs__srv__CancelGoal_Response__init(&outgoing_cancel_response);
  action_msgs__srv__CancelGoal_Response__init(&incoming_cancel_response);

  // Send cancel response with null action server
  rcl_ret_t ret = rcl_action_send_cancel_response(nullptr, &outgoing_cancel_response);
  ASSERT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send cancel response with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_send_cancel_response(&invalid_action_server, &outgoing_cancel_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send cancel response with null message
  ret = rcl_action_send_cancel_response(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send cancel response with valid arguments
  ret = rcl_action_send_cancel_response(&this->action_server, &outgoing_cancel_response);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel response with null action client
  ret = rcl_action_take_cancel_response(nullptr, &incoming_cancel_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel response with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_take_cancel_response(&invalid_action_client, &incoming_cancel_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel response with null message
  ret = rcl_action_take_cancel_response(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take cancel response with valid arguments
  ret = rcl_action_take_cancel_response(&this->action_client, &incoming_cancel_response);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  action_msgs__srv__CancelGoal_Response__fini(&incoming_cancel_response);
  action_msgs__srv__CancelGoal_Response__fini(&outgoing_cancel_response);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_result_request_comm)
{
  test_msgs__action__Fibonacci_Result_Request outgoing_result_request;
  test_msgs__action__Fibonacci_Result_Request incoming_result_request;
  test_msgs__action__Fibonacci_Result_Request__init(&outgoing_result_request);
  test_msgs__action__Fibonacci_Result_Request__init(&incoming_result_request);

  // Send result request with null action client
  rcl_ret_t ret = rcl_action_send_result_request(nullptr, &outgoing_result_request);
  ASSERT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send result request with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_send_result_request(&invalid_action_client, &outgoing_result_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send result request with null message
  ret = rcl_action_send_result_request(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send result request with valid arguments
  ret = rcl_action_send_result_request(&this->action_client, &outgoing_result_request);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result request with null action server
  ret = rcl_action_take_result_request(nullptr, &incoming_result_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result request with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_take_result_request(&invalid_action_server, &incoming_result_request);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result request with null message
  ret = rcl_action_take_result_request(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result request with valid arguments
  ret = rcl_action_take_result_request(&this->action_server, &incoming_result_request);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  test_msgs__action__Fibonacci_Result_Request__fini(&incoming_result_request);
  test_msgs__action__Fibonacci_Result_Request__fini(&outgoing_result_request);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_result_response_comm)
{
  test_msgs__action__Fibonacci_Result_Response outgoing_result_response;
  test_msgs__action__Fibonacci_Result_Response incoming_result_response;
  test_msgs__action__Fibonacci_Result_Response__init(&outgoing_result_response);
  test_msgs__action__Fibonacci_Result_Response__init(&incoming_result_response);

  // Send result response with null action client
  rcl_ret_t ret = rcl_action_send_result_response(nullptr, &outgoing_result_response);
  ASSERT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send result response with invalid action client
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_send_result_response(&invalid_action_server, &outgoing_result_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send result response with null message
  ret = rcl_action_send_result_response(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Send result response with valid arguments
  ret = rcl_action_send_result_response(&this->action_server, &outgoing_result_response);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result response with null action client
  ret = rcl_action_take_result_response(nullptr, &incoming_result_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result response with null message
  ret = rcl_action_take_result_response(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result response with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_take_result_response(&invalid_action_client, &incoming_result_response);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take result response with valid arguments
  ret = rcl_action_take_result_response(&this->action_client, &incoming_result_response);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  test_msgs__action__Fibonacci_Result_Response__fini(&incoming_result_response);
  test_msgs__action__Fibonacci_Result_Response__fini(&outgoing_result_response);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_feedback_comm)
{
  test_msgs__action__Fibonacci_Feedback outgoing_feedback;
  test_msgs__action__Fibonacci_Feedback incoming_feedback;
  test_msgs__action__Fibonacci_Feedback__init(&outgoing_feedback);
  test_msgs__action__Fibonacci_Feedback__init(&incoming_feedback);

  // Publish feedback with null action server
  rcl_ret_t ret = rcl_action_publish_feedback(nullptr, &outgoing_feedback);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish feedback with null message
  ret = rcl_action_publish_feedback(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish feedback with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_publish_feedback(&invalid_action_server, &outgoing_feedback);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish feedback with valid arguments
  ret = rcl_action_publish_feedback(&this->action_server, &outgoing_feedback);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take feedback with null action client
  ret = rcl_action_take_feedback(nullptr, &incoming_feedback);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take feedback with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_take_feedback(&invalid_action_client, &incoming_feedback);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take feedback with null message
  ret = rcl_action_take_feedback(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take feedback with valid arguments
  ret = rcl_action_take_feedback(&this->action_client, &incoming_feedback);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  test_msgs__action__Fibonacci_Feedback__fini(&incoming_feedback);
  test_msgs__action__Fibonacci_Feedback__fini(&outgoing_feedback);
}

TEST_F(CLASSNAME(TestActionCommunication, RMW_IMPLEMENTATION), test_status_comm)
{
  action_msgs__msg__GoalStatusArray incoming_status;
  action_msgs__msg__GoalStatusArray__init(&incoming_status);

  // Using rcl_action_goal_status_array_t in lieu of a message instance works
  // because these tests make use of C type support
  rcl_action_goal_status_array_t status_array =
    rcl_action_get_zero_initialized_goal_status_array();
  rcl_ret_t ret = rcl_action_get_goal_status_array(&this->action_server, &status_array);
  ASSERT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;

  // Publish status with null action server
  ret = rcl_action_publish_status(nullptr, &status_array.msg);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish status with null message
  ret = rcl_action_publish_status(&this->action_server, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish status with invalid action server
  rcl_action_server_t invalid_action_server = rcl_action_get_zero_initialized_server();
  ret = rcl_action_publish_status(&invalid_action_server, &status_array.msg);
  EXPECT_EQ(ret, RCL_RET_ACTION_SERVER_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish status with valid arguments (but empty array)
  ret = rcl_action_publish_status(&this->action_server, &status_array.msg);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take status with null action client
  ret = rcl_action_take_status(nullptr, &incoming_status);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take status with invalid action client
  rcl_action_client_t invalid_action_client = rcl_action_get_zero_initialized_client();
  ret = rcl_action_take_status(&invalid_action_client, &incoming_status);
  EXPECT_EQ(ret, RCL_RET_ACTION_CLIENT_INVALID) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take status with null message
  ret = rcl_action_take_status(&this->action_client, nullptr);
  EXPECT_EQ(ret, RCL_RET_INVALID_ARGUMENT) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take status with valid arguments (empty array)
  ret = rcl_action_take_status(&this->action_client, &incoming_status);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  ret = rcl_action_goal_status_array_fini(&status_array);
  ASSERT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;

  // Add a goal before publishing the status array
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  rcl_action_goal_handle_t * goal_handle;
  goal_handle = rcl_action_accept_new_goal(&this->action_server, &goal_info);
  ASSERT_NE(goal_handle, nullptr) << rcl_get_error_string().str;
  ret = rcl_action_get_goal_status_array(&this->action_server, &status_array);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Publish status with valid arguments (one goal in array)
  ret = rcl_action_publish_status(&this->action_server, &status_array.msg);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  // Take status with valid arguments (one goal in array)
  ret = rcl_action_take_status(&this->action_client, &incoming_status);
  EXPECT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;
  rcl_reset_error();

  ret = rcl_action_goal_status_array_fini(&status_array);
  ASSERT_EQ(ret, RCL_RET_OK) << rcl_get_error_string().str;

  action_msgs__msg__GoalStatusArray__fini(&incoming_status);
}
