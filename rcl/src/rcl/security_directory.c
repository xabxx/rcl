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

#include "rcl/security_directory.h"
#include "./tinydir/tinydir.h"
#include "rcutils/filesystem.h"
#include "rcutils/get_env.h"

/// Return the directory whose name most closely matches node_name (longest-prefix match), scanning under base_dir.
/**
 * By using a prefix match, a node named e.g. "my_node_123" will be able to load and use the directory "my_node" if no better match exists.
 * \param[in] base_dir
 * \param[in] node_name
 * \param[out] matched_name must be a valid memory address allocated with at least _TINYDIR_FILENAME_MAX characters.
 * \return true if a match was found
 */
static bool get_best_matching_directory(
    const char * base_dir,
    const char * node_name,
    char * matched_name)
{
    size_t max_match_length = 0;
    tinydir_dir dir;
    if (NULL == base_dir || NULL == node_name || NULL == matched_name) {
        return false;
    }
    if (-1 == tinydir_open(&dir, base_dir)) {
        return false;
    }
    while (dir.has_next) {
        tinydir_file file;
        if (-1 == tinydir_readfile(&dir, &file)) {
            goto cleanup;
        }
        if (!file.is_dir) {
            continue;
        }
        size_t matched_name_length = strnlen(file.name, sizeof(file.name) - 1);
        if (0 == strncmp(file.name, node_name, matched_name_length) && matched_name_length > max_match_length) {
            max_match_length = matched_name_length;
            strncpy(matched_name, file.name, max_match_length);
        }
        if (-1 == tinydir_next(&dir)) {
            goto cleanup;
        }
    }
    cleanup:
    tinydir_close(&dir);
    return max_match_length > 0;
}

/// Return the secure root directory associated with a node given its validated name and namespace.
/**
 * E.g. for a node named "c" in namespace "/a/b", the secure root path will be
 * "a/b/c", where the delimiter "/" is native for target file system (e.g. "\\" for _WIN32).
 * If no exact match is found for the node name, a best match would be used instead (by performing longest-prefix matching).
 *
 * However, this expansion can be overridden by setting the secure node directory environment
 * variable, allowing users to explicitly specify the exact secure root directory to be utilized.
 * Such an override is useful for where the FQN of a node is non-deterministic before runtime,
 * or when testing and using additional tools that may not otherwise not be easily provisioned.
 *
 * \param[in] node_name validated node name (a single token)
 * \param[in] node_namespace validated, absolute namespace (starting with "/")
 * \param[in] allocator the allocator to use for allocation
 * \returns machine specific (absolute) node secure root path or NULL on failure
 */
const char * rcl_get_secure_root(
    const char * node_name,
    const char * node_namespace,
    const rcl_allocator_t * allocator)
{
    bool ros_secure_node_override = true;
    const char * ros_secure_root_env = NULL;
    if (NULL == node_name) {
        return NULL;
    }
    if (rcutils_get_env(ROS_SECURITY_NODE_DIRECTORY_VAR_NAME, &ros_secure_root_env)) {
        return NULL;
    }
    if (!ros_secure_root_env) {
        return NULL;
    }
    size_t ros_secure_root_size = strlen(ros_secure_root_env);
    if (!ros_secure_root_size) {
        // check root directory if node directory environment variable is empty
        if (rcutils_get_env(ROS_SECURITY_ROOT_DIRECTORY_VAR_NAME, &ros_secure_root_env)) {
            return NULL;
        }
        if (!ros_secure_root_env) {
            return NULL;
        }
        ros_secure_root_size = strlen(ros_secure_root_env);
        if (!ros_secure_root_size) {
            return NULL;  // environment variable was empty
        } else {
            ros_secure_node_override = false;
        }
    }

    char * node_secure_root = NULL;
    if (ros_secure_node_override) {
        node_secure_root =
                (char *)allocator->allocate(ros_secure_root_size + 1, allocator->state);
        memcpy(node_secure_root, ros_secure_root_env, ros_secure_root_size + 1);
        // TODO(ros2team): This make an assumption on the value and length of the root namespace.
        // This should likely come from another (rcl/rmw?) function for reuse.
        // If the namespace is the root namespace ("/"), the secure root is just the node name.
    } else {
        // Perform longest prefix match for the node's name in directory <root dir>/<namespace>.
        char matched_dir[_TINYDIR_FILENAME_MAX] = {0};
        char * base_lookup_dir = NULL;
        if (strlen(node_namespace) == 1) {
            base_lookup_dir = (char *) ros_secure_root_env;
        } else {
            // TODO(ros2team): remove the hard-coded length, use the length of the root namespace instead.
            base_lookup_dir = rcutils_join_path(ros_secure_root_env, node_namespace + 1, *allocator);
        }
        if (get_best_matching_directory(base_lookup_dir, node_name, matched_dir)) {
            node_secure_root = rcutils_join_path(base_lookup_dir, matched_dir, *allocator);
        }
        if (base_lookup_dir != ros_secure_root_env && NULL != base_lookup_dir) {
            allocator->deallocate(base_lookup_dir, allocator->state);
        }
    }
    // Check node_secure_root is not NULL before checking directory
    if (NULL == node_secure_root) {
        return NULL;
    } else if (!rcutils_is_directory(node_secure_root)) {
        allocator->deallocate(node_secure_root, allocator->state);
        return NULL;
    }
    return node_secure_root;
}