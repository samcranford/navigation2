/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/visitation_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VisitationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d {
void VisitationLayer::onInitialize() {
    bool track_unknown_space;
    std::string visitation_footprint_string;

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("visited_cell_cost", rclcpp::ParameterValue(1));
    declareParameter("visitation_footprint", rclcpp::ParameterValue(std::string("[ [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5] ]")));

    node_->get_parameter(name_ + "." + "enabled", enabled_);
    node_->get_parameter(name_ + "." + "combination_method", combination_method_);
    node_->get_parameter(name_ + "." + "visited_cell_cost", visited_cell_cost_);
    node_->get_parameter(name_ + "." + "visitation_footprint", visitation_footprint_string);
    node_->get_parameter("track_unknown_space", track_unknown_space);

    // Reset to default if empty
    if (visitation_footprint_string == "" || visitation_footprint_string == "[]") {
        visitation_footprint_string = "[ [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5] ]";
    }
    makeFootprintFromString(visitation_footprint_string, visitation_footprint_);

    matchSize();
    current_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();
    if (track_unknown_space) {
        default_value_ = NO_INFORMATION;
    } else {
        default_value_ = FREE_SPACE;
    }

    reset_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/visitation_layer/reset_visited", 1,
                                                                 std::bind(&VisitationLayer::resetCallback, this, std::placeholders::_1));
}

void VisitationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_) {
        return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VisitationLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {
    transformFootprint(robot_x, robot_y, robot_yaw, visitation_footprint_, transformed_visitation_footprint_);
    for (unsigned int i = 0; i < transformed_visitation_footprint_.size(); i++) {
        touch(transformed_visitation_footprint_[i].x, transformed_visitation_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void VisitationLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) {
        return;
    }
    setConvexPolygonCost(transformed_visitation_footprint_, visited_cell_cost_);
    switch (combination_method_) {
        case 0:  // Overwrite
            updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
            break;
        case 1:  // Maximum
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
            break;
        default:  // Nothing
            break;
    }
}

void VisitationLayer::reset() {
    resetMaps();
}

void VisitationLayer::resetCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        reset();
    }
}

}  // namespace nav2_costmap_2d
