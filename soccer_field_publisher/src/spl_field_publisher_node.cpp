// Copyright 2021 Kenji Brameld
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

#define _USE_MATH_DEFINES
 
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "soccer_field_msgs/msg/field.hpp"

namespace soccer_field_publisher
{

class SPLFieldPublisherNode : public rclcpp::Node
{
public:
  SPLFieldPublisherNode()
  : Node("SPLFieldPublisherNode")
  {
    float field_length = this->declare_parameter<float>("field.length");
    float field_width = this->declare_parameter<float>("field.width");
    float field_penaltyCrossSize = this->declare_parameter<float>("field.penaltyCrossSize");
    float field_goalBoxAreaLength = this->declare_parameter<float>("field.goalBoxAreaLength");
    float field_goalBoxAreaWidth = this->declare_parameter<float>("field.goalBoxAreaWidth");
    float field_penaltyAreaLength = this->declare_parameter<float>("field.penaltyAreaLength");
    float field_penaltyAreaWidth = this->declare_parameter<float>("field.penaltyAreaWidth");
    float field_penaltyCrossDistance = this->declare_parameter<float>("field.penaltyCrossDistance");
    float field_centerCircleDiameter = this->declare_parameter<float>("field.centerCircleDiameter");
    float field_borderStripWidth = this->declare_parameter<float>("field.borderStripWidth");
    float goal_postDiameter = this->declare_parameter<float>("goal.postDiameter");
    float goal_height = this->declare_parameter<float>("goal.height");
    float goal_innerWidth = this->declare_parameter<float>("goal.innerWidth");
    float goal_depth = this->declare_parameter<float>("goal.depth");

    RCLCPP_DEBUG(get_logger(), "Parameters: ");
    RCLCPP_DEBUG(get_logger(), "  field_length : %f", field_length);
    RCLCPP_DEBUG(get_logger(), "  field_width : %f", field_width);
    RCLCPP_DEBUG(get_logger(), "  field_penaltyCrossSize : %f", field_penaltyCrossSize);
    RCLCPP_DEBUG(get_logger(), "  field_goalBoxAreaLength : %f", field_goalBoxAreaLength);
    RCLCPP_DEBUG(get_logger(), "  field_goalBoxAreaWidth : %f", field_goalBoxAreaWidth);
    RCLCPP_DEBUG(get_logger(), "  field_penaltyAreaLength : %f", field_penaltyAreaLength);
    RCLCPP_DEBUG(get_logger(), "  field_penaltyAreaWidth : %f", field_penaltyAreaWidth);
    RCLCPP_DEBUG(get_logger(), "  field_penaltyCrossDistance : %f", field_penaltyCrossDistance);
    RCLCPP_DEBUG(get_logger(), "  field_centerCircleDiameter : %f", field_centerCircleDiameter);
    RCLCPP_DEBUG(get_logger(), "  field_borderStripWidth : %f", field_borderStripWidth);
    RCLCPP_DEBUG(get_logger(), "  goal_postDiameter : %f", goal_postDiameter);
    RCLCPP_DEBUG(get_logger(), "  goal_height : %f", goal_height);
    RCLCPP_DEBUG(get_logger(), "  goal_innerWidth : %f", goal_innerWidth);
    RCLCPP_DEBUG(get_logger(), "  goal_depth : %f", goal_depth);

    float line_width = 0.05;
    float half_line_width = line_width / 2;
    // float goal_post_diameter = 0.100;
    // float goal_post_radius = goal_post_diameter / 2;
    // float goal_crossbar_diameter = 0.100;
    // float goal_crossbar_radius = goal_crossbar_diameter / 2.0;

    publisher_ = this->create_publisher<soccer_field_msgs::msg::Field>(
      "field", rclcpp::QoS(1).transient_local());

    soccer_field_msgs::msg::Field field;

    // The coordinate system used for terms: North, South, East and West
    // is defined as having the game controller at the South of the field.

    // Field of play - North-West Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = -field_length / 2 - field_borderStripWidth;
      point.y = field_width / 2 + field_borderStripWidth;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Field of play - North-East Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = field_length / 2 + field_borderStripWidth;
      point.y = field_width / 2 + field_borderStripWidth;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Field of play - South-East Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = field_length / 2 + field_borderStripWidth;
      point.y = -field_width / 2 - field_borderStripWidth;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Field of play - South-West Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = -field_length / 2 - field_borderStripWidth;
      point.y = -field_width / 2 - field_borderStripWidth;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Center line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = 0;
      marking.start.y = -field_width / 2 - half_line_width;
      marking.end.x = 0;
      marking.end.y = field_width / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // Center circle
    {
      soccer_field_msgs::msg::ArcMarking marking;
      marking.center.x = 0;
      marking.center.y = 0;
      marking.center.z = 0;
      marking.heading_start = 0;
      marking.heading_end = 2 * M_PI;
      marking.radius = field_centerCircleDiameter / 2;
      marking.line_width = line_width;
      field.markings.arcs.push_back(marking);
    }

    // West goal line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2;
      marking.start.y = -field_width / 2 - half_line_width;
      marking.end.x = -field_length / 2;
      marking.end.y = field_width / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East goal line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2;
      marking.start.y = -field_width / 2 - half_line_width;
      marking.end.x = field_length / 2;
      marking.end.y = field_width / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // North touch line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 - half_line_width;
      marking.start.y = field_width / 2;
      marking.end.x = field_length / 2 + half_line_width;
      marking.end.y = field_width / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // South touch line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 - half_line_width;
      marking.start.y = -field_width / 2;
      marking.end.x = field_length / 2 + half_line_width;
      marking.end.y = -field_width / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 - half_line_width;
      marking.start.y = field_penaltyAreaWidth / 2;
      marking.end.x = -field_length / 2 + field_penaltyAreaLength + half_line_width;
      marking.end.y = field_penaltyAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Box - Horizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 - half_line_width;
      marking.start.y = -field_penaltyAreaWidth / 2;
      marking.end.x = -field_length / 2 + field_penaltyAreaLength + half_line_width;
      marking.end.y = -field_penaltyAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 + field_penaltyAreaLength;
      marking.start.y = -field_penaltyAreaWidth / 2 - half_line_width;
      marking.end.x = -field_length / 2 + field_penaltyAreaLength;
      marking.end.y = field_penaltyAreaWidth / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 + half_line_width;
      marking.start.y = field_penaltyAreaWidth / 2;
      marking.end.x = field_length / 2 - field_penaltyAreaLength - half_line_width;
      marking.end.y = field_penaltyAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Box - Horizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 + half_line_width;
      marking.start.y = -field_penaltyAreaWidth / 2;
      marking.end.x = field_length / 2 - field_penaltyAreaLength - half_line_width;
      marking.end.y = -field_penaltyAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 - field_penaltyAreaLength;
      marking.start.y = -field_penaltyAreaWidth / 2 - half_line_width;
      marking.end.x = field_length / 2 - field_penaltyAreaLength;
      marking.end.y = field_penaltyAreaWidth / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Goal Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 - half_line_width;
      marking.start.y = field_goalBoxAreaWidth / 2;
      marking.end.x = -field_length / 2 + field_goalBoxAreaLength + half_line_width;
      marking.end.y = field_goalBoxAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Goal Box - Hotizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 - half_line_width;
      marking.start.y = -field_goalBoxAreaWidth / 2;
      marking.end.x = -field_length / 2 + field_goalBoxAreaLength + half_line_width;
      marking.end.y = -field_goalBoxAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Goal Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 + field_goalBoxAreaLength;
      marking.start.y = -field_goalBoxAreaWidth / 2 - half_line_width;
      marking.end.x = -field_length / 2 + field_goalBoxAreaLength;
      marking.end.y = field_goalBoxAreaWidth / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Goal Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 + half_line_width;
      marking.start.y = field_goalBoxAreaWidth / 2;
      marking.end.x = field_length / 2 - field_goalBoxAreaLength - half_line_width;
      marking.end.y = field_goalBoxAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Goal Box - Hotizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 + half_line_width;
      marking.start.y = -field_goalBoxAreaWidth / 2;
      marking.end.x = field_length / 2 - field_goalBoxAreaLength - half_line_width;
      marking.end.y = -field_goalBoxAreaWidth / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Goal Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 - field_goalBoxAreaLength;
      marking.start.y = -field_goalBoxAreaWidth / 2 - half_line_width;
      marking.end.x = field_length / 2 - field_goalBoxAreaLength;
      marking.end.y = field_goalBoxAreaWidth / 2 + half_line_width;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Cross - Horizontal Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 + field_penaltyCrossDistance - field_penaltyCrossSize / 2;
      marking.start.y = 0;
      marking.end.x = -field_length / 2 + field_penaltyCrossDistance + field_penaltyCrossSize / 2;
      marking.end.y = 0;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Cross - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -field_length / 2 + field_penaltyCrossDistance;
      marking.start.y = -field_penaltyCrossSize / 2;
      marking.end.x = -field_length / 2 + field_penaltyCrossDistance;
      marking.end.y = field_penaltyCrossSize / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Cross - Horizontal Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 - field_penaltyCrossDistance - field_penaltyCrossSize / 2;
      marking.start.y = 0;
      marking.end.x = field_length / 2 - field_penaltyCrossDistance + field_penaltyCrossSize / 2;
      marking.end.y = 0;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Cross - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = field_length / 2 - field_penaltyCrossDistance;
      marking.start.y = -field_penaltyCrossSize / 2;
      marking.end.x = field_length / 2 - field_penaltyCrossDistance;
      marking.end.y = field_penaltyCrossSize / 2;
      marking.line_width = line_width;
      field.markings.lines.push_back(marking);
    }

    publisher_->publish(field);
  }

private:
  rclcpp::Publisher<soccer_field_msgs::msg::Field>::SharedPtr publisher_;
};


}  // namespace soccer_field_publisher


int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<soccer_field_publisher::SPLFieldPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
