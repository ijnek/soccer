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

class MSLFieldPublisherNode : public rclcpp::Node
{
public:
  MSLFieldPublisherNode()
  : Node("MSLFieldPublisherNode")
  {
    RCLCPP_DEBUG(get_logger(), "Parameters: ");

    float A = this->declare_parameter<float>("field.A");
    RCLCPP_DEBUG(get_logger(), "  A : %f", A);
    float B = this->declare_parameter<float>("field.B");
    RCLCPP_DEBUG(get_logger(), "  B : %f", B);
    float C = this->declare_parameter<float>("field.C");
    RCLCPP_DEBUG(get_logger(), "  C : %f", C);
    float D = this->declare_parameter<float>("field.D");
    RCLCPP_DEBUG(get_logger(), "  D : %f", D);
    float E = this->declare_parameter<float>("field.E");
    RCLCPP_DEBUG(get_logger(), "  E : %f", E);
    float F = this->declare_parameter<float>("field.F");
    RCLCPP_DEBUG(get_logger(), "  F : %f", F);
    float G = this->declare_parameter<float>("field.G");
    RCLCPP_DEBUG(get_logger(), "  G : %f", G);
    float H = this->declare_parameter<float>("field.H");
    RCLCPP_DEBUG(get_logger(), "  H : %f", H);
    float I = this->declare_parameter<float>("field.I");
    RCLCPP_DEBUG(get_logger(), "  I : %f", I);
    float J = this->declare_parameter<float>("field.J");
    RCLCPP_DEBUG(get_logger(), "  J : %f", J);
    float K = this->declare_parameter<float>("field.K");
    RCLCPP_DEBUG(get_logger(), "  K : %f", K);
    float L = this->declare_parameter<float>("field.L");
    RCLCPP_DEBUG(get_logger(), "  L : %f", L);
    float M = this->declare_parameter<float>("field.M");
    RCLCPP_DEBUG(get_logger(), "  M : %f", M);
    float N = this->declare_parameter<float>("field.N");
    RCLCPP_DEBUG(get_logger(), "  N : %f", N);
    float O = this->declare_parameter<float>("field.O");
    RCLCPP_DEBUG(get_logger(), "  O : %f", O);
    float P = this->declare_parameter<float>("field.P");
    RCLCPP_DEBUG(get_logger(), "  P : %f", P);
    float Q = this->declare_parameter<float>("field.Q");
    RCLCPP_DEBUG(get_logger(), "  Q : %f", Q);

    float goal_height = this->declare_parameter<float>("goal.height");
    RCLCPP_DEBUG(get_logger(), "  goal_height : %f", goal_height);
    float goal_innerWidth = this->declare_parameter<float>("goal.innerWidth");
    RCLCPP_DEBUG(get_logger(), "  goal_innerWidth : %f", goal_innerWidth);
    float goal_depth = this->declare_parameter<float>("goal.depth");
    RCLCPP_DEBUG(get_logger(), "  goal_depth : %f", goal_depth);
    float goal_crossbarWidth = this->declare_parameter<float>("goal.crossbarWidth");
    RCLCPP_DEBUG(get_logger(), "  goal_crossbarWidth : %f", goal_crossbarWidth);
    float goal_postWidth = this->declare_parameter<float>("goal.postWidth");
    RCLCPP_DEBUG(get_logger(), "  goal_postWidth : %f", goal_postWidth);
    float goal_safetyZoneHeight = this->declare_parameter<float>("goal.safetyZoneHeight");
    RCLCPP_DEBUG(get_logger(), "  goal_safetyZoneHeight : %f", goal_safetyZoneHeight);

    publisher_ = this->create_publisher<soccer_field_msgs::msg::Field>(
      "field", rclcpp::QoS(1).transient_local());

    float half_line_width = K / 2;

    soccer_field_msgs::msg::Field field;

    // The coordinate system used for terms: North, South, East and West
    // is defined as having the game controller at the South of the field.

    // Field of play - North-West Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = -A / 2 - L;
      point.y = B / 2 + L;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Field of play - North-East Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = A / 2 + L;
      point.y = B / 2 + L;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Field of play - South-East Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = A / 2 + L;
      point.y = -B / 2 - L;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Field of play - South-West Corner Point
    {
      geometry_msgs::msg::Point32 point;
      point.x = -A / 2 - L;
      point.y = -B / 2 - L;
      point.z = 0;
      field.field_of_play.points.push_back(point);
    }

    // Center line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = 0;
      marking.start.y = -B / 2;
      marking.end.x = 0;
      marking.end.y = B / 2;
      marking.line_width = K;
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
      marking.radius = H / 2 - half_line_width;
      marking.line_width = K;
      field.markings.arcs.push_back(marking);
    }

    // West goal line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2 + half_line_width;
      marking.start.y = -B / 2;
      marking.end.x = -A / 2 + half_line_width;
      marking.end.y = B / 2;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East goal line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - half_line_width;
      marking.start.y = -B / 2;
      marking.end.x = A / 2 - half_line_width;
      marking.end.y = B / 2;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // North touch line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2;
      marking.start.y = B / 2 - half_line_width;
      marking.end.x = A / 2;
      marking.end.y = B / 2 - half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // South touch line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2;
      marking.start.y = -B / 2 + half_line_width;
      marking.end.x = A / 2;
      marking.end.y = -B / 2 + half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2;
      marking.start.y = C / 2 - half_line_width;
      marking.end.x = -A / 2 + E;
      marking.end.y = C / 2 - half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Box - Horizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2;
      marking.start.y = -C / 2 + half_line_width;
      marking.end.x = -A / 2 + E;
      marking.end.y = -C / 2 + half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // West Penalty Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2 + E - half_line_width;
      marking.start.y = -C / 2;
      marking.end.x = -A / 2 + E - half_line_width;
      marking.end.y = C / 2;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - E;
      marking.start.y = C / 2 - half_line_width;
      marking.end.x = A / 2;
      marking.end.y = C / 2 - half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Box - Horizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - E;
      marking.start.y = -C / 2 + half_line_width;
      marking.end.x = A / 2;
      marking.end.y = -C / 2 + half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East Penalty Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - E + half_line_width;
      marking.start.y = -C / 2;
      marking.end.x = A / 2 - E + half_line_width;
      marking.end.y = C / 2;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // West Goal Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2;
      marking.start.y = D / 2 - half_line_width;
      marking.end.x = -A / 2 + F;
      marking.end.y = D / 2 - half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // West Goal Box - Hotizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2;
      marking.start.y = -D / 2 + half_line_width;
      marking.end.x = -A / 2 + F;
      marking.end.y = -D / 2 + half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // West Goal Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = -A / 2 + F - half_line_width;
      marking.start.y = -D / 2;
      marking.end.x = -A / 2 + F - half_line_width;
      marking.end.y = D / 2;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East Goal Box - Horizontal Line (North)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - F;
      marking.start.y = D / 2 - half_line_width;
      marking.end.x = A / 2;
      marking.end.y = D / 2 - half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East Goal Box - Hotizontal Line (South)
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - F;
      marking.start.y = -D / 2 + half_line_width;
      marking.end.x = A / 2;
      marking.end.y = -D / 2 + half_line_width;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // East Goal Box - Vertical Line
    {
      soccer_field_msgs::msg::LineMarking marking;
      marking.start.x = A / 2 - F + half_line_width;
      marking.start.y = -D / 2;
      marking.end.x = A / 2 - F + half_line_width;
      marking.end.y = D / 2;
      marking.line_width = K;
      field.markings.lines.push_back(marking);
    }

    // North West Corner Arc
    {
      soccer_field_msgs::msg::ArcMarking marking;
      marking.center.x = - A / 2;
      marking.center.y = B / 2;
      marking.heading_start = 1.5 * M_PI;
      marking.heading_end = 2.0 * M_PI;
      marking.radius = G - half_line_width;
      marking.line_width = K;
      field.markings.arcs.push_back(marking);
    }

    // North East Corner Arc
    {
      soccer_field_msgs::msg::ArcMarking marking;
      marking.center.x = A / 2;
      marking.center.y = B / 2;
      marking.heading_start = M_PI;
      marking.heading_end = 1.5 * M_PI;
      marking.radius = G - half_line_width;
      marking.line_width = K;
      field.markings.arcs.push_back(marking);
    }

    // South East Corner Arc
    {
      soccer_field_msgs::msg::ArcMarking marking;
      marking.center.x = A / 2;
      marking.center.y = -B / 2;
      marking.heading_start = 0.5 * M_PI;
      marking.heading_end = M_PI;
      marking.radius = G - half_line_width;
      marking.line_width = K;
      field.markings.arcs.push_back(marking);
    }

    // South West Corner Arc
    {
      soccer_field_msgs::msg::ArcMarking marking;
      marking.center.x = - A / 2;
      marking.center.y = - B / 2;
      marking.heading_start = 0.0;
      marking.heading_end = 0.5 * M_PI;
      marking.radius = G - half_line_width;
      marking.line_width = K;
      field.markings.arcs.push_back(marking);
    }

    // West Penalty Spot
    {
      soccer_field_msgs::msg::SpotMarking marking;
      marking.center.x = -A / 2 + I;
      marking.radius = J / 2;
      field.markings.spots.push_back(marking);
    }

    // East Penalty Spot
    {
      soccer_field_msgs::msg::SpotMarking marking;
      marking.center.x = A / 2 - I;
      marking.radius = J / 2;
      field.markings.spots.push_back(marking);
    }

    // Center spot
    {
      soccer_field_msgs::msg::SpotMarking marking;
      marking.radius = J / 2;
      field.markings.spots.push_back(marking);
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
  rclcpp::spin(std::make_shared<soccer_field_publisher::MSLFieldPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
