// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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


#ifndef GRASPS_HPP_
#define GRASPS_HPP_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iterator>
#include <vector>
#include "msg.hpp"
#include "planning_functions.hpp"

float max_deviation_from_line = 0.1;

// Class that defines a two finger gripper. It is generally represented as a rectangle,
// with the shorter sides representing the gripper finger
// and the longer sides representing the stroke of the 2 finger gripper.

class TwoFinger
{
  //    Box representation of a gripper
  //          <-------gripper width------->
  // corner_1 _____________________________  corner_2
  //         |                             |              ^
  //         |                             |              |
  //         |                             |       gripper thickness
  //         |                             |              |
  //         |_____________________________|              v
  // corner_4                                corner_3

public:
  /*! \brief Corner 1 of the 2 finger grasp rectangle */
  std::vector<int> corner1;
  /*! \brief Corner 2 of the 2 finger grasp rectangle */
  std::vector<int> corner2;
  /*! \brief Corner 3 of the 2 finger grasp rectangle */
  std::vector<int> corner3;
  /*! \brief Corner 4 of the 2 finger grasp rectangle */
  std::vector<int> corner4;

  /*! \brief Final grasp quality of the chosen grasp */
  int final_gdi;

  /*! \brief  Minimum angle to be considered as zero.
              Prevents errors from very small angles*/
  float min_zero_angle;

  /*! \brief Minimum height(in mm) needed to pinch an object.
             Determine the limits for corner collisions*/
  float min_height_diff_to_grip;

  /*! \brief When Comparing grasp GDI, We only want to include centre GDI
             if the grasp GDI is similar. This parameter determines how close
             should a GDI be to be considered equal*/
  float min_gdi_diff_for_comparison;

  /*! \brief Maximum value of the centre GDI*/
  int centre_gdi_max;

  /*! \brief Intermediate placeholder for the maximum GDI for the current grasp*/
  int potential_gdi_max;

  /*! \brief Centre of grasp rectangle */
  std::vector<int> centre;

  /*! \brief  Table height in mm*/
  float table_height;
  /*! \brief Distance between fingers in pixels */
  float distance_between_fingers;

  /*! \brief Gripper finger thickness in pixels */
  float gripper_thickness;
  /*! \brief Target hypotenuse length of a grasp rectangle*/
  float target_length;

  /*! \brief*/

  /*! \brief
      As the diagonal length of a 2 finger gripper is constant, the
      length between the two points should be equal to that length.
      We give a certain allowable error of the length between 2 chosen points.

      TODO(Glenn):There needs to be a way to change this depending on orientation.
      When it is vertical or horizontal, the allowable error should be lower compared
      to an angled object
  */
  float allowable_length_error;
  /*! \brief Start x coordinates of grasp sampling*/
  std::vector<std::vector<int>> checkpoint_startx;
  /*! \brief Start y coordinates of grasp sampling*/
  std::vector<std::vector<int>> checkpoint_starty;
  /*! \brief End x coordinates of grasp sampling*/
  std::vector<std::vector<int>> checkpoint_endx;
  /*! \brief End y coordinates of grasp_sampling*/
  std::vector<std::vector<int>> checkpoint_endy;
  /*! \brief y-intersect vector of the gradient of the 2 lines that
            represents the bounds of the grasp samples for all objects*/
  std::vector<float> gradient_vector;
  /*! \brief first y-intersect vector of the gradient of the 2 lines that
             represents the bounds of the grasp samples for all objects*/
  std::vector<float> c1_vector;

  /*! \brief second y-intersect vector of the gradient of the 2
      lines that represents the bounds of the grasp samples for all objects */
  std::vector<float> c2_vector;

  /**
   * Constructor for an empty two finger grasp, used usually when there is no grasp
   */
  TwoFinger()
  {
    corner1 = {0, 0};
    corner3 = {0, 0};
    centre = {0, 0};
  }
  /**
   * Constructor for a considered grasp. As the input points represent
     diagonally opposite points for the grasp rectangle, populate point 1 and 3
   */
  TwoFinger(
    float min_zero_angle_, float min_height_diff_to_grip_,
    float min_gdi_diff_for_comparison_, float table_height_,
    float distance, float g_thickness)
  : min_zero_angle(min_zero_angle_),
    min_height_diff_to_grip(min_height_diff_to_grip_),
    min_gdi_diff_for_comparison(min_gdi_diff_for_comparison_),
    table_height(table_height_),
    distance_between_fingers(distance),
    gripper_thickness(g_thickness)
  {
    centre_gdi_max = 0;
    potential_gdi_max = 0;
    // The needed diagonal length for a grasp box, according to gripper paramters.
    target_length = sqrt(pow(distance_between_fingers, 2) + pow(gripper_thickness, 2));
    distance_between_fingers = distance;
    gripper_thickness = g_thickness;
    table_height = table_height_;
    min_gdi_diff_for_comparison = min_gdi_diff_for_comparison_;
    min_zero_angle = min_zero_angle_;
    min_height_diff_to_grip = min_height_diff_to_grip_;
  }

  /**
   * Get points to be sampled for grasp. The points will be along 2 lines
     which are reprenting the maximum ends of each side of the 2F gripper
   */
  void get_checkpoints(Msg msg)
  {
    for (int i = 0; i < msg.num_objects; i++) {       //  Iterate through the detected objects
      std::vector<int> temp_checkpoint_startx;
      std::vector<int> temp_checkpoint_starty;
      std::vector<int> temp_checkpoint_endx;
      std::vector<int> temp_checkpoint_endy;

      // 1 Deg is 0.0174533 rad
      float gradient_perpendicular;
      float gradient;
      int search_area_start_x;
      int search_area_start_y;
      int search_area_end_x;
      int search_area_end_y;
      float c1, c2;
      c1 = 0;
      c2 = 0;

      /* Determine search area for gripping start and end points.
         These search areas will only be used at the end of this function */

      /* If the gripper is bigger than the bounding box,
         the search area needs to be beyond the bounding box points*/

      if (abs(msg.br_y[i] - msg.tl_y[i]) < distance_between_fingers) {
        search_area_start_y = msg.tl_y[i] -
          static_cast<int>(round(
            abs(
              distance_between_fingers -
              abs(msg.br_y[i] - msg.tl_y[i])) / 2));

        search_area_end_y = msg.br_y[i] +
          static_cast<int>(round(
            abs(
              distance_between_fingers -
              abs(msg.br_y[i] - msg.tl_y[i])) / 2));
      } else {            // If not the search box can just be the bounding box
        search_area_start_y = msg.tl_y[i];
        search_area_end_y = msg.br_y[i];
      }
      // If the gripper is bigger than the bounding box,
      // the search area needs to be beyond the bounding box points
      if (abs(msg.br_x[i] - msg.tl_x[i]) < distance_between_fingers) {
        search_area_start_x = msg.tl_x[i] -
          static_cast<int>(round(
            abs(
              distance_between_fingers -
              abs(msg.br_x[i] - msg.tl_x[i])) / 2));


        search_area_end_x = msg.br_x[i] +
          static_cast<int>(round(
            abs(
              distance_between_fingers -
              abs(msg.br_x[i] - msg.tl_x[i])) / 2));
      } else {            // If not the search box can just be the bounding box
        search_area_start_x = msg.tl_x[i];
        search_area_end_x = msg.br_x[i];
      }

      // Ensure that the coordinates are valid within the image viewing space
      if (search_area_start_x < 0) {
        search_area_start_x = 0;
      }
      if (search_area_start_y < 0) {
        search_area_start_y = 0;
      }
      if (search_area_end_x > msg.img_width) {
        search_area_end_x = msg.img_width;
      }
      if (search_area_end_y > msg.img_height) {
        search_area_end_y = msg.img_height;
      }
      // Get the gradient value of the object
      // NOTE:: negative here because the x-y axis of an image is flipped about the x axis
      gradient = -tan(msg.angle[i]);
      // Get the gradient value of the perpendicular line from object
      gradient_perpendicular = -(1 / gradient);
      float c_perpendicular;
      float r = distance_between_fingers / 2;
      int x1, x2, y1, y2;
      // Get y-intercept of perpendicular line
      c_perpendicular = msg.center_y[i] - (gradient_perpendicular) * msg.center_x[i];
      // When the obect is horizontal WRT x axis, angle 0
      if (abs(gradient) < min_zero_angle) {
        allowable_length_error = 1;
        // the object is too close to the edge of the viewing range of the camera
        if (msg.center_y[i] - r < 0) {
          y1 = 0;
        } else {
          y1 = msg.center_y[i] - r;
        }
        y2 = y1 + 2 * r;
        /* For Horizontal objects, search area can be reduced to
           just adding all the x values along the horizontal lines */
        for (int j = search_area_start_x; j < search_area_end_x + 1; j++) {
          if (y2 < msg.img_height) {
            temp_checkpoint_endy.push_back(y2);
          } else {
            temp_checkpoint_endy.push_back(msg.img_height);
          }
          if (j < msg.img_width) {
            temp_checkpoint_endx.push_back(j);
            temp_checkpoint_startx.push_back(j);
          } else {
            temp_checkpoint_endx.push_back(msg.img_width);
            temp_checkpoint_startx.push_back(msg.img_width);
          }

          if (j > 0) {
            temp_checkpoint_endx.push_back(j);
            temp_checkpoint_startx.push_back(j);
          } else {
            temp_checkpoint_endx.push_back(0);
            temp_checkpoint_startx.push_back(0);
          }
          //
          if (y1 >= 0) {
            temp_checkpoint_starty.push_back(y1);
          } else {
            temp_checkpoint_starty.push_back(0);
          }
        }
      } else if (abs(gradient_perpendicular) < min_zero_angle) {
        // object is perpendicular wrt x axis, angle pi/2
        allowable_length_error = 1;
        if (msg.center_x[i] - r < 0) {
          x2 = 0;                // Get First x bound
        } else {
          x2 = msg.center_x[i] - r;
        }
        x1 = x2 + 2 * r;              //  Get Second x bound
        // For Horizontal objects, search area can be reduced to just
        // adding all the y values along the vertical lines
        for (int j = search_area_start_y; j < search_area_end_y + 1; j++) {
          if (j < msg.img_height) {
            temp_checkpoint_endy.push_back(j);
            temp_checkpoint_starty.push_back(j);
          } else {
            temp_checkpoint_starty.push_back(msg.img_height);
            temp_checkpoint_endy.push_back(msg.img_height);
          }
          if (j > 0) {
            temp_checkpoint_endy.push_back(j);
            temp_checkpoint_starty.push_back(j);
          } else {
            temp_checkpoint_starty.push_back(0);
            temp_checkpoint_endy.push_back(0);
          }
          if (x2 < msg.img_width) {
            temp_checkpoint_endx.push_back(x2);
          } else {
            temp_checkpoint_endx.push_back(msg.img_width);
          }
          if (x1 >= 0) {
            temp_checkpoint_startx.push_back(x1);
          } else {
            temp_checkpoint_startx.push_back(0);
          }
        }
      } else {            // If the object is angled
        allowable_length_error = 3;
        /*
            Given a circle with equation (x- centre[0])^2 + (y-centre[1])^2 = r^2, where r = gripper_width/2
            Find the two points where the line y = gradient_perpendicular* x + c intersect
        */

        std::vector<std::vector<int>> intersect_temp;
        intersect_temp = circle_line_intersect(
          gradient_perpendicular,
          c_perpendicular, r,
          {msg.center_x[i], msg.center_y[i]});
        /*
            Problem with using the function to generate the intersect points is that it comes as an int, so the c1 values created are abit truncated,
            compared to if the intersect points were kept as floats
        */

        c1 = intersect_temp[0][1] - gradient * intersect_temp[0][0];
        c2 = intersect_temp[1][1] - gradient * intersect_temp[1][0];
        /*
            Now that we know the two lines that represents the max and min bounds of the gripper.
            Get the points in the search area that are along these lines.
            Assumes y1 = m1x + c1 to be the end corner of the grasps
            Assumes y2 = m2x + c2 to be the end corner of the grasps
        */

        for (int m = search_area_start_y; m < search_area_end_y + 1; m++) {
          for (int j = search_area_start_x; j < search_area_end_x + 1; j++) {
            // Point is on left line
            if (abs(gradient * j + c1 - m) < max_deviation_from_line) {
              // Only include points that are within the image pixel limit
              if (j > 0) {
                temp_checkpoint_startx.push_back(j);
              } else {
                temp_checkpoint_startx.push_back(0);
              }

              if (m > 0) {
                temp_checkpoint_starty.push_back(m);
              } else {
                temp_checkpoint_starty.push_back(0);
              }
            } else if (abs(gradient * j + c2 - m) < max_deviation_from_line) {
              // Only include points that are within the image pixel limit
              if (j < msg.img_width) {
                temp_checkpoint_endx.push_back(j);
              } else {
                temp_checkpoint_endx.push_back(msg.img_width);
              }

              if (m < msg.img_height) {
                temp_checkpoint_endy.push_back(m);
              } else {
                temp_checkpoint_endy.push_back(msg.img_height);
              }
            }
          }
        }
      }

      // Add all required elements for this particular object
      checkpoint_startx.push_back(temp_checkpoint_startx);
      checkpoint_starty.push_back(temp_checkpoint_starty);
      checkpoint_endx.push_back(temp_checkpoint_endx);
      checkpoint_endy.push_back(temp_checkpoint_endy);
      c1_vector.push_back(c1);
      c2_vector.push_back(c2);
      gradient_vector.push_back(gradient);
    }
  }

  /**
   * Calculating the grasp quality of one side of a grasp rectangle
   */
  int get_corner_gdi(
    std::vector<int> centre,
    float radius,
    cv::Mat depth_img,
    float centre_depth)
  {
    int gdi = 0;
    // Returns the top left and bottom right corners of a region
    std::vector<std::vector<int>> centre_corners = get_border_corners(centre, radius);
    for (int i = centre_corners[0][1]; i < centre_corners[1][1]; i++) {
      for (int j = centre_corners[0][0]; j < centre_corners[1][0]; j++) {
        std::vector<int> point{j, i};
        // Get depth at current pixel
        float point_depth = depth_img.at<ushort>(point[1], point[0]);
        if (in_circle(radius, centre, point)) {
          /*
          A grasp is defined as good if the height of the centre of grasp is higher than the sides of grasps. thus for every point at the side that is
          lower than the centre, the gdi increases.
          */
          min_height_diff_to_grip = table_height - centre_depth;
          if (point_depth - centre_depth > min_height_diff_to_grip && point_depth != 0) {
            gdi++;
          } else {
            // No tolerance for any pixel that would collide with centre.
            return 0;
          }
        }
      }
    }
    return gdi;
  }

  /**
   * Generates a Grasp Decide Index value which measures the quality of a grasp approach.
   */
  int get_gdi_value(
    cv::Mat depth_img,
    std::vector<int> target_centre,
    std::vector<int> target_corner1,
    std::vector<int> target_corner2,
    std::vector<int> target_corner3,
    std::vector<int> target_corner4)
  {
    int gdi = 0;
    float radius = 0.5 * gripper_thickness;
    // Rather than using a single centre POINT,
    // we will use an average of the points of the centre REGION
    float centre_depth = 0;
    // Get average depth value of the center region of the grasp
    float total_centre_depth = 0;
    std::vector<std::vector<int>> centre_search_area;
    centre_search_area = get_border_corners(target_centre, radius);
    float num_points = 0;
    for (int centre_y = centre_search_area[0][1];
      centre_y < centre_search_area[1][1]; centre_y++)
    {
      for (int centre_x = centre_search_area[0][0];
        centre_x < centre_search_area[1][0]; centre_x++)
      {
        std::vector<int> point{centre_x, centre_y};
        float point_depth = depth_img.at<ushort>(point[1], point[0]);
        if (point_depth < table_height && point_depth > 1) {           //  Point is on object
          total_centre_depth += point_depth;
          num_points++;
        }
      }
    }
    // If no points in the centre region is on the object,  it is not a legitimate grasp.
    // Will be changed in the future to account for weird shapes (like a torus)
    if (num_points == 0) {
      return 0;
    } else {
      // Getting average depth value of the centre of the object
      centre_depth = total_centre_depth / num_points;
    }
    //  Center of grasp is on the surface of the object.
    if (centre_depth < table_height && centre_depth > 1) {
      /*
      We want to sample the points at the two ends of the grasp rectangle, along the
      longer axis of the rectangle. Thus we need to check if 1-4 or 1-2 is the short
      side of the rectangle.
      */

      std::vector<int> centre1;
      std::vector<int> centre2;

      if (abs(
          length(
            target_corner1[0], target_corner1[1],
            target_corner2[0], target_corner2[1]) - gripper_thickness) >

        abs(
          length(
            target_corner1[0], target_corner1[1],
            target_corner4[0], target_corner4[1]) - gripper_thickness))
      {
        // Get the midpoints of the short edges
        centre1.push_back(static_cast<int>((target_corner1[0] + target_corner4[0]) / 2));
        centre1.push_back(static_cast<int>((target_corner1[1] + target_corner4[1]) / 2));
        centre2.push_back(static_cast<int>((target_corner3[0] + target_corner2[0]) / 2));
        centre2.push_back(static_cast<int>((target_corner3[1] + target_corner2[1]) / 2));
      } else {
        // Get the midpoints of the short edges
        centre1.push_back(static_cast<int>((target_corner1[0] + target_corner2[0]) / 2));
        centre1.push_back(static_cast<int>((target_corner1[1] + target_corner2[1]) / 2));
        centre2.push_back(static_cast<int>((target_corner3[0] + target_corner4[0]) / 2));
        centre2.push_back(static_cast<int>((target_corner3[1] + target_corner4[1]) / 2));
      }

      // Iterate through the search area to find points within the grasp rectangle,
      // nearer to the two ends of the rectangle.

      int centre1_gdi = get_corner_gdi(centre1, radius, depth_img, centre_depth);
      int centre2_gdi = get_corner_gdi(centre2, radius, depth_img, centre_depth);
      //  Make sure both fingers are collision free
      if (centre1_gdi > 0 && centre2_gdi > 0) {
        gdi = centre1_gdi + centre2_gdi;
      }
    }
    return gdi;
  }

  /**
   * Given the coordinates of 2 corners of a grasp, find the coordinates of the other two corners of grasp
   */
  std::vector<std::vector<int>> find_2_points(
    std::vector<float> potential_corner1,
    std::vector<float> potential_corner2,
    int item_num)
  {
    //    Box representation of a gripper
    //          <-------gripper width------->
    // corner_1 _____________________________
    //         |                             |              ^
    //         |                             |              |
    //         |                             |       gripper thickness
    //         |                             |              |
    //         |_____________________________|              v
    //                                         corner_2
    std::vector<std::vector<int>> points;
    std::vector<std::vector<int>> points_1;
    std::vector<std::vector<int>> points_2;
    /*
        For a circle with center at one known corner, and radius gripper_thickness, find
        the intersection between the circle and each line

        Each line intersects at 2 points, which are 2 candidates for one of the new corner point in a 2F gripper.
        To find out which is the right point, the distance between that point and the second known corner
        should be equal to the gripper width. THe point closer to this dimension is the next corner

        Given there are 2 lines, each line containing 1 known point, and
        we need to find 2 new points, each on each line, this process is done twice.
    */

    // When the object makes an angle of 0 radians wrt x axis, gradient is ~ 0
    if (abs(gradient_vector[item_num]) < min_zero_angle) {
      /*
          For a object with 0 rad wrt x axis, the 2 lines are horizontal.
          The points intersected by the circle will be the x offset of magnitude <gripper thickness>
          y coordinate of the new point is constant since it is horizontal.
      */
      points_1.push_back(
        {static_cast<int>(round(
            potential_corner1[0] -
            gripper_thickness)),
          static_cast<int>(round(potential_corner1[1]))});

      points_1.push_back(
        {static_cast<int>(round(
            potential_corner1[0] +
            gripper_thickness)),
          static_cast<int>(round(potential_corner1[1]))});

      points_2.push_back(
        {static_cast<int>(round(
            potential_corner2[0] -
            gripper_thickness)),
          static_cast<int>(round(potential_corner2[1]))});

      points_2.push_back(
        {static_cast<int>(round(
            potential_corner2[0] +
            gripper_thickness)),
          static_cast<int>(round(potential_corner2[1]))});

    } else if (abs(1 / gradient_vector[item_num]) < min_zero_angle) {
      /*
          For a object with pi/2 rad wrt x axis, the 2 lines are vertical.
          the points intersected by the circle will be the y offset of magnitude <gripper thickness>
          x coordinate of the new point is constant since it is vertical.
      */

      points_1.push_back(
        {static_cast<int>(round(potential_corner1[0])),
          static_cast<int>(round(
            potential_corner1[1] -
            gripper_thickness))});

      points_1.push_back(
        {static_cast<int>(round(potential_corner1[0])),
          static_cast<int>(round(
            potential_corner1[1] +
            gripper_thickness))});

      points_2.push_back(
        {static_cast<int>(round(potential_corner2[0])),
          static_cast<int>(round(
            potential_corner2[1] -
            gripper_thickness))});
      points_2.push_back(
        {static_cast<int>(round(potential_corner2[0])),
          static_cast<int>(round(
            potential_corner2[1] +
            gripper_thickness))});

    } else {          // For any other angle, a more complicated approach is needed.
      points_1 = circle_line_intersect(
        gradient_vector[item_num], c1_vector[item_num],
        gripper_thickness, potential_corner1);
      points_2 = circle_line_intersect(
        gradient_vector[item_num], c2_vector[item_num],
        gripper_thickness, potential_corner2);
    }

    if (!points_1.empty() && !points_2.empty()) {
      /*
          For point derived with gripper width, the distance between the new points in
          line 1 and the known point in line 2 should be equal to distance_between_fingers

          The distance for each of the new point in line 1 will be compared with gripper width,
          the one with the smallest difference is the right point
      */

      if (abs(
          length(
            potential_corner2[0], potential_corner2[1],
            points_1[0][0], points_1[0][1]) - distance_between_fingers) <
        abs(
          length(
            potential_corner2[0], potential_corner2[1],
            points_1[1][0], points_1[1][1]) - distance_between_fingers))
      {
        points.push_back(points_1[0]);
      } else {
        points.push_back(points_1[1]);
      }
      if (abs(
          length(
            potential_corner1[0], potential_corner1[1],
            points_2[0][0], points_2[0][1]) - distance_between_fingers) <
        abs(
          length(
            potential_corner1[0], potential_corner1[1],
            points_2[1][0], points_2[1][1]) - distance_between_fingers))
      {
        points.push_back(points_2[0]);
      } else {
        points.push_back(points_2[1]);
      }
    } else {
      // The line is a tangent, cannot generate any corner
      return points;
    }

    // Negative coordinates are not valid
    if (points[0][0] < 0 || points[0][1] < 0 || points[1][0] < 0 || points[1][1] < 0) {
      points.clear();
      return points;
    }
    return points;
  }

  /**
   * Samples all possible grasps from a point and chooses the best one
   */
  int best_grasp_from_point(
    std::vector<int> point,
    Msg msg,
    cv::Mat depth_values,
    int item_num,
    int curr_max_gdi)
  {
    int gdi = curr_max_gdi;
    std::vector<int> potential_corner1;
    std::vector<int> potential_corner2;
    std::vector<int> potential_corner3;
    std::vector<int> potential_corner4;
    std::vector<int> potential_centre;

    // iterate through the points on the end line
    for (int i = 0; i < static_cast<int>(checkpoint_endx[item_num].size()); i++) {
      std::vector<int> curr_point = {checkpoint_endx[item_num][i],
        checkpoint_endy[item_num][i]};

      // Find out actual length between the 2 points considered.
      // It should be equal to the target diagonal length, with some margin of error

      float curr_diagonal_length = length(
        curr_point[0], curr_point[1],
        point[0], point[1]);

      // The diagonal length is sufficient as a grasp sample
      if (abs(curr_diagonal_length - target_length) < allowable_length_error) {
        potential_corner1 = point;
        potential_corner3 = curr_point;
        potential_centre = {static_cast<int>(round((point[0] + curr_point[0]) / 2)),
          static_cast<int>(round((point[1] + curr_point[1]) / 2))};

        std::vector<float> temp_point{static_cast<float>(point[0]),
          static_cast<float>(point[1])};

        std::vector<float> temp_point2{static_cast<float>(curr_point[0]),
          static_cast<float>(curr_point[1])};

        // Get the other 2 points for the 2F grasp (this forms a rectangle)
        std::vector<std::vector<int>> get_points = find_2_points(
          temp_point,
          temp_point2,
          item_num);
        // if the other two points can be generated
        if (!get_points.empty()) {
          // Assign all points to the grasp
          potential_corner2 = get_points[0];
          potential_corner4 = get_points[1];

          // Get the grasp quality of the grasp,
          int potential_gdi = get_gdi_value(
            depth_values, potential_centre,
            potential_corner1, potential_corner2,
            potential_corner3, potential_corner4);

          /*
          Another consideration to add into GDI is the distance the centre the grasp is to the center of the bounding box.
          The magnitude of this GDI is highest when exactly in the centre of the x and y axis.
          */
          float center_x_diff = abs(potential_centre[0] - msg.center_x[item_num]);
          float center_y_diff = abs(potential_centre[1] - msg.center_y[item_num]);
          int center_gdi = round(msg.bb_height[item_num] - center_y_diff) +
            round(msg.bb_width[item_num] - center_x_diff);

          // If this current grasp is similar to the previous best grasp
          if (abs(potential_gdi - potential_gdi_max) < min_gdi_diff_for_comparison &&
            potential_gdi > 0)
          {
            // compare the centers. if the current grasp is closer to the centre,
            // choose this grasp.
            if (center_gdi > centre_gdi_max) {
              corner1 = potential_corner1;
              corner2 = potential_corner2;
              corner3 = potential_corner3;
              corner4 = potential_corner4;
              centre = potential_centre;
              centre_gdi_max = center_gdi;
              potential_gdi_max = potential_gdi;
              gdi = centre_gdi_max + potential_gdi_max;
            }
            // Current sampled grasp is too different from the best grasp
          } else {
            // Current grasp is better than the best grasp
            if (potential_gdi > potential_gdi_max) {
              corner1 = potential_corner1;
              corner2 = potential_corner2;
              corner3 = potential_corner3;
              corner4 = potential_corner4;
              centre = potential_centre;
              centre_gdi_max = center_gdi;
              potential_gdi_max = potential_gdi;
              gdi = centre_gdi_max + potential_gdi_max;
            }
          }
        }
      }
    }
    return gdi;
  }

  /**
   * Iterate through all points to generate the best possible grasp
   */
  bool get_best_grasp(Msg msg, cv::Mat depth_values, int item_num)
  {
    int curr_gdi = 0;
    std::cout << " [easy_manipulation_deployment][Grasp Planner] Number of Grasp Samples: " <<
      static_cast<int>(checkpoint_starty[item_num].size()) << std::endl;
    for (int i = 0; i < static_cast<int>(checkpoint_starty[item_num].size()); i++) {
      std::vector<int> point{checkpoint_startx[item_num][i],
        checkpoint_starty[item_num][i]};
      int point_gdi = best_grasp_from_point(point, msg, depth_values, item_num, curr_gdi);
      curr_gdi = point_gdi;
      if (static_cast<int>(checkpoint_starty[item_num].size()) > 100) {
        i++;
      }
    }
    final_gdi = curr_gdi;

    if (curr_gdi == 0) {       // No suitable grasps
      return false;
    } else {
      return true;
    }
  }
};

// Class that defines a suction cup array. We assume that each suction cup is equal in size.
// Minimum array size is one suction cup. array shapes
// will be in a rectangle, of n * m number of cups

class SuctionCupArray
{
public:
  /*! \brief number of suction cups in the length dimension */
  int length_cup_num;
  /*! \brief number of suction cups in the lbreadth dimension */
  int breadth_cup_num;
  /*! \brief Radius of each suction cup */
  float radius;
  /*! \brief Area of each suction cup */
  float cup_area;
  /*! \brief Total area the suction cup array covers */
  float array_area;
  /*! \brief Table height in mm */
  float table_height;
  /*! \brief Total length of array */
  float total_length;
  /*! \brief Total breadth of array */
  float total_breadth;
  /*! \brief length of gap between a the suction cups of an array in the length dimension */
  float breadth_gap = 0;    //  currently hardcoded until suction array supported
  /*! \brief length of gap between a the suction cups of an array in the breadth dimension */
  float length__gap = 0;    // currently hardcoded until suction array supported
  /*! \brief Coordinates of the chosen grasp */
  std::vector<int> chosen_grasp;

  /*! \brief Grasp  */
  int gdi;

  /**
  * Empty Suction cup array constructor
  */
  SuctionCupArray()
  {
    gdi = 0;
    length_cup_num = 0;
    breadth_cup_num = 0;
    radius = 0;
    table_height = 0;

    cup_area = PI * pow(radius, 2);
    array_area = (length_cup_num * 2 * radius) * (breadth_cup_num * 2 * radius);
    total_length = radius * 2 * length_cup_num + length__gap * (length_cup_num - 1);
    total_breadth = radius * 2 * breadth_cup_num + breadth_gap * (breadth_cup_num - 1);
  }

  /**
   * Suction cup array constructor
   */
  SuctionCupArray(int length_cup_num_, int breadth_cup_num_, float radius_, float table_height_)
  {
    gdi = 0;
    length_cup_num = length_cup_num_;
    breadth_cup_num = breadth_cup_num_;
    radius = radius_;
    table_height = table_height_;

    cup_area = PI * pow(radius, 2);
    array_area = (length_cup_num * 2 * radius) * (breadth_cup_num * 2 * radius);
    total_length = radius * 2 * length_cup_num + length__gap * (length_cup_num - 1);
    total_breadth = radius * 2 * breadth_cup_num + breadth_gap * (breadth_cup_num - 1);
  }

  /*
   * Get the Grasp Decide Index of a single suction cup in a suction cup array
  */
  int get_cup_gdi(Msg msg, cv::Mat depth_img, std::vector<int> point, int item_num)
  {
    // Get the number of points that should be on an object surface
    int target_points = round(PI * pow(radius, 2));
    int points_on_surface = 0;

    std::vector<std::vector<int>> new_points = get_border_corners(point, radius);
    for (int i = new_points[0][0]; i < new_points[1][0]; i++) {
      for (int j = new_points[0][1]; j < new_points[1][1]; j++) {
        float point_depth = depth_img.at<ushort>(j, i);
        if (point_depth < table_height && point_depth > 1) {
          points_on_surface++;
          /* TODO(glenn): Another consideration to add into GDI is
          the distance the centre the grasp is to the center of the bounding box.
          // The magnitude of this GDI is highest
          when exactly in the centre of the x and y axis.*/
        }
      }
    }
    if (target_points - points_on_surface < 5) {
      int center_x_diff = round(abs(point[0] - msg.center_x[item_num]));
      int center_y_diff = round(abs(point[1] - msg.center_y[item_num]));
      return round(msg.bb_height[item_num] / 2 - center_y_diff) +
             round(msg.bb_width[item_num] / 2 - center_x_diff);
    } else {
      return 0;
    }
  }

  /**
   * Get the coordinates of the best grasp for the suction cup array.
   * Currently used for single suction cup.
   */
  bool get_best_grasp(Msg msg, cv::Mat depth_img, int item)
  {
    int max_gdi = 0;

    // For a suction cup to properly grasp any surface, it needs to be totally encompassed.
    // For a bounding box, the minium buffer within the box is the radius of one suction cup

    float buffer = radius;
    for (int i = msg.tl_x[item] + buffer; i < msg.tr_x[item] - buffer; i++) {
      // Iterate through every point in the bounding box.
      // Alternative is to iterate through every point + radius. might be faster
      // Another less better alternative is to iterate through the centre axis line.
      // May not be good if the object is weirdly shaped
      for (int j = msg.tl_y[item] + buffer; j < msg.br_y[item] - buffer; j++) {
        std::vector<int> temp_point = {i, j};
        int gdi_point = get_cup_gdi(msg, depth_img, temp_point, item);
        if (max_gdi < gdi_point) {
          chosen_grasp.clear();
          chosen_grasp.push_back(i);
          chosen_grasp.push_back(j);
          max_gdi = gdi_point;
        }
      }
    }
    if (chosen_grasp.empty()) {
      return false;
    } else {
      gdi = max_gdi;
      return true;
    }
  }
};

#endif  // GRASPS_HPP_
