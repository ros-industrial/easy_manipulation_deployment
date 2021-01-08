# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import cv2 as cv2
import os


class GraspValidator():

    def validate_grasps(self):
        path = os.getcwd()
        print(path)

        os.chdir("src/easy_manipulation_deployment/grasp_validator")

        path = os.getcwd()
        print(path)

        img = cv2.imread("images/object.jpg")

        path = os.getcwd()
        print(path)

        f = open("results/grasps.txt", "r")
        path = os.getcwd()
        print(path)
        for x in f:
            y = x.split('\n')
            nums = y[0].split(',')
            if(len(nums) == 3):
                (cv2.circle(
                    img, (int(nums[0]), int(nums[1])),
                    int(nums[2]), (255, 0, 0), thickness=-1))

            if(len(nums) == 8):
                (cv2.line(
                    img, (int(nums[0]), int(nums[1])),
                    (int(nums[2]), int(nums[3])),
                    (255, 0, 0), 2))

                (cv2.line(
                    img, (int(nums[2]), int(nums[3])),
                    (int(nums[4]), int(nums[5])),
                    (0, 255, 255), 2))

                (cv2.line(
                    img, (int(nums[4]), int(nums[5])),
                    (int(nums[6]), int(nums[7])),
                    (0, 0, 255), 2))

                (cv2.line(
                      img, (int(nums[6]), int(nums[7])),
                      (int(nums[0]), int(nums[1])),
                      (255, 0, 255), 2))

        print("Press any key to exit....")
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    grasp_validator = GraspValidator()
    grasp_validator.validate_grasps()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
