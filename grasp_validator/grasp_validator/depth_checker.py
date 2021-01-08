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


class DepthChecker():

    def check_depth(self):
        path = os.getcwd()
        print(path)

        os.chdir("src/easy_manipulation_deployment/grasp_validator")

        path = os.getcwd()
        print(path)

        img = cv2.imread("images/depth_img.jpg")

        path = os.getcwd()
        print(path)
        f = open("results/object_coordinates.txt", "r")
        path = os.getcwd()
        print(path)
        for x in f:
            y = x.split('\n')
            nums = y[0].split(',')
            img[int(nums[1]), int(nums[0])] = (0, 255, 0)

        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        os.chdir("images")
        print(path)
        cv2.imwrite('object.jpg', img)


def main(args=None):
    rclpy.init(args=args)
    depth_checker = DepthChecker()
    depth_checker.check_depth()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
