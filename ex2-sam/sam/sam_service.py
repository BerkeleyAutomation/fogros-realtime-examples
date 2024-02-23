# Copyright 2022 The Regents of the University of California (Regents)
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
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import hashlib
import random
import socket
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
from sensor_msgs.msg import CompressedImage

from shared_srvs.srv import Sam


def print_string_with_color_based_on_name(string, name):
    
    hash_value = int(hashlib.sha256(name.encode()).hexdigest(), 16)
    # Choose a color based on the hash value
    color_code = hash_value % 8  # You can change the number of colors as needed

    # Define color codes (you can modify this based on your preferences)
    colors = ["\033[91m", "\033[92m", "\033[93m", "\033[94m", "\033[95m", "\033[96m", "\033[97m", "\033[90m"]

    return colors[color_code] + string + "\033[00m"


class SamServiceNode(Node):
    def __init__(self):
        super().__init__("sam")
        # generate random name
        self.host_name = socket.gethostname() + str(random.randint(0, 1000))
        self.get_logger().info(f'I am {self.host_name}. Starting service /sam.')
        self.service = self.create_service(Sam, "sam", self.sam_callback)

        # TODO: right way to get path of src in ros?
        # model_path = Path(f"{Path(__file__).parent}/../../../../../../src/fogros-realtime-examples/models/sam_vit_h_4b8939.pth")
        model_path = "/tmp/sam_vit_h_4b8939.pth"
        self.sam = sam_model_registry["default"](checkpoint=model_path)
        self.sam.to(device="cuda") # comment this line if not have nvidia gpu
        self.sam_mask_generator = SamAutomaticMaskGenerator(self.sam)

    def sam_callback(self, request, response):
        self.get_logger().info(
            print_string_with_color_based_on_name(f'Received request', self.host_name)
        )

        req_image = cv2.imdecode(np.frombuffer(request.image.data, np.uint8), cv2.IMREAD_COLOR)
        self.get_logger().info(
            print_string_with_color_based_on_name(f'Generating masks...', self.host_name)
        )
        masks_output = self.sam_mask_generator.generate(req_image)

        response.masks = []
        response.scores = []
        for m in masks_output:
            compressed_image = CompressedImage()
            compressed_image.data = np.array(cv2.imencode('.png', m['segmentation'].astype(np.uint8))[1]).tostring()
            compressed_image.format = "png"
            response.masks.append(compressed_image)
            response.scores.append(m['stability_score'])

        response.server_name = self.host_name
        self.get_logger().info(
            print_string_with_color_based_on_name(f'Sent response: scores {response.scores}', self.host_name)
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    sam_service_node = SamServiceNode()
    rclpy.spin(sam_service_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sam_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
