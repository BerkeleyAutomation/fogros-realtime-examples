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
import io
import socket
import time
from pathlib import Path
from time import sleep, time
from datetime import datetime
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib import pyplot as plt
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from shared_srvs.srv import Sam


def show_anns(masks):
    """based on https://github.com/facebookresearch/segment-anything/blob/main/notebooks/automatic_mask_generator_example.ipynb"""
    if len(masks) == 0:
        return
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((masks[0].shape[0], masks[0].shape[1], 4))
    img[:, :, 3] = 0
    for mask in masks:
        m = np.all(mask == [1, 1, 1], axis=-1)
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    ax.imshow(img)


def print_string_with_color_based_on_name(string, name):
    hash_value = int(hashlib.sha256(name.encode()).hexdigest(), 16)
    # Choose a color based on the hash value
    color_code = hash_value % 8  # You can change the number of colors as needed

    # Define color codes (you can modify this based on your preferences)
    colors = [
        "\033[91m",
        "\033[92m",
        "\033[93m",
        "\033[94m",
        "\033[95m",
        "\033[96m",
        "\033[97m",
        "\033[90m",
    ]

    return colors[color_code] + string + "\033[00m"


class SamClientNode(Node):
    def __init__(self):
        super().__init__("sam_period_async")
        self.get_logger().info(f"Initializing client for /sam.")
        self.cli = self.create_client(Sam, "sam")

        # declare a parameter of bool
        self.declare_parameter("send_on_odd_minute", True)
        self.send_on_odd_minute = self.get_parameter("send_on_odd_minute").value

        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("service not available, waiting again...")
        self.req = Sam.Request()

    def send_request(self, image):
        self.req.image = image

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)

    sam_client = SamClientNode()
    # TODO: right way to get path of src in ros?
    image_path = Path(
        f"{Path(__file__).parent}/../../../../../../src/fogros-realtime-examples/images/ucb-transit.png"
    )

    request_image = CompressedImage()
    request_image.format = "png"
    with Image.open(image_path) as image_file:
        image_file = image_file.convert("RGB")
        byte_io = io.BytesIO()
        image_file.save(byte_io, format="PNG")
        byte_io.seek(0)
        request_image.data = byte_io.read()

    latency = dict()
    latency_timestamp = dict()
    beginning_time = time()

    while True:
        time_start = time()
        if int(datetime.now().minute) % 2 == sam_client.send_on_odd_minute:
            sam_client.get_logger().info(f"I am {host_name} on {host_ip}. Sending request")
            response = sam_client.send_request(request_image)
            sam_client.get_logger().info(
                print_string_with_color_based_on_name(
                    f"Received from {response.server_name}.",
                    response.server_name,
                )
            )
            image = cv2.imdecode(
                np.frombuffer(request_image.data, np.uint8), cv2.IMREAD_COLOR
            )
            masks = [
                cv2.imdecode(np.frombuffer(mask.data, np.uint8), cv2.IMREAD_COLOR)
                for mask in response.masks
            ]
        else:
            sam_client.get_logger().info(
                print_string_with_color_based_on_name(
                    f"Skipping request.",
                    "None",
                )
            )  
            sleep(3)
            continue 


    sam_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
