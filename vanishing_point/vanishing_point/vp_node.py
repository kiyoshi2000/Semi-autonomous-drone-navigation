#!/usr/bin/env python
#
# Software License Agreement (BSD)
#
# \file      vp_node.py
# \authors   Jeremy Fix <jeremy.fix@centralesupelec.fr>
# \copyright Copyright (c) 2022, CentraleSupélec, All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#  * Neither the name of Autonomy Lab nor the names of its contributors may be
#    used to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WAR- RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, IN- DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# External imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from example_interfaces.msg import Float32  # std_msgs.msg.Float32 is deprecated
from cv_bridge import CvBridge
import cv2
import numpy as np


global vpa 
vpa = (0,0)

# Local imports

from vanishing_point import line_lib_test as mll


class VPNode(Node):
    def __init__(self):

        super().__init__("vp_node")

        self.lsd = cv2.createLineSegmentDetector(0)
        
        self.min_length = 50
        init_h_angle = 20
        self.h_cos_tol = np.cos(init_h_angle*np.pi/180.0)
        self.up = np.array([[0],[1]])
        self.ceiling = 200
        

        self.debug_pub = self.create_publisher(

            CompressedImage, "/debug/vpimg/image_raw/compressed", 1


        ) 

        
        #TODO: Define here your additional publishers
        # The publishers must always be defined before the subscribers
        # using them
        self.hor =self.create_publisher(Float32,"vp_offset",10)
        self.an=self.create_publisher(Float32,"vp_angle",10)
        
        self.img_sub = self.create_subscription(
            
            CompressedImage, "camera/rgb/image_raw/compressed", self.on_image, 1
        )

        self.bridge = CvBridge()

    def cb_params(self, data):

        self.get_logger().info(f"{data}")
        for p in data:
            name = p.name
            value = p.value
            setattr(self.vp_processor, name, value)
        return SetParametersResult(successful=True)

    def on_image(self, msg):

        # Extract and decode the frame from the ROS2 message
        # frame is of type ndarray
        # and of shape H , W , 3
        # encoded in Blue Green Red (BGR)

        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        self.get_logger().info(
            f"Receiving video frame {frame.shape}, of type : {type(frame)}"

        )


        # TODO
        # Let us do some dummy operations on the image just to illustrate
        # the manipulation of the image and a publisher
        # You obvisouly have to adapt/remove this dummy code
        # Our dummy operation is inverting the colors

        if self.debug_pub.get_subscription_count() > 0:

            # frame = 255 - frame
            # ao inves de inverter as cores printar o vanishing point

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            lines = self.lsd.detect(gray)[0]
            lines = mll.from_lsd(lines)

            # We display the ceiling value (a horizontal line)
            cv2.line(frame, (0, self.ceiling), (frame.shape[1], self.ceiling), (0, 0, 200), 1)

            # We filter out the short segments
            lines = lines[lines[...,7] > self.min_length]
            # We display in dark yellow the remaining lines.
            #mll.draw_lines(frame, lines, (20, 100, 100), 1)

            # We filter out the segments from which one point is above (i.e. numerically below) the ceiling.
            lines = lines[np.logical_and(lines[..., 1] > self.ceiling, lines[..., 3] > self.ceiling)]

            # We filter out the lines which are too horizontal.unbalance_two_angles(
            
            #mll.draw_lines(frame, lines, (50, 255, 255), 1)

            nlinhas = mll.filterlines(lines)
            mll.draw_lines(frame,nlinhas,(0,255,0),1)
            ps = mll.intersections(nlinhas)
            #mll.draw_intersections(frame,ps)
            
            global vpa
            vp = mll.Vanishing_point(frame,ps,vpa)
            #print("vp",vp)
            (r,t) = vp
            r = int(r)
            t = int(t)
            vpa = (r,t)
            
            cv2.circle(frame,(r,t),3,(255,255,255),-1)

            x_p = int((frame.shape[1])/2)
            y_p = int((frame.shape[0])/2)
            
            
            #cv2.circle(frame,(x_p,y_p),1,(0,255,0),1) #center point printed on image


            # And drawing a line
            cv2.line(frame, (0, 0), (100, 100), (255, 0, 0), 5)


            outmsg = self.bridge.cv2_to_compressed_imgmsg(frame.copy())

            

            self.debug_pub.publish(outmsg)

            horizontal_offset_msg = Float32()
            unbalance_msg = Float32()


            if vp is not None:
                horizontal_offset_msg.data = mll.horizontal_misplacement(vp,0,frame.shape[1])
                unbalance_msg.data = mll.unbalance_two_angles(frame,vp)
                self.hor.publish(horizontal_offset_msg)
                self.an.publish(unbalance_msg)


        


def main(args=None):

    rclpy.init(args=args)

    vp_node = VPNode()
    rclpy.spin(vp_node)

    vp_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
