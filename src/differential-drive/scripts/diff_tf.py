#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import rospy
import roslib
import tf
import time

import tf2_ros
import tf2_geometry_msgs

import geometry_msgs.msg

from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16
from std_msgs.msg import Int32

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 50))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.245)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        self.wheel_frame_id = rospy.get_param('~wheel_frame_id', 'chassis_link') # the name of wheel frame
        
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32767)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )

        self.publish_tf = rospy.get_param('~publish_tf', True)
        rospy.loginfo("Parameter: publish_tf set to: {}".format(self.publish_tf))
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # Add a tf listener so we can transforms to the base link
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()

        # The offset frame self.x2 = 0
        self.y2 = 0
        
        # subscriptions
        rospy.Subscriber("lwheel", Int32, self.lwheelCallback, queue_size=1)
        rospy.Subscriber("rwheel", Int32, self.rwheelCallback, queue_size=1)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()



        # Odom covariances. I'm not going to be fusing these.
        covar_x = 0.1
        covar_y = 0.1
        covar_z = 1

        # roll pitch yaw
        covar_roll = 1e6
        covar_pitch = 1e6
        covar_yaw = 0.3

        # These are the covariances that matter, xdot, ydot and yawdot.                         
        covar_twist_x = 0.01
        covar_twist_y = 0.01
        covar_twist_z = 1e6

        covar_twist_roll = 1e6;
        covar_twist_pitch = 1e6;
        covar_twist_yaw = 0.025;
        

        self.ODOM_POSE_COVARIANCE = [covar_x, 0, 0, 0, 0, 0, 
                                     0, covar_y, 0, 0, 0, 0,
                                     0, 0, covar_z, 0, 0, 0,
                                     0, 0, 0, covar_roll, 0, 0,
                                     0, 0, 0, 0, covar_pitch, 0,
                                     0, 0, 0, 0, 0, covar_yaw]
  
  
        self.ODOM_TWIST_COVARIANCE = [covar_twist_x, 0, 0, 0, 0, 0, 
                                      0, covar_twist_y, 0, 0, 0, 0,
                                      0, 0, covar_twist_z, 0, 0, 0,
                                      0, 0, 0, covar_twist_roll, 0, 0,
                                      0, 0, 0, 0, covar_twist_pitch, 0,
                                      0, 0, 0, 0, 0, covar_twist_yaw]
        

    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
   
        while True:
            try:
                self.trans = self.tfBuffer.lookup_transform(self.wheel_frame_id, 
                        self.base_frame_id, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                rospy.logwarn("diff_tf transform exception")
                time.sleep(1)


        rospy.loginfo("""Received the Transform 
                         ========================= 
                         Trans x: %f\t Trans y: %f\t Trans z: %f """ %(
                    self.trans.transform.translation.x,
                    self.trans.transform.translation.y,
                    self.trans.transform.translation.z)
                    );

        self.x_trans = self.trans.transform.translation.x
   
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities

            # Guard against elapsed being 0
            if elapsed == 0:
                elapsed = 0.000000001

            # Velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
            # This if velocity in the y direction of the base link that is 
            # present if there is an offset from the wheel center to the base_link
            # center. It is the y velocity induced on the body through angular
            # velocity
            self.dy = sin(self.dr)*self.x_trans
           
             
            if (d != 1):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            # build the Qauternion
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )



            # Convert the frame!

            dx = cos(self.th)*self.x_trans
            self.x2 = self.x + dx

            dy = sin(self.th)*self.x_trans
            self.y2 = self.y + dy

            # rospy.loginfo("dx %f dy %f" % (dx, dy))
            # rospy.loginfo(" x: %f \t y: %f \tth: %f" % (self.x, self.y, self.th))
            # rospy.loginfo("x2: %f \ty2: %f \tth: %f" % (self.x2, self.y2, self.th))


            # Publish the transform  
            if self.publish_tf:
                self.odomBroadcaster.sendTransform(
                    (self.x2, self.y2, 0),
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    self.base_frame_id,
                    self.odom_frame_id
                    )

            # Send over the odom
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x2
            odom.pose.pose.position.y = self.y2
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = self.dy
            odom.twist.twist.angular.z = self.dr
            
            # Build the covariance matrix
        odom.pose.covariance =self.ODOM_POSE_COVARIANCE                
            odom.twist.covariance =self.ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)


    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()
