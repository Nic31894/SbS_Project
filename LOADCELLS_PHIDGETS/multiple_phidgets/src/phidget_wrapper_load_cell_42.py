#!/usr/bin/env python
#Nicole Maugliani 6 May 2020
import rospy
from  phidgets_forces_msgs.msg import forces
from std_msgs.msg import Header
from Phidget22.Net import Net
#import numpy as np
import Phidget22.Devices.VoltageRatioInput as vri
#load cell F90829043
class PhidgetSensor:
    def read_sensor_data_phidgets(self, event=None):
        self.channelValueRawMsg=forces()
        self.channelValueRawMsg.header= Header()
        self.channelValueRawMsg.header.stamp = rospy.Time.now()
#        self.channelValueRawMsg.value_0 = 0 * voltageRatioInput0.getVoltageRatio() # this channel is not linked with anything. 
        
        self.channelValueRawMsg.value_0 = 588927
        self.channelValueRawMsg.Fx =467343* voltageRatioInput1_1.getVoltageRatio() -3.6909
        self.channelValueRawMsg.Fy = 496016*voltageRatioInput2_1.getVoltageRatio() -6.1991
        self.channelValueRawMsg.Fz = 503400*voltageRatioInput3_1.getVoltageRatio() -0.8183
       
        self.publish_data_phidgets()
    def __init__(self):
        # Create a ROS publisher
       self.phi_publisher = rospy.Publisher('load_cell_42', forces, queue_size=10)
    def publish_data_phidgets(self, event=None):
       msg=forces()
       msg.header= self.channelValueRawMsg.header
       msg.header.stamp= self.channelValueRawMsg.header.stamp
       msg.header.seq=self.channelValueRawMsg.header.seq
       msg.header.frame_id=str(588891)
       msg.value_0=self.channelValueRawMsg.value_0
       msg.Fx=self.channelValueRawMsg.Fx
       msg.Fy=self.channelValueRawMsg.Fy
       msg.Fz=self.channelValueRawMsg.Fz       
       self.phi_publisher.publish(msg)
if __name__ == '__main__':
    try:

        Net.addServer("serverName", "127.0.0.1", 5661, "", 0);
        pub = rospy.Publisher('load_cell_42', forces, queue_size=10)
        rospy.init_node('PhidgetsWrapperForces', anonymous=True)

        voltageRatioInput1_1 = vri.VoltageRatioInput()
        voltageRatioInput2_1 = vri.VoltageRatioInput()
        voltageRatioInput3_1 = vri.VoltageRatioInput()
        voltageRatioInput1_1.setDeviceSerialNumber(588927)
        voltageRatioInput2_1.setDeviceSerialNumber(588927)
        voltageRatioInput3_1.setDeviceSerialNumber(588927)

        voltageRatioInput1_1.setChannel(1)
        voltageRatioInput2_1.setChannel(2)
        voltageRatioInput3_1.setChannel(3)
        voltageRatioInput1_1.openWaitForAttachment(5000)
        voltageRatioInput2_1.openWaitForAttachment(5000)
        voltageRatioInput3_1.openWaitForAttachment(5000)
        fr=250
        voltageRatioInput1_1.setDataInterval(fr)
        voltageRatioInput2_1.setDataInterval(fr)
        voltageRatioInput3_1.setDataInterval(fr)
       
        ps= PhidgetSensor()
        fr=float(fr)
        rospy.Timer(rospy.Duration(1.0/fr), ps.read_sensor_data_phidgets)
#        rospy.Timer(rospy.Duration(1.0/fr), ps.publish_data_phidgets)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#        voltageRatioInput0.close()
        voltageRatioInput1_1.close()
        voltageRatioInput2_1.close()
        voltageRatioInput3_1.close()
        