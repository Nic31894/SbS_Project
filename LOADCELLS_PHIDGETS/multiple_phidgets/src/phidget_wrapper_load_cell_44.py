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
        self.channelValueRawMsg.value_0 = 588986
        self.channelValueRawMsg.Fx = 479270*voltageRatioInput1_1.getVoltageRatio()-6.1836
        self.channelValueRawMsg.Fy =479270* voltageRatioInput2_1.getVoltageRatio()-2.4742
        self.channelValueRawMsg.Fz = 468542*voltageRatioInput3_1.getVoltageRatio()-2.4834
        self.publish_data_phidgets()
    def __init__(self):
        # Create a ROS publisher
       self.phi_publisher = rospy.Publisher('load_cell_44', forces, queue_size=10)
    def publish_data_phidgets(self, event=None):
       msg=forces()
       msg.header= self.channelValueRawMsg.header
       msg.header.stamp= self.channelValueRawMsg.header.stamp
       msg.header.seq=self.channelValueRawMsg.header.seq
       msg.header.frame_id=str(588986)
       msg.value_0=self.channelValueRawMsg.value_0
       msg.Fx=self.channelValueRawMsg.Fx
       msg.Fy=self.channelValueRawMsg.Fy
       msg.Fz=self.channelValueRawMsg.Fz       
       self.phi_publisher.publish(msg)
if __name__ == '__main__':
    try:

        Net.addServer("serverName", "127.0.0.1", 5661, "", 0);
        pub = rospy.Publisher('load_cell_44', forces, queue_size=10)
        rospy.init_node('PhidgetsWrapperForces', anonymous=True)
        voltageRatioInput1_1 = vri.VoltageRatioInput()
        voltageRatioInput2_1 = vri.VoltageRatioInput()
        voltageRatioInput3_1 = vri.VoltageRatioInput()
        voltageRatioInput1_1.setDeviceSerialNumber(588986)
        voltageRatioInput2_1.setDeviceSerialNumber(588986)
        voltageRatioInput3_1.setDeviceSerialNumber(588986)
        
# add the serial number of the other phidgets      
    #the serial number should be added in case we have more than one phidget attached.
    #the calling of the serial number should be done for each channel
        voltageRatioInput1_1.setChannel(1)

        voltageRatioInput2_1.setChannel(2)

        voltageRatioInput3_1.setChannel(3)
        
#        voltageRatioInput0.openWaitForAttachment(5000)
        voltageRatioInput1_1.openWaitForAttachment(5000)
        voltageRatioInput2_1.openWaitForAttachment(5000)
        voltageRatioInput3_1.openWaitForAttachment(5000)
        
    #the following steps on frequency are important to control the frequency
    #the value in setDataInterval is >30 && <1000
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
        