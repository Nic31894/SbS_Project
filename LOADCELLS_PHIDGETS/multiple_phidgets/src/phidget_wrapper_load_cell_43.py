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
        
        self.channelValueRawMsg.value_0 = 588962
        self.channelValueRawMsg.Fx = 455308*voltageRatioInput1_1.getVoltageRatio() -5.3694
        self.channelValueRawMsg.Fy = 439705*voltageRatioInput2_1.getVoltageRatio() -3.7751
        self.channelValueRawMsg.Fz = 552840*voltageRatioInput3_1.getVoltageRatio() -6.0949
    
        self.publish_data_phidgets()
    def __init__(self):
        # Create a ROS publisher
       self.phi_publisher = rospy.Publisher('load_cell_43', forces, queue_size=10)
    def publish_data_phidgets(self, event=None):
       msg=forces()
       msg.header= self.channelValueRawMsg.header
       msg.header.stamp= self.channelValueRawMsg.header.stamp
       msg.header.seq=self.channelValueRawMsg.header.seq
       msg.header.frame_id=str(588962)
       msg.value_0=self.channelValueRawMsg.value_0
       msg.Fx=self.channelValueRawMsg.Fx
       msg.Fy=self.channelValueRawMsg.Fy
       msg.Fz=self.channelValueRawMsg.Fz       
       self.phi_publisher.publish(msg)
if __name__ == '__main__':
    try:

        Net.addServer("serverName", "127.0.0.1", 5661, "", 0);
        pub = rospy.Publisher('load_cell_43', forces, queue_size=10)
        rospy.init_node('PhidgetsWrapperForces', anonymous=True)
        
#    rate = rospy.Rate(10)  #200hz
#this function voltageRatioInput() recalls the opening of the channels
 #       voltageRatioInput0 = vri.VoltageRatioInput()
        
        voltageRatioInput1_1 = vri.VoltageRatioInput()
        voltageRatioInput2_1 = vri.VoltageRatioInput()
        voltageRatioInput3_1 = vri.VoltageRatioInput()
        voltageRatioInput1_1.setDeviceSerialNumber(588962)
        voltageRatioInput2_1.setDeviceSerialNumber(588962)
        voltageRatioInput3_1.setDeviceSerialNumber(588962)
        
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
#        voltageRatioInput0.setDataInterval(fr)
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
        