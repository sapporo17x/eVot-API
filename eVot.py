# eVot API
from time import *
import os
import sys
from math import degrees, pi
import glob
import serial
import threading, thread
from Locator_EKF import Locator_EKF


if os.name == 'nt':
    try:
        import _winreg as winreg
    except:
        pass

class SafeSerial(serial.Serial):
    def __init__(self, *args, **kws):
        lock = kws.pop("lock",None)
        if isinstance(lock,thread.LockType):
            self.lock = lock
        else:
            self.lock = threading.Lock()
        super(SafeSerial, self).__init__(*args,**kws)
        return

    def readline(self):
        with self.lock:
            m = super(SafeSerial, self).readline()
        return m

    def write(self, *args,**kws):
        with self.lock:
            m = super(SafeSerial, self).write(*args,**kws)
        return m

    def flushInput(self):
        with self.lock:
            m = super(SafeSerial, self).flushInput()
        return m

    def flushOutput(self):
        with self.lock:
            m = super(SafeSerial, self).flushOutput()
        return m


class eVot:
    def __init__(self, pos=(0.,0.), heading=0., lock=None):
        self.sonarValues = [0, 0, 0, 0, 0, 0]
        self.all_Values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.port = None
        self.serialReady = False
        self.ldrvalue = [0, 0]
        self.p_value = [0, 0]
        self.acc_values = [0, 0, 0, 0, 0, 0]
        self.pos_values = [0, 0, 0]
        self.EKF = Locator_EKF(pos, heading, 0.1)
        self.updating = False
##        self.offset = False
        self.gyro_heading = degrees(heading)
##        self.offset_counter_iteration = 100
        self.lock = lock
        # simo add
        self.faults = 0
        return

    def destroy(self):
        """
        Destructor function for eBot class.
        """
        self.disconnect()
        self.sonarValues = None
        self.port = None
        self.serialReady = None

    def getOpenPorts(self):
        """
        Windows only function: Obtains a list of tuples with eBot-relevant port number and description.

        :rtype: list
        :return: devicePorts: list of port numbers and descriptions of relevant serial devices.
        """
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        ports = []
        #maximum 256 entries, will break anyways
        for i in range(256):
            try:
                val = winreg.EnumValue(key, i)
                port = (str(val[1]), str(val[0]))
                ports.append(port)
            except Exception:
                winreg.CloseKey(key)
                break
        devicePorts = []
        for port in ports:
            #Just because it is formatted that way...
            if 'BthModem' in port[1][8:] or 'VCP' in port[1][8:] or 'ProlificSerial' in port[1][8:]:
                devicePorts.append((int(port[0][3:]) - 1))
        return devicePorts

    def open(self):
        """
        Opens connection with the eBot via BLE. Connects with the first eBot that the computer is paired to.

        :raise Exception: No eBot found
        """
        self.connect()

    def connect(self, port_path=None):
        """
        Opens connection with the eBot via BLE. Connects with the first eBot that the computer is paired to.

        :raise Exception: No eBot found
        """
        baudRate = 115200
        if port_path:
            ports = [port_path]
        else:
            ports = []
            if os.name == "posix":
                if sys.platform == "linux2":
                    ports = glob.glob('/dev/rfcomm*')
                elif sys.platform == "darwin":
                    ports = glob.glob('/dev/tty.eBo*')
                else:
                    sys.stderr.write("Unknown posix OS.")
                    sys.exit()
            elif os.name == "nt":
                ports = self.getOpenPorts()

        connect = 0
        line = "a"
        print "# Connecting",
        for port in ports:
            print ".",
            try:
                if (line[:2] == "&;"):
                    break
                s = SafeSerial(port, baudRate, timeout=1.0, writeTimeout=1.0, lock=self.lock)
                s.flushInput()
                s.flushOutput()
                strikes = 3
                while line[:2] != "&;" and strikes>0:
                    strikes-=1
                    s.write("<<&?")
                    sleep(0.5)
                    line = s.readline()
                    #print line[:2]
                if line[:2] == "&;":
                    #print "dentro"    
                    connect = 1
                    self.port = s
                    self.portName = port
                    self.port.flushInput()
                    self.port.flushOutput()
                else:
                    s.close()
            except:
                try:
                    s.close()
                except:
                    pass

        if (connect == 0):
            try:
                s.close()
            except:
                pass
            raise Exception("No Evot found")

        try:
####            simo
##            self.port.write('<<1E')
##            sleep(0.4)
##            line = self.port.readline()
##            # print line
##            if (line != ">>1B\n" and line != ">>1B" and line != ">>\n" and line != ">>"):
##            # if (line != "EBOTDEMO\n"):
##                #print "quiiiiii"
##                self.lostConnection()
##            self.port.write("<<1O")
##            sleep(0.4)
##            self.port.write("F")
##            sleep(0.2)
##            self.port.flushInput()
##            self.port.flushOutput()
            print "Done"
##            self.serialReady = True
        except:
            sys.stderr.write("Could not write to serial port.\n")
            self.serialReady = False
            sys.stderr.write("Robot turned off or no longer connected.\n")
        self.start_update_background()
        return

    def start_update_background(self):
        if not self.updating:
            self.update_thread = threading.Thread(target= self.update_background)
            self.updating = True
            self.update_thread.start()
            print "Start Update Background\n"
        return

    def stop_update_background(self):
        self.updating = False
        if self.update_thread.is_alive():
            self.update_thread.join(5)
        if self.update_thread.is_alive():
            raise Exception("eVot: Could not stop update_background thread properly")
        return

    def read_all(self, num_fields = 16):
        ## Uncomment this if request-based data
        ## transfer is implemented
        #if self.serialReady:
        #    try:
        #       self.port.write("2S")
        #    except:
        #        self.lostConnection()
        #line = self.port.readline()
        
####        data[0] ----> TimeStamp;        data[1] ----> Current;
####        data[2] ----> Voltage;          data[3] ----> IR_L;
####        data[4] ----> IR_C;             data[5] ----> IR_R;
####        data[6] ----> IMU_disp_x [mm/s^2];       data[7] ----> IMU_disp_y [mm/s^2];
####        data[8] ----> Yaw [Degree];              data[9] ----> Opt_flow_x;
####        data[10] ----> Opt_flow_y;      data[11] ----> Opt_flow_Yaw;
####        data[12] ----> Encoder_x        data[13] ----> Encoder_y;
####        data[14] ----> Motor_L [PWM Ratios];         data[15] ----> Motor_R [PWM Ratios]
        
        
        data_ready = False
        data = []
        line = None
        while self.port.inWaiting()>70: # one message now is 105 chars long
            line=self.port.readline()
            data_ready = True
##        print line
        if data_ready:
            try:
                x = [ x for x in line.rstrip('\n').split(";")]
##                print x
##                print len(x)

                if len(x)==18:
                    x.remove("&")
                    x.remove("")
                    data = [float(x) for x in x]
                    self.faults=0
##                    print data
                else:
                    self.faults=self.faults+1
                    if self.faults >= 2:
                        raise
            except:
##                self.faults=self.faults+1
##                print self.faults
                sys.stderr.write("eVot.read_all(): Expected %s of floats. Got this:"%num_fields)
                sys.stderr.write(line)
                data=[]
        return data

    
    def update_all(self):
        data = self.read_all(16)
        # simo
        if data:
            self.prev_time_stamp = self.time_stamp
            self.time_stamp , self.current , self.voltage , self.IR_L , self.IR_C , self.IR_R , \
            self.IMU_disp_x , self.IMU_disp_y , self.Yaw , self.Opt_flow_x , Opt_flow_y , self.Opt_flow_Yaw , \
            self.Encoder_x , self.Encoder_y , self.Motor_L , self.Motor_R = data
        else:
            return data
##        sampling_time = (self.time_stamp-self.prev_time_stamp)/1000.
##        if sampling_time>0:
##            if abs(self.Gz-self.Gz_offset)>50: # to remove the noise
##                self.gyro_heading += sampling_time*(self.Gz-self.Gz_offset)/130.5 # the integration to get the heading
##            heading_scaled = self.gyro_heading  % 360.
##            if heading_scaled>180:
##                heading_scaled-=360
##            elif heading_scaled<-180:
##                heading_scaled+=360
##            self.pos_values[0], self.pos_values[1], self.pos_values[2] = \
##                self.EKF.update_state([heading_scaled*pi/180.,self.encoder_right/1000.,self.encoder_left/1000.],sampling_time)
##            self.pos_values[2] = degrees(self.pos_values[2])
        return data

    def update_background(self):
        data = []
        while not data:
            data = self.read_all(16)
        self.time_stamp = data[0]
        while self.updating:
            data = self.update_all()
            try:
                data = self.update_all()
            except:
                self.pos_values = None
                self.updating = False
                sys.stderr.write("eVot.update_background(): Stop updating due to error.")
        self.halt()
        return

    def close(self):
        """
        Close BLE connection with eBot.
        """
        self.disconnect()

    #TODO: add disconnect feedback to robot
    def disconnect(self):
        """
        Close BLE connection with eBot.
        """
        self.stop_update_background()
        if self.serialReady:
            try:
                self.port.close()
            except:
                self.lostConnection()

    def robot_IR(self):
        """
        Retrieves and returns all six ultrasonic sensor values from the eBot in meters.

        :rtype: list
        :return: sonarValues
        """
        self.infraredValues[0] = float(self.IR_L)
        self.infraredValues[1] = float(self.IR_C) 
        self.infraredValues[2] = float(self.IR_R)
        return self.infraredValues

    def calibration_values(self):
        """
        Retrieves and returns the calibration values of the eBot.

        :rtype: list
        :return: all_Values (calibration values)
        """
        if self.serialReady:
            try:
                self.port.write("2C")
            except:
                self.lostConnection()
        line = self.port.readline()
        print line
        values = line.split(";")
##        print values
####        simo
##        while len(values) < 10:
##            if self.serialReady:
##                try:
##                    self.port.write("2C")
##                except:
##                    self.lostConnection()
##            line = self.port.readline()
##            values = line.split(";")
##        self.all_Values[0] = float(values[0])
##        self.all_Values[1] = float(values[1])
##        self.all_Values[2] = float(values[2])
##        self.all_Values[3] = float(values[3])
##        self.all_Values[8] = float(values[4]) / 1000
##        self.all_Values[7] = float(values[5]) / 1000
##        self.all_Values[6] = float(values[6]) / 1000
##        self.all_Values[5] = float(values[7]) / 1000
##        self.all_Values[4] = float(values[8]) / 1000
##        self.all_Values[9] = float(values[9]) / 1000
##        return self.all_Values

    def halt(self):
        """
        Halts the eBot, turns the motors and LEDs off.
        """
        if self.serialReady:
            try:
                self.port.write("2H")
            except:
                self.lostConnection()
        sleep(0.05)

    
    #Double check true vs. false
    def obstacle(self):
        """
        Tells whether or not there is an obstacle less than 250 mm away from the front of the eBot.

        :rtype: bool
        :return: True if obstacle exists
        """
        if self.Ultrasonic_front>250:
            return False
        else:
            return True


    #TODO: implement x, y, z returns and a seperate odometry function
    def acceleration(self):
        """
        Retrieves and returns accelerometer values; absolute values of X,Y and theta coordinates of robot with reference
        to starting position.

        :rtype: list
        :return: acc_values: Accelerometer values
        """
        self.acc_values[0] = float(self.IMU_disp_x )
        self.acc_values[1] = float(self.IMU_disp_y)
        self.acc_values[2] = float(self.Yaw)
        return self.acc_values

    def position(self):
        """
        Retrieves and returns position values of the eBot.

        :rtype: list
        :return: pos_values: X,Y position values + heading
        """
        return self.pos_values

    def power(self):
        """

        :return:
        """
        self.p_value[0] = float(self.voltage)
        self.p_value[1] = float(self.current)
        return self.p_value

    def imperial_march(self):
        """

        """
        if self.serialReady:
            try:
                self.port.write("2b")
            except:
                self.lostConnection()

    
    def port_name(self):
        """
        Returns port name of currently connected eBot.

        :return: port: Port name
        """
        return self.port


    def port_close(self):
        """
        Closes the COM port that corresponds to the eBot object.

        :raise Exception: Could not close COM port
        """
        try:
            self.port.close()
        except:
            self.serialReady = False
            raise Exception("Could not close COM port.")

    #TODO: Add com port argument functionality
    def port_open(self):
        """
        Still under development, currently just calls connect
        """
        self.connect()

    def wheels(self, LS, RS, dt):
        """
        Controls the speed of the wheels of the robot according to the specified values
        :param LS: Speed of left motor [mm/s]
        :param RS: Speed of right motor
        : param dt: Time [ms]
        """
        if LS > 1:
            LS = 1
        elif LS < -1:
            LS = -1
        if RS > 1:
            RS = 1
        elif RS < -1:
            RS = -1
        left_speed = int((LS + 2) * 100)
        right_speed = int((RS + 2) * 100)
        try:
            self.port.write( "w%i;%i;%i;"%(left_speed, right_speed,dt) )
        except:
            self.lostConnection()
        sleep(0.05)
        return


    def calibration(self, LS, RS):
        """
        Controls the speed of the wheels of the robot according to the specified values
        :param LS: Speed of left motor
        :param RS: Speed of right motor
        """
        if LS > 9999:
            LS = 9999
        elif LS < 1:
            LS = 1
        if RS > 9999:
            RS = 9999
        elif RS < 1:
            RS = 1
        left_calibration = str(LS).zfill(4)
        right_calibration = str(RS).zfill(4)
        try:
            self.port.write( ":c%s;%s"%(left_calibration, right_calibration) )
        except:
            self.lostConnection()
        sleep(0.05)
        return

    def lostConnection(self):
        """
        Handler for the case that the computer loses connection with the eBot.

        :raise Exception: Robot Connection Lost
        """
        try:
            self.port.close()
        except:
            pass
        self.serialReady = False
        raise Exception("Robot Connection Lost")
        ################################################################################
