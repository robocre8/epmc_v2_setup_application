# REFERENCE: https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/gyro-calibration-with-jupyter
import tkinter as tk
import ttkbootstrap as tb
from ttkbootstrap.constants import *

from collections import deque
from termcolor import colored
from time import time_ns, sleep

from epmc_v2.globalParams import g


class ImuCompYawDriftFrame(tb.Frame):
  def __init__(self, parentFrame):
    super().__init__(master=parentFrame)

    self.start_process = False
    self.loop_count = 0
    self.no_of_samples = 10000

    # intialize parameter
    self.yawVelBiasArr = []
    self.yaw0 = 0.00
    self.t0 = 0

    self.label = tb.Label(self, text="COMPUTE YAW VEL BIAS", font=('Monospace',16, 'bold') ,bootstyle="dark")
  
    #create widgets to be added to the Fame
    percent = 0.0
    self.textVal = tb.Label(self, text=f'{percent} %', font=('Monospace',20, 'bold'), bootstyle="primary")
    self.progressBar = tb.Progressbar(self, bootstyle="danger striped", mode='determinate',
                                      maximum=100, length=200, value=0.0)

    buttonStyle = tb.Style()
    buttonStyleName = 'primary.TButton'
    buttonStyle.configure(buttonStyleName, font=('Monospace',15,'bold'))
    self.pressButton = tb.Button(self, text="START", style=buttonStyleName,
                                 command=self.change_btn_state)
    
    self.canvasFrame = tb.Frame(self)
    
    #add created widgets to Frame
    self.label.pack(side='top', pady=(20,50))
    self.textVal.pack(side='top', expand=True, fill='y')
    self.progressBar.pack(side='top', expand=True, fill='x', padx=50)
    self.pressButton.pack(side='top', fill='y')
    self.canvasFrame.pack(side='top', expand=True, fill='both', pady=(10,0))

    #create widgets to be added to the canvasFame
    self.canvas = tb.Canvas(self.canvasFrame, width=300, height=10, autostyle=False ,bg="#FFFFFF", relief='solid')

    #add created widgets to canvasFame
    self.canvas.pack(side='left', expand=True, fill='both')

    # start process
    self.calibrate_imu()

  def reset_all_params(self):
    self.loop_count = 0
    self.no_of_samples = 10000

    self.yawVelBiasArr = deque(maxlen=self.no_of_samples)
    self.yaw0 = g.epmcV2.readYawWithDrift()
    self.t0 = time_ns()

    percent = 0.0
    self.textVal.configure(text=f'{percent} %')
    self.progressBar['value'] = percent

  def read_data(self):
    if self.start_process:
      self.loop_count += 1

      if self.loop_count%10 == 0:

        yaw1 = g.epmcV2.readYawWithDrift()
        t1 = time_ns()

        dt = t1-self.t0

        yawVelDrift = (yaw1-self.yaw0)*1e9/float(dt)

        self.yawVelBiasArr.append(yawVelDrift)

      percent = (self.loop_count*100)/10000
      self.textVal.configure(text=f'{percent} %')
      self.progressBar['value'] = percent

      if percent >= 100.0:
        percent = 100.0
        self.textVal.configure(text=f'{percent} %')
        self.progressBar['value'] = percent
        self.print_data()
      else:
        self.canvas.after(1, self.read_data)

    else:
      self.reset_all_params()
      self.canvas.after(10, self.calibrate_imu)

  def print_data(self):
    yawVelDriftBias = self.average(self.yawVelBiasArr)

    g.epmcV2.writeYawVelDriftBias(yawVelDriftBias)

    yawVelDriftBias = g.epmcV2.readYawVelDriftBias()

    print(colored("\n---------------------------------------------------------------", 'magenta'))
    print(colored("stored yaw vel drift bias:", 'green'))
    print(yawVelDriftBias)
    print(colored("---------------------------------------------------------------", 'magenta'))


  def calibrate_imu(self):
    if self.start_process:
      self.reset_all_params()
      self.read_data()
    else:
      self.reset_all_params()
      self.canvas.after(10, self.calibrate_imu)

  def change_btn_state(self):
    if self.start_process:
      self.start_process = False
      self.pressButton.configure(text='START')

    else:
      self.start_process = True
      self.pressButton.configure(text='STOP')

  def average(self, val):
      ans = 0
      for i in val:
        ans= ans + i
      
      ans = ans/len(val)
      
      return ans