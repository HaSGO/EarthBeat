#!/usr/bin/env python 2.7
"""
This file is part of EarthBeat.

EarthBeat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

EarthBeat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with EarthBeat.  If not, see <http://www.gnu.org/licenses/>.
"""
"""
This file read data sent from an Arduino on the usb-serial and show them using mathplotlib.
"""
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import serial
import numpy as np
from time import sleep	
from time import gmtime, strftime
import math
import argparse
import os
#TODO check file1 for dbg, modified
#TODO test "data save" files functions

class EarthbeatCom:
	"""
	Class to manage the communication with acquisition hardware (Arduino board).
	"""
	
	def __init__(self, filename, baudrate):
		"""
		Open the serial port and set the baudrate.
		Try to read the first bytes to see if they are correct (all zeroes).
		"""
		self.p=serial.Serial(filename, baudrate)
		self.file1=open("dbg.txt","w") #open the log file
		sleep(2)
		self.p.write('c') #set continuos read-mode
		for i in range(6):
			tmp = self.p.read()
			if tmp != '\x00': 
				raise ValueError('Response is not 0')
		self.p.write('r')
		
	def __del__(self):
		self.p.close()
		
	def getSync(self):
		"""
		Recover the situation. If we have lost the sync with the hardware, throw the current message and resync on the next.
		"""
		inizio=False
		while not inizio:
			tmp=ord(self.p.read())
			if tmp==0:
				tmp1=ord(self.p.read())
				if tmp1==0:
					tmp2=ord(self.p.read())
					start = tmp2 >> 3
					if start == 0:  
						inizio=True
						tmp=tmp2
		return [True,tmp] 
	
	def readSample(self, count = 0):
		"""
		Read a sample, made of three distinct numbers, one for each channel.
		"""
		inizio=False

		tmp=ord(self.p.read())
		check=tmp
		#check if the first byte is zero, if not resync
		if tmp != 0:
			self.file1.write("First byte of the start sequence is not 0")
			ret=self.getSync() 
			inizio=ret[0]
			tmp=ret[1]
		
		if not inizio:
			tmp = ord(self.p.read())
			#check if first 5 bits of the second byte are zeroes, if not resync
			start = tmp >> 3
			if start != 0:
				self.file1.write("Last 5 bit of start sequence are not 0 - start=" + str(start) + " count=" + str(count) + " tmp=" + str(tmp)+"\n")
				ret=self.getSync() 
				inizio=ret[0]
				tmp=ret[1]
		
		#unpack the three 10-digit numbers acquired and packed by arduino
		check1=tmp
		tmpx = tmp << 7
		tmp = ord(self.p.read())
		check2=tmp

		tmpx = tmpx | tmp >> 1
		tmp= tmp & 0b00000001
		tmpy = tmp << 9
		tmp = ord(self.p.read())
		check3=tmp

		tmpy = tmpy | tmp<<1
		tmp = ord(self.p.read())
		check4=tmp

		tmpy = tmpy | tmp >> 7
		tmp = tmp & 0b01111111
		tmpz = tmp <<3
		tmp = ord( self.p.read())
		check5=tmp

		tmpz = tmpz | tmp >> 5
		tmp = tmp & 0b00011111
		
		#check the last five bits of the second to last byte, if not resync
		if tmp != 0:
			self.file1.write("First 5 bit of stop sequence are not 0 - count=" + str(count) + " tmp=" + str(tmp)+"\n")
			ret=self.getSync() 
			inizio=ret[0]
			tmp=ret[1]
			
		inizio=False	
		if not inizio:
			tmp = ord(self.p.read())
			check6=tmp
			#check if the last byte is zero, if not resync
			if tmp != 0:
				self.file1.write("Last byte of stop sequence is not 0 - count=" + str(count) + " tmp=" + str(tmp)+"\n")
				ret=self.getSync() 
				inizio=ret[0]
				tmp=ret[1]

		return [tmpx, tmpy, tmpz]

def openFiles(dateStr):
	"""
	Open write file for data collection
	"""
	out_vel = open(args.out+"/V_"+dateStr+".tsv","w")
	return out_vel

def plotFFT(liner, dataArray):
	"""
	Evaluate FFT and print it on a graph using mathplotlib
	"""
	sp = np.fft.fft(dataArray)
	sp_abs = [0] * ( len(sp)/2 )
	for fourierIndex in range(len(sp)/2):
		sp_abs[fourierIndex]=math.log(abs(sp[fourierIndex]))
	liner.set_ydata(sp_abs)

parser = argparse.ArgumentParser(description='Earthbeat data acquisition.')
parser.add_argument('-d', '--device', default='/dev/ttyACM0',
                   help='Path to the serial device used by Arduino')
parser.add_argument('-s', '--speed', type=int, default=460800,
                   help='Speed of the serial device used by Arduino')
parser.add_argument('-w', '--writetofile', action="store_true",
                   help='Enable data store into the data subdirectory.')
parser.add_argument('-o', '--out', default='./data',
                   help='Path to the data directory, default is \'./data\'.')
args = parser.parse_args()
sn_dev = args.device
sn_speed = args.speed
writeDataEnable = args.writetofile
print("Dev: "+sn_dev)
print("Speed: "+str(sn_speed) )
print("Write: "+str(writeDataEnable))
if writeDataEnable:
	if os.path.isdir(args.out) == False:
		 os.makedirs(args.out)
ion()

#initialize tmp variables to store and plot data.
arrayX=[]
arrayY=[]
arrayZ=[]
arrayXY=[]
arrayYZ=[]
arrayXZ=[]

arrayX=[0]*1200
arrayY=[0]*1200
arrayZ=[0]*1200
A_x=[0]*1200
A_y=[0]*1200
A_z=[0]*1200	
dataMin=0
dataMax=1024

#setup plots
f, ((ax1, ax2, ax3) , (ax4, ax5, ax6)) = plt.subplots(2, 3, sharex=False)
r1=np.arange(0,1200,1)
ylim([0,1024])	
ax1.set_title('Vx')
ax2.set_title('Vy')
ax3.set_title('Vz')

ax4.set_title('FFT(x)')
ax5.set_title('FFT(y)')
ax6.set_title('FFT(z)')

linex, = ax1.plot(arrayX)
liney, = ax2.plot(arrayY,'r-')
linez, = ax3.plot(arrayZ,'g-')

sp = np.fft.fft(arrayX)
sp_abs = [0] * ( len(sp)/2 )

linex_fft, = ax4.plot(sp_abs)
liney_fft, = ax5.plot(sp_abs,'r-')
linez_fft, = ax6.plot(sp_abs,'g-')

ax1.set_ylim(dataMin, dataMax)
ax2.set_ylim(dataMin, dataMax)
ax3.set_ylim(dataMin, dataMax)

ax4.set_ylim(-2, 12)
ax5.set_ylim(-2, 12)
ax6.set_ylim(-2, 12)

#show plots
f.frameon=True
f.show()
plt.pause(1)

count = 1

#check if we have to write data to file
if writeDataEnable:	
	out_vel = openFiles(strftime("%Y-%m-%d_%H:%M:%S", gmtime()))


#open Earthbeat Acquisition on ttyACM0
reader = EarthbeatCom(sn_dev, sn_speed) 
print("Start")

#start the data acquisition loop
while (1):
	try:
		smpl = reader.readSample(count) 
		tmpx = smpl[0]
		tmpy = smpl[1]
		tmpz = smpl[2]
		#if arrayX's lengths is over or equal to 1200, delete the oldest value. All the arrays have the same length.
		if(len(arrayX)>=1200):
			del arrayX[0]
			del arrayY[0]
			del arrayZ[0]
		#add latest samples.
		arrayX.append(tmpx)		
		arrayY.append(tmpy)
		arrayZ.append(tmpz)
		if writeDataEnable:
			out_vel.write(str(tmpx)+"\t"+str(tmpy)+"\t"+str(tmpz)+"\n")
		
		if(len(arrayX)>=1200):
			#every 100 samples update the graphs
			if (count%100)==0:
				#update graphs every 100 samples
				linex.set_ydata(arrayX)
				liney.set_ydata(arrayY)
				linez.set_ydata(arrayZ)
				
				plotFFT(linex_fft, arrayX)
				plotFFT(liney_fft, arrayY)
				plotFFT(linez_fft, arrayZ)

				f.canvas.draw()
				plt.pause(0.00000001)
		if writeDataEnable:		
			#every 60000 samples do an output file rotation.
			if(count%60000==59999):
				#file rotation
				out_vel.close()
				out_vel = openFiles(strftime("%Y-%m-%d_%H:%M:%S", gmtime()))
			
		count=count+1
		
	#catch CTRL+C
	except KeyboardInterrupt:
		print "\n...Bye"
		out_vel.close()
		sys.exit(0)

