"""Motorcycle path following controller."""

from controller import Robot, Motor, InertialUnit
from numpy import *
from Rollover import Rollover
from realtime_plotter import RealTimePlot
from MC_model import *
from maptools import *
from pointmassracer import *
from matplotlib.pyplot import *
import control
import control.matlab as cnt
#load map of the track for use in controller
#when you run updateTrack from the trackbuilder/ folder, this file gets updated automatically:
map = Map(type='xyz',filename='../../map/track.csv')
#get a speed profile for the track using point-mass racer.
pmr = PointMassRacer('xyz','../../map/track.csv',0.2,0.2,10,Kthresh=.02,check_decreasing=False)
Ugrid = pmr.getSpeedProfile()

showPlots = False

if showPlots:
    figure
    plot(map.S,Ugrid)
    figure()
    plot(map.X,map.Y)
    axis('equal')

#plot = RealTimePlot(x_label='time',y_label='rollrate',marker='k')

# create the Robot instance.
robot = Robot()
yawCorr = Rollover()
roadCorr = Rollover()

# SIMULATION SETUP
driveVelocity = 10
step_mag = 0.1
goalRoll = step_mag
doSteadyTurn = False

param_names = [   'a ','b ','c','hrf','mrf','xff','zff','mff','Rfw','mfw','Rrw','mrw','Jyyf','Jyyr','lam']

#rough KX450 params:
#MC_params = array([.5,1.49,.1,   .9,   200, 1.25, .75,    5,   .3,    12,  .3,   15,    1    ,1.35,  1.1])
#razor params:
#MC_params = array([.451,.99,.0526,.506,22.717,.9246,.515,5.105,.2413,4.278,.2413,7.1,.125,.2,1.345])
MC_params = array([.708,1.45,0.115,0.509,158.1,1.25,0.7347,10,0.356,10,0.33,13,.3,.3,1.1])

##### do eigenvalue study ######
if(showPlots):
    vstudy,restudy,imstudy = plotEigStudy(MC_params,True)

# LANE CONTROL PARAMETERS:
Tprev =1
Kprev = .1
Kdprev= 0.05
eprev_old = 0

# CONTROL PARAMETERS
lastControlTime = 0
dTcontrol = 0.005
# get controller designs for a range of speeds
lqrspeeds = arange(1,16,1)
Rlqr = .001
Qlqr = eye(5)#/10.0
Qlqr[4,4]=1.0

########## GET LQR GAINS ################
if(showPlots):
    fig,ax = subplots()
    
for k in range(0,len(lqrspeeds)):
    Klqr,sys = getLQRI(lqrspeeds[k],Q=Qlqr,R=Rlqr,params=MC_params)
    if showPlots:
        yout,tout = cnt.step(sys)
        yout*=step_mag
        plot(tout,yout[:,0],label='U='+str(lqrspeeds[k]))

    allGains = ravel(Klqr)
    if(k==0):
        gainmtx = hstack((lqrspeeds[k],allGains))
    else:
        gainmtx = vstack((gainmtx,hstack((lqrspeeds[k],allGains))))

if(showPlots):
    xlabel('Time (s)')
    ylabel('Roll (rad)')
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)
    show()

print("LQR gain matrix: ")
print(gainmtx)

def clamp(val,min,max):
    assert min<max
    if(val<min):
        val=min
    elif val>max:
        val=max
    return val


def setDriveMotorTorque(self,motor,command,omega):
    #Assume that the torque to the wheel is 650 N-m max
    #for now, just allow max torque, and assume brake is same
    motorTorque = clamp(command,-650,650)
    #set motor force
    motor.setTorque(motorTorque)

def find_nearest_index(array, value):
    array = asarray(array)
    idx = (abs(array - value)).argmin()
    return idx

def getCurrentGains(speed):
    #find the speed in the gain array closest to ours
    idx = find_nearest_index(lqrspeeds,speed)
    #find the gain set at this index
    Klqr = gainmtx[idx,1:]
    return Klqr,lqrspeeds[idx]



recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./webots_data.txt','w')
    f.write("# time, goalRoll, Torque, speed, roll, rollrate, steer, steerrate, intE\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)
steergyro = robot.getDevice('steergyro')
steergyro.enable(timestep)

steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)


####### INITIALIZE VALUES ########
T = 0

simtime = 0.0
yawRate = 0
oldYaw = 0

rollInt = 0
inteYawRate = 0
oldRoll = 0

steerangle = 0
oldsteer = 0
steerRate = 0

oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()

firstLoop = True

#set the simulation forward speed and calculate rear wheel omega
Rrw = MC_params[10]
driveOmega = driveVelocity/Rrw



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime+=timestep/1000.0
    if(firstLoop):
        oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
        oldYaw = yawCorr.update(oldYaw)
        oldsteer = steersensor.getValue()
        firstLoop=False
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    #get current fwd speed
    U = gps.getSpeed()
    #get GPS reading
    xyz = gps.getValues()
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    #rollInt += (timestep/1000.0)*rpy[0]
    yaw = rpy[2]
    yaw = yawCorr.update(yaw)
    yawRate = gyros[2]#(yaw-oldYaw)/(timestep/1000.0)
    # print("yaw/old: "+str(yaw)+","+str(oldYaw))
    oldYaw = yaw
    roll = rpy[0]
    rollRate = gyros[0]#(roll-oldRoll)/(timestep/1000.0)
    rollRate_bad = (roll-oldRoll)/(timestep/1000.0)
    # print(rollRate,rollRate_bad)
    oldRoll = roll

    #now get steer values and calculate steer rate.
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    steerangle = steersensor.getValue()
    steerRate = (steerangle-oldsteer)/(timestep/1000.0)
    oldsteer = steerangle

    ################### FINISH READING SENSORS, BEGIN MAP Control ##########
    # get current offset from lane from maptools
    mapstation,offset,roadyaw,roadK = map.station_here(xyz[0],xyz[1],xyz[2],type="xyz")
    #get current drive velocity based on Ugrid
    if not doSteadyTurn: #make sure we didn't set a flag to do steady turn instead of path follow
        driveVelocity = interp(mapstation,map.S,Ugrid)
    #now figure out the commanded rear wheel angular velocity
    driveOmega = driveVelocity/MC_params[10]
    #now set to that velocity (TODO make speed controller!)
    # motor.setAvailableTorque(1000)
    motor.setVelocity(driveOmega)
    # print("speed = "+str(U))

    if((simtime-lastControlTime)>dTcontrol):

        #get yaw angle of road, accounting for rollowver:
        roadyaw = roadCorr.update(roadyaw)
        #pick preview distance TODO: make it constant time, adjust w/speed?
        Sprev = Tprev*U
        #using map, determine lane error in future.
        prev_x,prev_y,prev_z = map.relativeMapPoints(xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],yaw,array([Sprev]))
        #using map, determine roll at future.
        prevK = interp(Sprev+mapstation,map.S,map.K)
        prevYaw = interp(Sprev+mapstation,map.S,map.roadyaw)
        #now use this to determine goal roll.
        eprev = -prev_y[0]
        if not doSteadyTurn:
            goalRoll = Kprev*eprev + Kdprev*(eprev - eprev_old)/dTcontrol#-arctan(U**2*prevK/9.81)#-Kprev*prev_y[0]
            eprev_old = eprev
        #determine goal yaw angle change based on this.
        #the misalignment with this future position is
        #for small angles, prev_y/Sprev
        goalYaw = 0#roadyaw + prev_y[0]/Sprev
        yawError = goalYaw - yaw

        # goalRoll = 0
        eRoll = goalRoll - roll
        rollInt = rollInt + eRoll*(simtime-lastControlTime)

        #goal steer based on goal yaw rate
        goalYawRate = 0

        Klqr,lqrU = getCurrentGains(U)
        # print("K = "+str(Klqr))
        T = Klqr[0]*(eRoll) - Klqr[1]*steerangle - Klqr[2]*rollRate - Klqr[3]*steerRate - Klqr[4]*rollInt #+Klqr[4]*yawError#+ Klqr[4]*yawError
        #print("rate = "+str(rollRate)+", bad: "+str(rollRate_bad))
        Tlim = 1000
        if(T>Tlim):
            T = Tlim
        elif(T<-Tlim):
            T = -Tlim

        # print("speed = "+str(U)+", Tq: "+str(T)+", prev error: "+str(prev_y[0])+", yaw error: "+str(yawError)+", yaw: "+str(yaw)+", goalYaw: "+str(goalYaw) )
        print("lqr U: "+str(lqrU)+", roll: "+str(roll)+", steer: "+str(steerangle)+", T="+str(T)+", rI: "+str(rollInt))
        # print("Torque: "+str(T))
        steer.setControlPID(0.0001,0,0)
        steer.setPosition(float('inf'))
        # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
        steer.setTorque(T)
        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(T)+","+str(U)+","+str(roll)+","+str(steerangle)+","+str(rollRate)+","+str(steerRate)+","+str(rollInt)+"\r\n")
