from numpy import *
from matplotlib.pyplot import *
#rcMC_params['text.usetex'] = True
from scipy import signal
import control
import control.matlab as cnt
import sys
#include the Models directory, where the whipple_model.py (and other common modules) live.
sys.path.insert(0, '../Models')
import matplotlib.pyplot as plt
from Lane_Controller import getLQRy

#construct close-loop model of the bike.
param_names = ['a ','b ','c','hrf','mrf','xff','zff','mff','Rfw','mfw','Rrw','mrw','Jyyf','Jyyr','lam']
# MC_params = array([.3,1.02,.08,.9,85,.9,.7,4,.35,3,.3,3,.28*.65,.12*.65,1.25])
MC_params = array([.6888,1.45,0.115,0.5186,158.1,1.25,0.7347,10,0.356,10,0.33,13,0.6657066,0.6554166,1.1])

def getEigsVecs(MC_params):
    #create a vector of velocities to investigate
    vvec = arange(.01,10,.01)
    #create a second copy of this vector that is four rows. This will make plotting easier
    vvec2 = zeros((6,len(vvec)))

    #our model is fourth order, so will have 4 eigenvalues. each can have a real and/or imaginary part.
    eigs_re = zeros((6,len(vvec)))
    eigs_im = zeros((6,len(vvec)))

    for k in range(0,len(vvec)):
        #get current velocity
        v = vvec[k]
        driveVelocity = 10
        Rlqr = .01#.001
        Qlqr = eye(6)#/10.0
        #get state space model at this speed
        KLQR,sys = getLQRy(v,Q=Qlqr,R=Rlqr,params=MC_params)
        #sys= getModelSS(v,MC_params)
        #get eigenvalues at this speed
        eigs,vecs = linalg.eig(sys.A)
        #get real parts and place in proper matrix for storage
        eigs_re[:,k] = real(eigs)
        #get imaginary parts and place in proper matrix for storage
        eigs_im[:,k] = imag(eigs)
        #fill up velocity vector corresponding with each eigenvalue
        vvec2[:,k] = [v,v,v,v,v,v]
    return vvec,eigs_re,eigs_im
######################## EIGS VS SPEED ##########################
def makePlot():
    vvec, eigs_re, eigs_im = getEigsVecs(MC_params)
    figure()
    plot(vvec,eigs_re[0,:],'k.',vvec,eigs_im[0,:],'k')
    xlabel('Speed (m/s)')
    ylabel('Eigenvalue (1/s)')
    legend(['real','imaginary'])
    plot(vvec,eigs_re[1,:],'k.',vvec,abs(eigs_im[1,:]),'k')
    plot(vvec,eigs_re[2,:],'k.',vvec,abs(eigs_im[2,:]),'k')
    plot(vvec,eigs_re[3,:],'k.',vvec,abs(eigs_im[3,:]),'k')
    ylim([-10,10])
######################## STEP RESPONSE ##########################



#load data file from webots:
    t,goalroll,tq,spd,roll,laneposition,yaw,y,rollrate,steer,steerrate = loadtxt("closeloop_webots_data.txt",delimiter=",",unpack=True)
    U = mean(spd)#what was the speed of the test
    T = mean(tq)#what was the step magnitude?
    X0 = array([roll[0],steer[0],rollrate[0],steerrate[0],yaw[0],y[0]])
    print("Testing at velocity "+str(U)+" and step torque "+str(T))

    #get state space model of bike based on close_loop_model
    driveVelocity = 10
    Rlqr = .1#.001
    Qlqr = eye(6)#/10.0
    KLQR,sys = getLQRy(driveVelocity,Q=Qlqr,R=Rlqr,params=MC_params)
    #perform an lsim using mlaneposition and initial condition values
    yout,tout,xout = cnt.lsim(sys,y,t,X0)


    #now plot data vs. close_loop_model
    figure()
    subplot(3,1,1)
    #plt.tight_layout()
    plot(tout,yout[:,0],'k',t,roll,'r')
    legend(['model','Webots'],fontsize=15)
    title('$U=$ '+str(round(U,2))+"m/s; $T_\delta=$ "+str(round(T,2))+"Nm; $\phi_0=$"+str(round(roll[0],2))+" rad",fontsize=15)
    ylabel('Roll (rad)',fontsize=12)
    plt.xticks(fontsize = 11) 
    plt.yticks(fontsize = 11) 
    subplot(3,1,2)
    plot(tout,yout[:,1],'k',t,steer,'r')
    ylabel('Steer (rad)',fontsize=12)
    xlabel('Time (s)',fontsize=12)
    plt.xticks(fontsize = 11) 
    plt.yticks(fontsize = 11) 
    subplot(3,1,3)
    plot(tout,yout[:,5],'k',t,y,'r')
    ylabel('Laneposition(m)',fontsize=12)
    xlabel('TIme(s)',fontsize=12)
    plt.xticks(fontsize = 11) 
    plt.yticks(fontsize = 11) 
    plt.savefig("../../scripts/Figures/4_closeloop_motocycle_model_vs_Webots_phi_0=$"+str(round(roll[0],2))+" rad.png")
    show()
