from numpy import *
from maptools import *
from matplotlib.pyplot import *
from scipy.signal import medfilt

#Alexander Brown, March 2023

class PointMassRacer():
    def __init__(self,type,filename,axmax,aymax,maxSpeed,Kthresh = .1,check_decreasing = False):
        self.Kthresh = Kthresh
        self.axmax = axmax
        self.aymax = aymax
        self.M = Map(type,filename)
        #print(self.M.S)
        self.Umax = maxSpeed
        self.check_decreasing = check_decreasing

    def K_here(self,S):
        return interp(S,self.M.S,self.M.K)

    def inTurn(self,K):
        return K>self.Kthresh

    def maxSpeed(self,K):
        return sqrt((self.aymax*9.81)/abs(K))

    def nextCurvature(self,S_here):
        inds = where((self.M.S>S_here)&(abs(self.M.K)>self.Kthresh))

        #print("inds: "+str(inds[0]))
        if(len(inds[0])>1):
            next_turn_ind = inds[0][0]
            K_next = self.M.K[next_turn_ind]
            S_next_turn = self.M.S[next_turn_ind]
            if(self.check_decreasing):
                #in this turn, how many degrees should we check for a tighter radius?
                arc_theta = pi/4
                #how many meters is that? theta/K = S
                arc_S = arc_theta/abs(K_next)
                #print(arc_S,S_next_turn,self.M.S[inds])
                #find the max curvature in that range of S.
                query_inds = where(self.M.S[inds]<(arc_S+S_next_turn))+inds[0][0]
                crit_K_ind = argmax(abs(self.M.K[query_inds]))
                K_crit = self.M.K[crit_K_ind+inds[0][0]]
                # print("K crit: "+str(abs(self.M.K[query_inds])))
                #print("K crit ind: "+str(crit_K_ind))
                #print("K_next = "+str(K_next))
            else:
                K_crit = K_next
        else:
            next_turn_ind = len(self.M.S)-1
            K_next = self.M.K[-1]
            K_crit = K_next
        dist_to_next_turn = self.M.S[next_turn_ind]-S_here
        #print("next turn ind: "+str(next_turn_ind))
        return K_crit,dist_to_next_turn

    def getCriticalSpeed(self,S_here):
        #get dist to next turn
        K_next,dist_to_next_turn = self.nextCurvature(S_here)
        #get critical speed for this turn
        Uturn = self.maxSpeed(K_next)
        #print("dist_to_next_turn, Uturn, K_next: "+str(dist_to_next_turn)+","+str(Uturn)+","+str(K_next))

        #back-propagate max brake to find the speed
        #we would need to be going to necessitate a brake.
        Ucrit = sqrt(Uturn**2 + 2*self.axmax*9.81*dist_to_next_turn)
        return Ucrit

    def tooFast(self,U_here,S_here):
        Ucrit = self.getCriticalSpeed(S_here)
        #print("Ucrit, U_here returns: "+str(Ucrit)+","+str(U_here))
        return U_here>=Ucrit

    def getLap(self):
        Svec = array([0])
        Uvec = array([self.Umax])
        tvec = array([0])
        dt = .01
        stop = False
        while Svec[-1]<max(self.M.S):
            #print("S: "+str(Svec[-1]))
            #find curvature now
            K = self.K_here(Svec[-1])
            tnew = tvec[-1]+dt
            tvec = append(tvec,tnew)
            #determine if we're currently in a turn
            if (self.inTurn(K)):
                #print("in turn")
                #Unew = min(self.maxSpeed(K),self.Umax)
                Unew = Uvec[-1]
            else:
                #print("Uvec end is: "+str(Uvec[-1]))
                #if we're not in a turn, should we accel or brake?
                #print("toofast returns: "+str(self.tooFast(Uvec[-1],Svec[-1])))
                if(self.tooFast(Uvec[-1],Svec[-1])):
                    #update U with negative accel
                    Unew = Uvec[-1] - self.axmax*9.81*dt
                else:
                    #we should be accelerating if we're not at top speed
                    if(Uvec[-1]<self.Umax):
                        Unew = Uvec[-1]+self.axmax*9.81*dt
                    else:
                        Unew = self.Umax

            #line below tests integration approach
            #Unew = Uvec[-1]+ self.axmax*9.81*dt
            Snew = Svec[-1] + Uvec[-1]*dt
            Uvec = append(Uvec,Unew)
            Svec = append(Svec,Snew)
        return tvec, Svec, Uvec

    def getSpeedProfile(self):
        t,S,U = self.getLap()
        U = medfilt(U,11)
        U_grid = interp(self.M.S,S,U)
        return U_grid




def main():
    #pmr = PointMassRacer('gps','grasstrack.csv',.05,.15,6,.1,False)
    #pmr2 = PointMassRacer('gps','grasstrack.csv',.05,.15,6,.1,True)
    pmr = PointMassRacer('gps','grasstrack2.csv',.05,.2,6.25,Kthresh = .02,check_decreasing=False)
    pmr2 = PointMassRacer('gps','grasstrack2.csv',.05,.2,6.25,Kthresh = .02,check_decreasing=True)
    t,S,U = pmr.getLap()
    t2,S2,U2 = pmr2.getLap()
    Ugrid = pmr.getSpeedProfile()
    Ugrid2 = pmr2.getSpeedProfile()
    accel = diff(U)/diff(t)
    accel = append(accel,0)
    accel2 = diff(U2)/diff(t2)
    accel2 = append(accel2,0)
    figure(0)
    plot(t,accel,t2,accel2)
    xlabel('Time (s)')
    ylabel('Long accel (m/s/s)')
    legend(['no decreasing radius detection','decreasing radius detection'])

    figure()
    plot(t,U,t2,U2)
    xlabel('Time (s)')
    ylabel('Speed (m/s)')

    # figure()
    # plot(S,U,pmr.M.S,Ugrid,'ro')
    # xlabel('Station (m)')
    # ylabel('Speed (m/s)')

    figure()
    plot(pmr.M.S,pmr.M.K)
    xlabel('Station (m)')
    ylabel('Curvature (1/m)')
    show()

    figure()
    plot(pmr.M.X,pmr.M.Y,'ko')
    xlabel('E (m)')
    ylabel('N (m)')
    axis('equal')

if __name__ == '__main__':
    main()
