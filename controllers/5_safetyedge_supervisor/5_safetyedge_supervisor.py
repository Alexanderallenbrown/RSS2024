from controller import Supervisor
from controller import Node

# 0, 1, 2, 3, 4, check where code is stopping and not runnig
# get access to nodes through Supervisor
#mc_node = robot.getFromDef('PTW')
TIME_STEP = 1
robot = Supervisor()

#mc_node = robot.getFromDef('PTW')
# get our safety edge node
safedge_node = robot.getFromDef('SlantRoad')
road_pos_field = safedge_node.getField('roadPosition')
print('00')
road_pos = road_pos_field.getSFVec3f() # creates a road pos vector of length 3
print('0')

# create and set all variables needed for simulation
sim_counter = 0 # counts number of sims occured
sim_time = 10 # sets sim time we want to rst sim at
offset_values = [-0.50, -1.0, -1.5, -2.0, -2.5, -3.0, -3.5, -4.0, -4.5, -5.0]
i = 0 # index for offset_values array
rp_x = 0 # x pos of road vec
rp_y = 0 # y pos of road vec
rp_z = 0 # z pos of road vec

# sets each index of vector equal to those
road_pos[0] = rp_x
road_pos[1] = rp_y
road_pos[2] = rp_z

print('vars')
print(road_pos)


y = open('road_position_y_position.txt', 'w') # opens txt for file
y.write('0.0') # writes the initial y position to file
y.close() # closes file...
print('initial printed to text file')



for sim_counter in range(0,10): # replaced while with for loop
        print('going through while loop') # this is bening printed
        
        rp_y = 0 # resets y for each sim so offsets incs correctly
        offset = offset_values[i]
        new_rp_y = rp_y + offset # made new var to avoid pot issues
        new_road_pos = [0, new_rp_y, 0] # we need to put new y values back into vec
        road_pos = road_pos_field.setSFVec3f(new_road_pos) # sets original road_pos to new one
        print(new_road_pos) # this is being printed

        y = open('road_position_y_position.txt', 'w') # opens txt for file
        y.write('Road Y Position is ' + str(new_rp_y)) # writes new val to txt file, # str(val)
        y.close()
        print('printed to the txt file') # keeps rewriting over previous values, this is a later fix
        
        while robot.getTime() < sim_time:
            robot.step(TIME_STEP)
            print('going through while loop')
       
        robot.simulationReset() # only reset the simulation if
        i+=1 # updates index of offset values array
        sim_counter+=1
        print(sim_counter) # sim counter is implenting correctly!!!
        print('3')

                 #if (i <20 ):
                #mc_node.setVelocity([10,0,0,0,0,0])
                #i += 1

                # code moves t
