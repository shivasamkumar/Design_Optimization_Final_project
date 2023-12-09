#MPC Controller for a Automated car (bi-cycle model)
#Import the Necessary libraries for the code 
import numpy as np
import matplotlib.pyplot as plt
import support_files as sp_files
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from qpsolvers import *
np.set_printoptions(suppress=True)

 
support=sp_files.SupportFiles()
constants=support.constants

# Load the constant values needed in the main file
Ts=constants['Ts']
outputs=constants['outputs'] 
hz = constants['hz'] # horizon prediction period
inputs=constants['inputs']
x_lim=constants['x_lim']
y_lim=constants['y_lim']
trajectory=constants['trajectory']


# Sekect Your Desired Trajectory 1,2 or 3 
x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref,t=support.trajectory_generator()
sim_length=len(t) # Number of control loop iterations
refSignals=np.zeros(len(X_ref)*outputs)

# Build up the reference signal vector:
#Vector contains all the outputs 
k=0
for i in range(0,len(refSignals),outputs):
    refSignals[i]=x_dot_ref[k]
    refSignals[i+1]=psi_ref[k]
    refSignals[i+2]=X_ref[k]
    refSignals[i+3]=Y_ref[k]
    k=k+1

# Load the initial states
# initial staes are same as the reference states 

x_dot=x_dot_ref[0]
y_dot=y_dot_ref[0]
psi=psi_ref[0]
psi_dot=0.
X=X_ref[0]
Y=Y_ref[0]

states=np.array([x_dot,y_dot,psi,psi_dot,X,Y])
statesTotal=np.zeros((len(t),len(states))) 
statesTotal[0][0:len(states)]=states

#Acceleration
x_dot_dot=0.
y_dot_dot=0.
psi_dot_dot=0.

accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
accelerations_total=np.zeros((len(t),len(accelerations)))



# Load the initial input
# Input at t = -0.02 s (steering wheel angle in rad (delta) and acceleration in m/s^2 (a))
U1=0 
U2=0 
UTotal=np.zeros((len(t),2)) 
UTotal[0][0]=U1
UTotal[0][1]=U2

# Initiate the controller - simulation loops
k=0
du=np.zeros((inputs*hz,1))


# Arrays for the animation
t_ani=[]
x_dot_ani=[]
psi_ani=[]
X_ani=[]
Y_ani=[]
delta_ani=[]

for i in range(0,sim_length-1):

    # Generate the discrete state space matrices
    Ad,Bd,Cd,Dd=support.state_space(states,U1,U2)

    # Generate the augmented current state and the reference vector
    x_aug_t=np.transpose([np.concatenate((states,[U1,U2]),axis=0)])

    # From the refSignals vector, we only extract the reference values from  [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
    # Example: Ts=0.1 seconds, t_now is 3 seconds, hz = 15 , so from refSignals vectors, you move the elements to vector r:
    # r=[x_dot_ref_3.1, psi_ref_3.1, X_ref_3.1, Y_ref_3.1, x_dot_ref_3.2, psi_ref_3.2, X_ref_3.2, Y_ref_3.2, ... , x_dot_ref_4.5, psi_ref_4.5, X_ref_4.5, Y_ref_4.5]
    # With each loop, it all shifts by 0.1 second because Ts=0.1 s
    k=k+outputs
    if k+outputs*hz<=len(refSignals):
        r=refSignals[k:k+outputs*hz]
    else:
        r=refSignals[k:len(refSignals)]
        hz=hz-1

    # Generate the compact simplification matrices for the cost function
    Hdb,Fdbt,Cdb,Adc,G,ht=support.mpc_simplification(Ad,Bd,Cd,Dd,hz,x_aug_t,du,i)
    ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
   
    ##Optimization##
    # Using the qp solver for finding the mininum cost function with respect to constraints
    try:
        du=solve_qp(Hdb,ft,G,ht,solver="cvxopt")
        du=np.transpose([du])
        # print(du)
        # exit()
    except ValueError as ve:
        print(Hdb)
        print(ft)
        print(G)
        print(ht)
        print(Adc)
        print(x_aug_t)
        print(du)
        print(i)
        break;

   
    # Update the real inputs
    U1=U1+du[0][0]
    U2=U2+du[1][0]

    UTotal[i+1][0]=U1
    UTotal[i+1][1]=U2

    states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states(states,U1,U2)
    statesTotal[i+1][0:len(states)]=states

    
    accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
    accelerations_total[i+1][0:len(accelerations)]=accelerations

    # This is to monitor the progress of the simulation
    if i%500==0:
        print("Loading: ######### "+str(round(i/sim_length*100,2))+"%")

    # To make the animations 5x faster
    if i%5==1:
        t_ani=np.concatenate([t_ani,[t[i]]])
        x_dot_ani=np.concatenate([x_dot_ani,[states[0]]])
        psi_ani=np.concatenate([psi_ani,[states[2]]])
        X_ani=np.concatenate([X_ani,[states[4]]])
        Y_ani=np.concatenate([Y_ani,[states[5]]])
        delta_ani=np.concatenate([delta_ani,[U1]])

# Animation Loop
# Animation loop has been reffered from the Udemy course : Python engineering animations: Bring math & data to life 

frame_amount=len(X_ani)
lf=constants['lf']
lr=constants['lr']

def update_plot(num):
    hz=constants['hz']
    car_1.set_data([X_ani[num]-lr*np.cos(psi_ani[num]),X_ani[num]+lf*np.cos(psi_ani[num])],
        [Y_ani[num]-lr*np.sin(psi_ani[num]),Y_ani[num]+lf*np.sin(psi_ani[num])])

    car_determined.set_data(X_ani[0:num],Y_ani[0:num])
    x_dot.set_data(t_ani[0:num],x_dot_ani[0:num])
    yaw_angle.set_data(t_ani[0:num],psi_ani[0:num])
    X_position.set_data(t_ani[0:num],X_ani[0:num])
    Y_position.set_data(t_ani[0:num],Y_ani[0:num])
    
    car_1_body.set_data([-lr*np.cos(psi_ani[num]),lf*np.cos(psi_ani[num])],
        [-lr*np.sin(psi_ani[num]),lf*np.sin(psi_ani[num])])

    car_1_body_extension.set_data([0,(lf+40)*np.cos(psi_ani[num])],
        [0,(lf+40)*np.sin(psi_ani[num])])

    car_1_back_wheel.set_data([-(lr+0.5)*np.cos(psi_ani[num]),-(lr-0.5)*np.cos(psi_ani[num])],
        [-(lr+0.5)*np.sin(psi_ani[num]),-(lr-0.5)*np.sin(psi_ani[num])])

    car_1_front_wheel.set_data([lf*np.cos(psi_ani[num])-0.5*np.cos(psi_ani[num]+delta_ani[num]),lf*np.cos(psi_ani[num])+0.5*np.cos(psi_ani[num]+delta_ani[num])],
        [lf*np.sin(psi_ani[num])-0.5*np.sin(psi_ani[num]+delta_ani[num]),lf*np.sin(psi_ani[num])+0.5*np.sin(psi_ani[num]+delta_ani[num])])

    car_1_front_wheel_extension.set_data([lf*np.cos(psi_ani[num]),lf*np.cos(psi_ani[num])+(0.5+40)*np.cos(psi_ani[num]+delta_ani[num])],
        [lf*np.sin(psi_ani[num]),lf*np.sin(psi_ani[num])+(0.5+40)*np.sin(psi_ani[num]+delta_ani[num])])

    yaw_angle_text.set_text(str(round(psi_ani[num],2))+' rad')
    steer_angle.set_text(str(round(delta_ani[num],2))+' rad')
    body_x_velocity.set_text(str(round(x_dot_ani[num],2))+' m/s')

    print(f"Processing frame {num+1}/{frame_amount}", end='\r')

    return car_determined,car_1,x_dot,yaw_angle,X_position,Y_position,\
    car_1_body,car_1_body_extension,car_1_back_wheel,car_1_front_wheel,car_1_front_wheel_extension,\
    yaw_angle_text,steer_angle,body_x_velocity#,car_predicted


# Set up your figure properties
fig_x=16
fig_y=9
fig=plt.figure(figsize=(fig_x,fig_y),dpi=120,facecolor=(0.8,0.8,0.8))
n=12
m=12
gs=gridspec.GridSpec(n,m)

# Main trajectory
plt.subplots_adjust(left=0.05,bottom=0.08,right=0.95,top=0.95,wspace=0.15,hspace=0)

ax0=fig.add_subplot(gs[:,0:9],facecolor=(0.9,0.9,0.9))
ax0.grid(True)
plt.title('Autonomous vehicle animation (5x faster than the reality)',size=15)
plt.xlabel('X-position [m]',fontsize=15)
plt.ylabel('Y-position [m]',fontsize=15)

# Plot the reference trajectory
ref_trajectory=ax0.plot(X_ref,Y_ref,'b',linewidth=1)

# Draw a motorcycle
car_1,=ax0.plot([],[],'k',linewidth=3)
# car_predicted,=ax0.plot([],[],'-m',linewidth=2)
car_determined,=ax0.plot([],[],'-r',linewidth=1)

# Zoomed vehicle
if trajectory==1:
    ax1=fig.add_subplot(gs[8:12,3:7],facecolor=(0.9,0.9,0.9))
elif trajectory==2:
    ax1=fig.add_subplot(gs[3:9,2:7],facecolor=(0.9,0.9,0.9))
else:
    ax1=fig.add_subplot(gs[2:6,2:5],facecolor=(0.9,0.9,0.9))
ax1.axes.get_xaxis().set_visible(False)
ax1.axes.get_yaxis().set_visible(False)

bbox_props_x_dot=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='b',lw='1')
bbox_props_steer_angle=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='r',lw='1')
bbox_props_angle=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='k',lw='1')

neutral_line=ax1.plot([-50,50],[0,0],'k',linewidth=1)
car_1_body,=ax1.plot([],[],'k',linewidth=3)
car_1_body_extension,=ax1.plot([],[],'--k',linewidth=1)
car_1_back_wheel,=ax1.plot([],[],'r',linewidth=4)
car_1_front_wheel,=ax1.plot([],[],'r',linewidth=4)
car_1_front_wheel_extension,=ax1.plot([],[],'--r',linewidth=1)


plt.xlim(-5,5)
plt.ylim(-4,4)

body_x_velocity=ax1.text(3,-1.5,'',size='10',color='b',bbox=bbox_props_x_dot)
steer_angle=ax1.text(3,-2.5,'',size='10',color='r',bbox=bbox_props_steer_angle)
yaw_angle_text=ax1.text(3,-3.5,'',size='10',color='k',bbox=bbox_props_angle)

body_x_velocity_word=ax1.text(3.7,3.4,'x_dot',size='10',color='b',bbox=bbox_props_x_dot)
steer_angle_word=ax1.text(3.8,2.5,'delta',size='10',color='r',bbox=bbox_props_steer_angle)
yaw_angle_word=ax1.text(4.2,1.6,'Psi',size='10',color='k',bbox=bbox_props_angle)


# x_dot function
ax2=fig.add_subplot(gs[0:3,9:12],facecolor=(0.9,0.9,0.9))
x_dot_reference=ax2.plot(t,x_dot_ref,'-b',linewidth=1)
x_dot,=ax2.plot([],[],'-r',linewidth=1)
ax2.spines['bottom'].set_position(('data',-9999999))
ax2.yaxis.tick_right()
ax2.grid(True)
plt.xlabel('time [s]',fontsize=12)
plt.ylabel('x_dot [m/s]',fontsize=12)
ax2.yaxis.set_label_position("right")

# Psi function
ax3=fig.add_subplot(gs[3:6,9:12],facecolor=(0.9,0.9,0.9))
yaw_angle_reference=ax3.plot(t,psi_ref,'-b',linewidth=1)
yaw_angle,=ax3.plot([],[],'-r',linewidth=1)
ax3.spines['bottom'].set_position(('data',-9999999))
ax3.yaxis.tick_right()
ax3.grid(True)
plt.xlabel('time [s]',fontsize=12)
plt.ylabel('Psi [rad]',fontsize=12)
ax3.yaxis.set_label_position("right")

# X function
ax4=fig.add_subplot(gs[6:9,9:12],facecolor=(0.9,0.9,0.9))
X_position_reference=ax4.plot(t,X_ref,'-b',linewidth=1)
X_position,=ax4.plot([],[],'-r',linewidth=1)
ax4.spines['bottom'].set_position(('data',-9999999))
ax4.yaxis.tick_right()
ax4.grid(True)
plt.xlabel('time [s]',fontsize=12)
plt.ylabel('X-position [m]',fontsize=12)
ax4.yaxis.set_label_position("right")

# Y function
ax5=fig.add_subplot(gs[9:12,9:12],facecolor=(0.9,0.9,0.9))
Y_position_reference=ax5.plot(t,Y_ref,'-b',linewidth=1)
Y_position,=ax5.plot([],[],'-r',linewidth=1)
ax5.yaxis.tick_right()
ax5.grid(True)
plt.xlabel('time [s]',fontsize=12)
plt.ylabel('Y-position [m]',fontsize=12)
ax5.yaxis.set_label_position("right")


car_ani=animation.FuncAnimation(fig, update_plot,
    frames=frame_amount,interval=20,repeat=True,blit=True)

# uncomment this saving the video file 
# writer = animation.PillowWriter(fps=40)
# car_ani.save("circleUpdate.gif", writer=writer)
plt.show()



#Set the conditions

if trajectory==1:
    x_lim=1000
    y_lim=1000
elif trajectory ==2:
    x_lim=600
    y_lim=350
elif trajectory == 3:
     x_lim=170*2
     y_lim=160*2

# Plot the world
plt.plot(X_ref,Y_ref,'--b',linewidth=2,label='The trajectory')
plt.plot(statesTotal[:,4],statesTotal[:,5],'r',linewidth=1,label='Car position')
plt.xlabel('X-position [m]',fontsize=12)
plt.ylabel('Y-position [m]',fontsize=12)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')
plt.xlim(0,x_lim)
plt.ylim(0,y_lim)
plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
plt.title("Comparison of the Actual trajectory and reference trajectory",fontsize=15)
plt.show()


plt.subplot(2,1,1)
plt.plot(t,UTotal[:,0],'r',linewidth=1,label='steering wheel angle')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('steering wheel angle [rad]',fontsize=12)
plt.grid(True)
plt.legend(loc='lower right',fontsize='small')

plt.subplot(2,1,2)
plt.plot(t,UTotal[:,1],'r',linewidth=1,label='applied acceleration')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('applied acceleration [m/s^2]',fontsize=12)
plt.grid(True)
plt.legend(loc='lower right',fontsize='small')
plt.show()



plt.subplot(3,1,1)
plt.plot(t,psi_ref,'--b',linewidth=2,label='Yaw_ref angle')
plt.plot(t,statesTotal[:,2],'r',linewidth=1,label='Car yaw angle')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('psi_ref-position [rad]',fontsize=12)
plt.grid(True)
plt.legend(loc='lower right',fontsize='small')

plt.subplot(3,1,2)
plt.plot(t,X_ref,'--b',linewidth=2,label='X_ref position')
plt.plot(t,statesTotal[:,4],'r',linewidth=1,label='Car X position')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('X-position [m]',fontsize=12)
plt.grid(True)
plt.legend(loc='lower right',fontsize='small')

plt.subplot(3,1,3)
plt.plot(t,Y_ref,'--b',linewidth=2,label='Y_ref position')
plt.plot(t,statesTotal[:,5],'r',linewidth=1,label='Car Y position')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('Y-position [m]',fontsize=12)
plt.grid(True)
plt.legend(loc='lower right',fontsize='small')
plt.show()


plt.subplot(3,1,1)
plt.plot(t,x_dot_ref,'--b',linewidth=2,label='x_dot_ref')
plt.plot(t,statesTotal[:,0],'r',linewidth=1,label='x_dot')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('x_dot [m/s]',fontsize=12)
plt.grid(True)
plt.legend(loc='center right',fontsize='small')

plt.subplot(3,1,2)
plt.plot(t,y_dot_ref,'--b',linewidth=2,label='y_dot_ref')
plt.plot(t,statesTotal[:,1],'r',linewidth=1,label='y_dot')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('y_dot [m/s]',fontsize=12)
plt.grid(True)
plt.legend(loc='center right',fontsize='small')

plt.subplot(3,1,3)
plt.plot(t,statesTotal[:,3],'r',linewidth=1,label='psi_dot')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('psi_dot [rad/s]',fontsize=12)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')
plt.show()


#Acceleration Plots
plt.subplot(3,1,1)
plt.plot(t,accelerations_total[:,0],'b',linewidth=1,label='x_dot_dot')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('x_dot_dot [m/s^2]',fontsize=12)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')

plt.subplot(3,1,2)
plt.plot(t,accelerations_total[:,1],'b',linewidth=1,label='y_dot_dot')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('y_dot_dot [m/s^2]',fontsize=12)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')

plt.subplot(3,1,3)
plt.plot(t,accelerations_total[:,2],'b',linewidth=1,label='psi_dot_dot')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('psi_dot_dot [rad/s^2]',fontsize=12)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')
plt.show()

exit()




