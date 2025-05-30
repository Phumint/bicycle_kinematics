import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
import keyboard
import time
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass

@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # in radians

def rotation_matrix_2d(theta_deg):
    theta = np.radians(theta_deg)
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

#initial state
pose = RobotPose()
dt = 0.01  # time step

body_width = 150
body_length = 250
s = 3  # distance between wheels
wheel_radius = 25
wheel_thickness = 25
COM = body_length/2
Lr = body_length/2 #rear axle to COM
Lf = body_length/2 #front axle to COM
L = Lr + Lf
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.3)

#for the keyboard control
current_steering_angle = 0.0  # in radians
current_velocity = 0.0        # in pixels/sec
steering_rate = np.radians(1.5)  # radians per frame
acceleration = 10             # pixels/sec^2 per frame
max_velocity = 1000
max_steering = np.radians(50)

#create robot parts
body = patches.Rectangle(xy=(-body_length/2, -body_width/2),width=body_length, height=body_width, edgecolor='black', facecolor='blue')
left_drive_wheel = patches.Rectangle(xy=(-wheel_thickness/2, -wheel_radius), width=wheel_radius*2,height=wheel_thickness, edgecolor='black', facecolor='green')
right_drive_wheel = patches.Rectangle(xy=(-wheel_thickness/2, -wheel_radius), width=wheel_radius*2,height=wheel_thickness, edgecolor='black', facecolor='green')
ideal_drive_wheel = patches.Rectangle(xy=(-wheel_thickness/2, -wheel_radius), width=wheel_radius*2,height=wheel_thickness, edgecolor='black', facecolor='purple')
left_steering_wheel = patches.Rectangle(xy=(-wheel_thickness/2, -wheel_radius), width=wheel_radius*2,height=wheel_thickness, edgecolor='black', facecolor='yellow')
right_steering_wheel = patches.Rectangle(xy=(-wheel_thickness/2, wheel_radius), width=wheel_radius*2,height=wheel_thickness, edgecolor='black', facecolor='yellow')
ideal_steering_wheel = patches.Rectangle(xy=(-wheel_thickness/2, -wheel_radius), width=wheel_radius*2,height=wheel_thickness, edgecolor='black', facecolor='orange')

ax.add_patch(body)
ax.add_patch(left_drive_wheel)
ax.add_patch(right_drive_wheel)
ax.add_patch(ideal_drive_wheel)
ax.add_patch(left_steering_wheel)
ax.add_patch(right_steering_wheel)
ax.add_patch(ideal_steering_wheel)

ax.set_xlim(-1600, 1600) #wro dimension
ax.set_ylim(-1600, 1600) #wro dimension
# ax.set_xlim(-400, 400)
# ax.set_ylim(-400, 400)
ax.set_aspect('equal')
ax.grid(True)
ax.autoscale(False)

#sliders
axSteering = plt.axes([0.25, 0.2, 0.65, 0.03])
steering_slider = Slider(axSteering, 'steering angle', -50, 50, valinit=0)
axv = plt.axes([0.25, 0.15, 0.65, 0.03])
v_slider = Slider(axv, 'linear velocity', 0, 1000, valinit=0)

#reset button
axreset = axwr = plt.axes([0.25, 0.1, 0.65, 0.03])
button = Button(axreset, 'Reset', hovercolor='0.975')

def reset(event):
    steering_slider.reset()
    v_slider.reset()

#for animation
def animate(frame):
    global current_steering_angle, current_velocity
    # Update steering
    if keyboard.is_pressed('left'):
        current_steering_angle += steering_rate
    elif keyboard.is_pressed('right'):
        current_steering_angle -= steering_rate
    else:
        # Optional: auto-return to center (like real steering wheel)
        current_steering_angle *= 0.9  # damping factor
    
    # Clamp steering angle
    current_steering_angle = np.clip(current_steering_angle, -max_steering, max_steering)

    # Update velocity
    if keyboard.is_pressed('up'):
        current_velocity += acceleration
    elif keyboard.is_pressed('down'):
        current_velocity -= acceleration
    else:
        # Optional: gradual deceleration (friction)
        current_velocity *= 0.95

    # Clamp velocity
    current_velocity = np.clip(current_velocity, -max_velocity, max_velocity)

    # Use updated values
    steering_angle = current_steering_angle
    v = current_velocity

    #kinematic bicycle equation of motions
    beta = np.arctan((Lr / L) * np.tan(steering_angle))
    dx = v * np.cos(pose.theta+ beta) * dt
    dy = v * np.sin(pose.theta+ beta) * dt
    dtheta = v/L * np.cos(beta) * (np.tan(steering_angle))* dt

    pose.x += dx
    pose.y += dy
    pose.theta += dtheta

    pose.x = np.clip(pose.x, -1600, 1600)
    pose.y = np.clip(pose.y, -1600, 1600)

    #local-to-world wheel positions center bach kit tha bottom left corner ey te 
    left_drive_wheel_local = np.array([-body_length/2,body_width/2])
    right_drive_wheel_local = np.array([-body_length/2,-body_width/2])
    ideal_drive_wheel_local = np.array([-body_length/2, 0])
    left_steering_wheel_local = np.array([+body_length/2,body_width/2])
    right_steering_wheel_local = np.array([+body_length/2,-body_width/2])
    ideal_steering_wheel_local = np.array([body_length/2, 0])

    if steering_angle != 0:
        turning_radius = L / np.tan(steering_angle)
        R_inner = turning_radius - body_width / 2
        R_outer = turning_radius + body_width / 2
        inner_steering_angle = np.arctan(L / R_inner)
        outer_steering_angle = np.arctan(L / R_outer)
    else:
        inner_steering_angle = outer_steering_angle = 0.0

    R = rotation_matrix_2d(np.degrees(pose.theta))
    robot_center = np.array([pose.x, pose.y])
    left_drive_wheel_world = R @ left_drive_wheel_local + robot_center
    right_drive_wheel_world = R @ right_drive_wheel_local + robot_center
    ideal_drive_wheel_world = R @ ideal_drive_wheel_local + robot_center
    left_steering_wheel_world = R @ left_steering_wheel_local + robot_center
    right_steering_wheel_world = R @ right_steering_wheel_local + robot_center
    ideal_steering_wheel_world = R @ ideal_steering_wheel_local + robot_center

    #update patch positions with the bottom left corner offset for patch bro yknow how it is gang
    body.set_xy((pose.x - body_length / 2, pose.y - body_width / 2))
    left_drive_wheel.set_xy((left_drive_wheel_world[0]-wheel_radius, left_drive_wheel_world[1]-wheel_thickness/2))
    right_drive_wheel.set_xy((right_drive_wheel_world[0]-wheel_radius, right_drive_wheel_world[1]-wheel_thickness/2))
    ideal_drive_wheel.set_xy((ideal_drive_wheel_world[0]-wheel_radius, ideal_drive_wheel_world[1]-wheel_thickness/2))
    left_steering_wheel.set_xy((left_steering_wheel_world[0]-wheel_radius, left_steering_wheel_world[1]-wheel_thickness/2))
    right_steering_wheel.set_xy((right_steering_wheel_world[0]-wheel_radius, right_steering_wheel_world[1]-wheel_thickness/2))
    ideal_steering_wheel.set_xy((ideal_steering_wheel_world[0]-wheel_radius, ideal_steering_wheel_world[1]-wheel_thickness/2))

    #update patch orientation
    t_b = mpl.transforms.Affine2D().rotate_around(pose.x, pose.y, pose.theta) + ax.transData
    t_ldw = mpl.transforms.Affine2D().rotate_around(left_drive_wheel_world[0], left_drive_wheel_world[1], pose.theta) + ax.transData
    t_rdw = mpl.transforms.Affine2D().rotate_around(right_drive_wheel_world[0], right_drive_wheel_world[1], pose.theta) + ax.transData
    t_idw = mpl.transforms.Affine2D().rotate_around(ideal_drive_wheel_world[0], ideal_drive_wheel_world[1], pose.theta) + ax.transData
    if keyboard.is_pressed('left'):
        t_lsw = mpl.transforms.Affine2D().rotate_around(left_steering_wheel_world[0], left_steering_wheel_world[1], pose.theta + inner_steering_angle) + ax.transData
        t_rsw = mpl.transforms.Affine2D().rotate_around(right_steering_wheel_world[0], right_steering_wheel_world[1], pose.theta + outer_steering_angle) + ax.transData
    elif keyboard.is_pressed('right'):
        t_lsw = mpl.transforms.Affine2D().rotate_around(left_steering_wheel_world[0], left_steering_wheel_world[1], pose.theta + outer_steering_angle) + ax.transData
        t_rsw = mpl.transforms.Affine2D().rotate_around(right_steering_wheel_world[0], right_steering_wheel_world[1], pose.theta + inner_steering_angle) + ax.transData
    else:
        t_lsw = mpl.transforms.Affine2D().rotate_around(left_steering_wheel_world[0], left_steering_wheel_world[1], pose.theta + steering_angle) + ax.transData
        t_rsw = mpl.transforms.Affine2D().rotate_around(right_steering_wheel_world[0], right_steering_wheel_world[1], pose.theta + steering_angle) + ax.transData
    t_isw = mpl.transforms.Affine2D().rotate_around(ideal_steering_wheel_world[0], ideal_steering_wheel_world[1], pose.theta + steering_angle) + ax.transData
    	
    body.set_transform(t_b)
    left_drive_wheel.set_transform(t_ldw)
    right_drive_wheel.set_transform(t_rdw)
    ideal_drive_wheel.set_transform(t_idw)
    left_steering_wheel.set_transform(t_lsw)
    right_steering_wheel.set_transform(t_rsw)
    ideal_steering_wheel.set_transform(t_isw)

    steering_slider.set_val(np.degrees(current_steering_angle))
    v_slider.set_val(abs(current_velocity))  # just for display


# start animation
ani = FuncAnimation(fig, animate, interval=10)
button.on_clicked(reset)
plt.show()
