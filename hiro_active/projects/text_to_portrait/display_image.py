"""
By Richard Gao & HK Rho, 2018
This is the master script of displaying the image outline in gui
This script will:

Dependencies:


To use:
- Open Terminal and run the code below:
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=10.42.0.175
rosrun hiro ur5_arm_node.py _robot_ip:=10.42.0.175
python2 display_image.py
"""

import os, shutil, time, math
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation
from images import fetch_image
import pprint
from itertools import islice
from move_arm import DrawArm
from close_gripper import GripperController

# Define Constants
max_lowThreshold = 100
INIT_LOWER = 100
INIT_UPPER = 200
# Define origin/home set of robot arm
ARTBOARD_WIDTH = 600
ARTBOARD_HEIGHT = 400
X_AXIS_ARM = 259 / 2
Y_AXIS_ARM = 795

# Global Variables
# img: current image file
# draw_img: current plot image
img = draw_img = slthresh = None
suthresh = fig = img_num = None
height = width = ax = None
coordinates = coord_count = None
animate_img = sbutton = dbutton = cbutton = ani = continue_drawing = None
closed = False
speed = 15
plots = []

def load_images(path):
    '''
    Walks through directory given by path and reads all images into a list

    path: file path to directory to read images from

    returns: list of images
    '''
    images = []

    for filename in os.listdir(path):
        if not filename.endswith('.jpg'):
            continue

        # Read in image
        img = cv.imread(path + filename)

        # Skip over any corrupted images
        if img is None:
            continue

        images.append(img)
    
    return images

def reset_images():
    '''
    Resets the images directory by deleting all files and directories within
    '''
    for filename in os.listdir('images'):
        shutil.rmtree('images/' + filename)

def update(val):
    global slthresh
    global suthresh
    global img
    global draw_img

    # Get values from the current plot
    lower = slthresh.val
    upper = suthresh.val

    # Process image with new values
    edge_img = get_edge(img, lower, upper)

    # Update plot
    draw_img.set_data(edge_img)

    # Rerender
    fig.canvas.draw_idle()

def change_speed(val):
    global speed
    
    speed = int(val)

def get_coordinates(img, lower, upper):
    global coordinates

    # Get edges using canny edge detection
    edges = cv.Canny(img, lower, upper)

    # Get xy coordinates
    indices = np.where(edges != [0])
    coordinates = zip(indices[1], indices[0])
    
    return coordinates

def get_edge(img, lower, upper):
    '''
    Extracts the edges from the image using Canny Edge Detection

    img: image to extract edges from
    lower: lower threshold
    upper: upper threshold

    returns: image to plot with drawn edges
    '''
    global height
    global width

    coordinates = get_coordinates(img, lower, upper)

    # Create a black image of original image height and width
    edge_img = np.zeros((height, width, 3), np.uint8)

    # Draw Points
    for x, y in coordinates:
        cv.line(edge_img, (x, y), (x+1, y+1), (255,255,255), 2)
    
    return edge_img

def animate(event):
    global height
    global width
    global draw_img
    global fig
    global coord_count
    global animate_img
    global ani
    global coordinates
    
    if not ani is None:
        return

    # Reset Canvas
    animate_img = np.zeros((height, width, 3), np.uint8)
    plt.axes([0.27, 0.25, 0.6, 0.6])
    draw_img = plt.imshow(animate_img, cmap = 'gray', interpolation = 'bicubic')
    ax.relim()
    ax.autoscale_view(True,True,True)
    fig.canvas.draw_idle()

    coord_count = 0

    ani = animation.FuncAnimation(fig, updatefig, frames=len(coordinates), interval=1, repeat=False)
    ani.running = True

    plt.show() 

def updatefig(frame):
    global coord_count
    global coordinates
    global animate_img

    # Check if animation finished
    if coord_count >= len(coordinates):
        print('Done drawing animation')
        return 0

    x, y = coordinates[coord_count]
    cv.line(animate_img, (x, y), (x+1, y+1), (255,255,255), 2)
    draw_img.set_data(animate_img)

    coord_count += 1

    return draw_img

def stop(event):
    global ani
    global sbutton
    
    if ani is None:
        return

    if ani.running:
        ani.event_source.stop()
        sbutton.label.set_text("Resume")
    else:
        ani.event_source.start()
        sbutton.label.set_text("Stop")
    ani.running ^= True

def draw(event):
    global continue_drawing
    global dbutton

    if not continue_drawing is None:
        print('Stop Drawing')
        dbutton.label.set_text("Draw")
        continue_drawing = None
        fig.canvas.draw_idle()
    else :
        print('Starting to draw portrait')
        dbutton.label.set_text("Cancel")
        continue_drawing = True
        fig.canvas.draw_idle()
        optimize_draw()

def close_gripper(event):
    global closed
    global cbutton

    if not closed:
        gripper.close()
        closed = True
        sbutton.label.set_text("Open")
        print('Open Gripper')
    else:
        gripper.open()
        closed = False 
        sbutton.label.set_text("Close")
        print('Close Gripper')

def reset(event):
    '''
    Resets the slider values to default/initial
    '''
    global slthresh
    global suthresh

    slthresh.reset()
    suthresh.reset()

def prev_img(event):
    global img_num

    if (img_num <= 0):
        print('Cannot get previous image')
        return
    
    img_num -= 1

    change_img(img_num)

def next_img(event):
    global img_num

    if (img_num >= len(images) - 1):
        print('Cannot get next image')
        return

    img_num += 1

    change_img(img_num)

def change_img(img_num):
    global slthresh
    global suthresh 
    global img 
    global draw_img
    global fig
    global height
    global width
    global ax
    global ani
    global animate_img
    global sbutton

    # Sets current image
    img = images[img_num]

    # Sets height and width per image
    height, width = images[img_num].shape[:2]
    
    print(height, width)

    # Updates plots cache
    if type(plots[img_num]) != 'numpy.ndarray':
        edge_img = get_edge(images[img_num], INIT_LOWER, INIT_UPPER)
        plots[img_num] = edge_img
    else :
        edge_img = plots[img_num]

    # Update plot/window
    plt.axes([0.27, 0.25, 0.6, 0.6])
    draw_img = plt.imshow(edge_img, cmap = 'gray', interpolation = 'bicubic')
    ax.relim()
    ax.autoscale_view(True,True,True)
    fig.canvas.draw_idle()

    # Reset global variables
    slthresh.reset()
    suthresh.reset()

    if not ani is None:
        ani.event_source.stop()
        ani = animate_img = None
    
    sbutton.label.set_text("Stop")

def plot_image(name):
    global slthresh
    global suthresh 
    global img 
    global draw_img
    global fig
    global ax
    global sbutton
    global dbutton
    global cbutton

    # Setup plot
    # fig, ax = plt.subplots(num=None, figsize=(16, 12), dpi=80, edgecolor='k')
    fig, ax = plt.subplots(num=None, dpi=80, edgecolor='k')    
    plt.axis('off')
    fig.canvas.set_window_title('Thresholds Picker')    
    # plt.subplots_adjust(left=0.25, bottom=0.25)
    plt.title('PORTRAIT RENDERING: ' + name)

    plt.axes([0.27, 0.25, 0.6, 0.6])
    draw_img = plt.imshow(edge_img, cmap = 'gray', interpolation = 'bicubic')

    #--------Sliders----------
    # Add sliders to current figure
    lthreshax = plt.axes([0.25, 0.2, 0.65, 0.03])
    uthreshax = plt.axes([0.25, 0.15, 0.65, 0.03])
    speedax = plt.axes([0.25, 0.1, 0.65, 0.03])

    # Create sliders
    slthresh = Slider(lthreshax, 'Lower Threshold', 0, 300, valinit=INIT_LOWER, facecolor='#e5d9c4')
    suthresh = Slider(uthreshax, 'Upper Threshold', 0, 300, valinit=INIT_UPPER, facecolor='#e5d9c4')
    sspeed = Slider(speedax, 'Draw Speed', 1, 100, valinit=15, facecolor='#e5d9c4')

    # Add event handlers
    slthresh.on_changed(update)
    suthresh.on_changed(update)
    sspeed.on_changed(change_speed)

    #--------BUTTONS----------
    # Add buttons to current figure
    drawax = plt.axes([0.025, 0.55, 0.1, 0.04])
    closetax = plt.axes([0.025, 0.5, 0.1, 0.04])
    animateax = plt.axes([0.025, 0.45, 0.1, 0.04])
    stopax = plt.axes([0.025, 0.4, 0.1, 0.04])
    resetax = plt.axes([0.025, 0.35, 0.1, 0.04])

    prevax = plt.axes([0.5, 0.05, 0.1, 0.04])
    nextax = plt.axes([0.6, 0.05, 0.1, 0.04])


    # Create buttons
    dbutton = Button(drawax, 'Draw', color='#0aeb7e', hovercolor='#08bc64')
    cbutton = Button(closetax, 'Close', color='#d02edd', hovercolor='#a624b0')
    abutton = Button(animateax, 'Animate', color='#a9c2ee', hovercolor='#879bbe')
    sbutton = Button(stopax, 'Stop', color='#ff0074', hovercolor='#cc005c')
    rbutton = Button(resetax, 'Reset', color='#f28500', hovercolor='#c16a00')

    pbutton = Button(prevax, 'Prev', color='#99cc33', hovercolor='#7aa328')
    nbutton = Button(nextax, 'Next', color='#99cc33', hovercolor='#7aa328')

    # Add event handlers
    dbutton.on_clicked(draw)
    cbutton.on_clicked(close_gripper)
    abutton.on_clicked(animate)
    sbutton.on_clicked(stop)
    rbutton.on_clicked(reset)

    pbutton.on_clicked(prev_img)
    nbutton.on_clicked(next_img)

    plt.show()

def optimize_draw():
    global height
    global width
    global img
    global slthresh
    global suthresh
    global speed

    # Setup UR5 arm
    arm = DrawArm()

    # Get values from the current plot
    lower = slthresh.val
    upper = suthresh.val

    print('hw', height, width)
    # Scale picture to fix the bounds of the artboard
    # if height > width:
    #     print('Scale to h')
    #     points_left = get_coordinates(scale_to_artboard(img, h=ARTBOARD_WIDTH), lower, upper)
    # else:
    #     print('Scale to w')
    #     points_left = get_coordinates(scale_to_artboard(img, w=ARTBOARD_WIDTH), lower, upper)

    points_left = get_coordinates(scale_to_artboard(img, width, height), lower, upper)


    print("Artboard Dimesions: ", ARTBOARD_WIDTH, ARTBOARD_HEIGHT)

    # Image to display on window
    edge_img = np.zeros((ARTBOARD_HEIGHT, ARTBOARD_WIDTH, 3), np.uint8)

    # Fill in np.array 2d array with 0 and 1
    bin_img = np.zeros((ARTBOARD_HEIGHT, ARTBOARD_WIDTH), np.int32)
    for x, y in points_left:
        bin_img[y, x] = 1

    # Start drawing in hover position
    arm.move_gesture('portrait_hover') 
    time.sleep(10)   

    point_length = len(points_left)

    # Start at first point in points left -> Check surrounding for points of value 1 -> draw polyline while popping out points from points left -> repeat 
    while not continue_drawing is None and len(points_left) > 0 :
        line = []
        find_line(0, points_left, bin_img, line)

        if len(line) <= speed:
            continue
        elif len(line) == 1:
            x, y = line[0]
            line.append((x+1, y+1))

        print('----------Drawing Line:-----------')
        print('Progress: ' + str(point_length - len(points_left)) + '/' + str(point_length))        
        print(line)

        it = iter(enumerate(line))

        for idx, point in it: 
            print('**INDEX: ' + str(idx) + '**')
            if idx >= len(line) - speed:
                break

            # Draw to screen
            cv.line(edge_img, point, line[idx+speed], (255,255,255), 2)

            #---------Draw from point to line[idx + 1]---------
            start = transform_to_arm_axis(point[0], point[1])
            end = transform_to_arm_axis(line[idx+speed][0], line[idx+speed][1])

            time.sleep(2)

            # Draw to paper
            arm.draw_line(start, end)

            next(islice(it, speed, speed), None)

        time.sleep(4)
        arm.move_gesture('portrait_hover')
        
        # Draw to window
        plt.axes([0.27, 0.25, 0.6, 0.6])
        plt.imshow(edge_img, cmap = 'gray', interpolation = 'bicubic')
        ax.relim()
        ax.autoscale_view(True,True,True)
        fig.canvas.draw_idle()
        plt.pause(.000001)

        time.sleep(5) 
        # break
    
    arm.move_gesture('portrait_reset')
    arm = None

    if not continue_drawing is None:
        plt.close('all')
        print('----------Done Drawing!-----------')

def transform_to_arm_axis(x, y):
    # Transform x,y coordinate to fit UR5 coordinate system 
    y_arm = x - Y_AXIS_ARM
    x_arm = y - X_AXIS_ARM

    return (x_arm, y_arm)

def scale_to_artboard(image, w, h):
    # Scale image to fit artboard size

    # global height
    # global width

    # dim = None

    # if w is None:
    #     r = h / float(height)
    #     dim = (int(width * r), h)
    # else:
    #     r = w / float(width)
    #     dim = (w, int(height * r))
    
    # width = dim[0]
    # height = dim[1]

    # print('Scaled Dimensions: ' + str(dim))

    # resized = cv.resize(image, dim, interpolation=cv.INTER_AREA)
    global ARTBOARD_HEIGHT
    global ARTBOARD_WIDTH

    area = ARTBOARD_HEIGHT * ARTBOARD_HEIGHT
    aspect_ratio = w / h
    height = int(math.sqrt(area/ aspect_ratio))
    width = int(height * aspect_ratio)

    dim = (width, height)
    
    print('Scaled Dimensions: ' + str(dim))

    resized = cv.resize(image, dim, interpolation=cv.INTER_AREA)

    return resized

# Recursively find nearby points to draw
def find_line(current, points_left, bin_img, line):
    '''
    Find line to draw by looking at nearby points. Edits the line list

    current: current point(index) in points_left to look at
    points_left: list of x, y coordinates
    bin_img: 2d array that holds binary data on where a point is
    line: current list of points to draw
    '''

    height = ARTBOARD_HEIGHT
    width = ARTBOARD_WIDTH

    if not len(points_left):
        return 

    # tolerance level
    # 1 = 3

    if current is None:
        return

    x, y = points_left[current]

    if x is None or y is None:
        return

    if (x > 0 and x + 1 < width) and (y > 0 and y + 1 < height): # Middle
        # Check all eight that touch point
        if bin_img[y+1, x-1]: #down-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #down
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #down-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x+1]: #mid-right
            add_point(x+1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #top-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #top
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #top-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x-1]: #mid-left
            add_point(x-1, y, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: middle')
            return
    elif (x > 0 and x + 1 < width) and (y + 1 < height): # top edge
        if bin_img[y, x-1]: #mid-left
            add_point(x-1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #down-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #down
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #down-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x+1]: #mid-right
            add_point(x+1, y, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: top edge')
            return
    elif (x > 0 and x + 1 < width) and (y > 0): # bottom edge
        if bin_img[y, x-1]: #mid-left
            add_point(x-1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #top-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #top
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #top-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x+1]: #mid-right
            add_point(x+1, y, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: bottom edge')
            return
    elif (x + 1 < width) and (y > 0 and y + 1 < height): # left edge
        if bin_img[y+1, x]: #top
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #top-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x+1]: #mid-right
            add_point(x+1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #down-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #down
            add_point(x, y+1, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: left edge')
            return
    elif (x > 0) and (y > 0 and y + 1 < height): # right edge    
        if bin_img[y+1, x]: #top
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #top-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x-1]: #mid-left
            add_point(x-1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #down-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #down
            add_point(x, y+1, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: right edge')
            return
    elif (x + 1 < width) and (y + 1 < height): # top-left edge
        if bin_img[y, x+1]: #mid-right
            add_point(x+1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #down-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #down
            add_point(x, y+1, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: top left edge')
            return
    elif (x > 0) and (y + 1 < height): # top-right edge
        if bin_img[y, x-1]: #mid-left
            add_point(x-1, y, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #down-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x]: #down
            add_point(x, y+1, points_left, bin_img, current, line)
        else:
            
            print('End of line: top right edge')
            return
    elif (x + 1 < width) and (y > 0): # bottom-left edge
        if bin_img[y+1, x]: #top
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x+1]: #top-right
            add_point(x+1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x+1]: #mid-right
            add_point(x+1, y, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: bottom left edge')
            return
    elif (x > 0) and (y > 0): # bottom-right edge
        if bin_img[y+1, x]: #top
            add_point(x, y+1, points_left, bin_img, current, line)
        elif bin_img[y+1, x-1]: #top-left
            add_point(x-1, y+1, points_left, bin_img, current, line)
        elif bin_img[y, x-1]: #mid-left
            add_point(x-1, y, points_left, bin_img, current, line)
        else:
            del points_left[current]
            line.append((x, y))
            bin_img[y, x] = 0
            print('End of line: bottom right edge')
            return
    else:
        print('Cannot draw line. Error. X Y not within bounds')
        return

def add_point(x, y, points_left, bin_img, current, line):
    # Updates lists and adds point to line, starts next level of recursion by calling find_line on the next point

    del points_left[current]
    line.append((x, y))
    bin_img[y, x] = 0

    new_current = find_index(x, y, points_left)

    find_line(new_current, points_left, bin_img, line)

def find_index(x, y, points_left):
    # Finds the index of the x and y passed in within the points left list
    
    if len(points_left) <= 0:
        return 0

    # Find new current
    for idx, coords in enumerate(points_left):
        x_new, y_new = coords
        if x_new == x and y_new == y:
            # Add idx to current to return index relative to points_left
            return idx
if __name__ == '__main__':
   
    print('Enter Name:')
    name = raw_input()

    # Remove previous images and their directory
    reset_images()

    # Download images from google
    fetch_image(name)

    path = 'images/' + name + '/'

    if not os.path.exists(path):
        exit(0)
    
    images = load_images(path)
    img = images[0]
    height, width = images[0].shape[:2]
    img_num = 0

    # Put in temp values of 0
    for i in images:
        plots.append(0)

    # Process image
    edge_img = get_edge(images[0], INIT_LOWER, INIT_UPPER)
    plots[0] = edge_img

    gripper = GripperController()

    plot_image(name.upper())

    