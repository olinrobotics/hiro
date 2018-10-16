"""
By Khang Vu, 2017
Last modified Dec 13, 2017

This script generates digit images from fonts
All basic fonts are stored in folder "./fonts"
Use either chosen fonts in "fonts_list.txt" or all fonts in system fonts "./fonts"
"""
import glob
import os
from PIL import Image, ImageDraw, ImageFont

# Font size
fontSize = 90

# Output image size
imgSize = (150, 150)

# Background color of the image
background_color = (0, 0, 0)

# Text color of the character
char_color = (255, 255, 255)

# Position of a char/digit on the image
position = (15, 15)

# All images will be stored in 'font_images' directory under current directory
dataset_path = os.path.join(os.getcwd(), 'font_images')
if not os.path.exists(dataset_path):
    os.makedirs(dataset_path)

# Store all chars/digits in an array
chars_list = []
for d in range(0, 10):
    chars_list.append(str(d))

# Handle desired fonts
fhandle = open('fonts_list.txt', 'r')
desired_fonts = []
for line in fhandle:
    desired_fonts.append(line.rstrip('\n'))

# Handle all system fonts
sys_fonts = glob.glob("./fonts/*.ttf")


def take_all_fonts():
    font_count = 0
    for sys_font in sys_fonts:
        font_count += 1
        path = sys_font
        desired_font = ImageFont.truetype(path, fontSize)

        for ch in chars_list:
            # For each char, draw a blank image and put text on the image
            image = Image.new("RGB", imgSize, background_color)
            draw = ImageDraw.Draw(image)
            draw.text(position, ch, char_color, font=desired_font)

            # Save the image
            file_name = '%c_%i.jpg' % (ch, font_count)
            file_name = os.path.join(dataset_path, file_name)
            image.save(file_name)

    print "Total fonts: %d" % font_count


def take_desired_fonts():
    font_count = 0
    for desired_font in desired_fonts:
        # Only pick desired fonts
        f_lower = desired_font.lower()
        found = False
        for sys_font in sys_fonts:
            s_lower = sys_font.lower()

            # Check if a desired font is in system fonts
            if f_lower in s_lower:
                found = True
                font_count += 1
                path = sys_font
                desired_font = ImageFont.truetype(path, fontSize)

                for ch in chars_list:
                    # For each char, draw a blank image and put text on the image
                    image = Image.new("RGB", imgSize, background_color)
                    draw = ImageDraw.Draw(image)
                    draw.text(position, ch, char_color, font=desired_font)

                    # Save the image
                    file_name = '%c_%i.jpg' % (ch, font_count)
                    file_name = os.path.join(dataset_path, file_name)
                    image.save(file_name)

        if not found:
            print "Cannot find font %s" % f_lower

    print "Total fonts: %d" % font_count


# Choose either all fonts or desired fonts
take_all_fonts()
# take_desired_fonts()
