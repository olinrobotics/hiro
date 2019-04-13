# text-to-portrait
Allow an UR5 arm to draw any picture that a user wants.

## Description
Text to portrait is a script that allows users to input what image they want Castor or Pollux (UR5 arms) to draw. Using the google custom search api, it downloads the first 10 images of a specific search query. It then uses opencv to extract the edges of the images and translates it to coordinates for the arm to move in. A gui is also displayed to help users select which image to draw and to control certain parameters like thresholds and drawing speed.

<img src="pikachu_1.png" width="500px"/>
<img src="pikachu_2.png" width="500px"/>
<img src="katamari.png" width="500px"/>

## Demo Video
[![Watch the video](preview.jpg)](https://youtu.be/_iO4P9R-ilU)

## Tools Used
* Ubuntu 16.04
* Ros-kinetic
* Python
* OpenCV
* Matplotlib

## Team Members
Richard Gao  
HK Rho