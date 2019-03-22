---
date: 2017-04-29 04:00:00 +0000
layout: project
title: Math Problem Solving
sub_heading: Edwin's Mathem Problem Solving code gives Edwin the capability to read
  a math problem, solve it, and write the answer in the proper position.
tags: []
banner_image: ''
description: Edwin's Mathem Problem Solving code gives Edwin the capability to read
  a math problem, solve it, and write the answer in the proper position. This project
  aims to make Edwin more organic by slowing down his problem solving and showing
  more of the steps, so to speak. Additionally, the angle of "Edwin does your homework"
  is appealing for demos.
slug: ''
image_slider: []
members:
- Victoria McDermott
- Hannah Kolano
- Connor Novak
- Matthew Brucker

---
## Description

<p>Edwin's Mathem Problem Solving code gives Edwin the capability to read a math problem, solve it, and write the answer in the proper position. This project aims to make Edwin more organic by slowing down his problem solving and showing more of the steps, so to speak. Additionally, the angle of "Edwin does your homework" is appealing for demos.</p>

## Tools Used:

* [Ubuntu 14.04](https://www.ubuntu.com/ "Ubuntu 14.04")
* [OpenCV Vision Library](http://opencv.org/)
* Atom Text Editor
* ROS Indigo
* PyCharm 2017.1
* Endoscope USB Camera

## Technical Explanation

### Part 1: Reading Equation

<p>Edwin reads text by using the OpenCV library to perform image processing and select the individual letters as contours. In the event that a symbol has more than one line (=, i, %), each line is saved as an individual contour.</p>

#### 1.1: Image Pre-Processing

<p>To find symbols in the image, the program first blurs the image through a Gaussian blur. This removes some noise from the image. The program then changes the image to grayscale. This operation changes an image from an NxNx3 matrix (where every entry is the red, green, or blue value of an xy-pixel) to an NxN matrix (where every entry is the intensity of an xy-pixel). The next step is to apply two different sharpening kernels to make the letters stand out more distinctly.</p>

<p>Once the image has been cleaned in this fashion, the program thresholds it, turning each pixel either black or white based on a chosen value. OpenCV provides a dynamic thresholding feature that chooses the threshold value based on relative light values in the image. Now, the image is ready to contour.</p>

#### 1.2 Contouring & ROIs

<p>OpenCV has many convenient contouring tools. The program uses basic contouring on the binary image and lists the total contours found in the image. For each resulting contour, a bounding rectangle is created around the contour. The pieces of the image located inside the rectangles are referred to as Regions of Interest (ROIs). These ROIs are then deskewed(straightened into a single frame of reference) and resized (so as to all be the same size), allowing more accurate comparison. The relative location of the ROI is also saved for use in step 1.4. </p>

#### 1.3 Initial Symbol Recognition (SVM)

Once the ROIs are all processed and saved, they are fed into an SVM, which is a Support Vector Machine, created using OpenCV. An SVM is one method of classification of many different items which takes training data and calculates a hyperplane that divides different samples. Then, for a new piece of data, the SVM determines which grouping the data falls in and classifies the data accordingly.<br

For more specific information on SVMS, read the OpenCV documentation article [Introduction to Support Vector Machines](http://docs.opencv.org/2.4/doc/tutorials/ml/introduction_to_svm/introduction_to_svm.html "Introduction to Support Vector Machines")

#### 1.4: Ambiguous Contour Resolution

When defining symbols through individual contours, certain symbols are impossible to distinguish. For example, an equality sign (=) is characterized as two separate horizontal lines. Individually, this means that all equals signs are labeled as two subtraction signs (-). To fix this problem, the program sorts out "ambiguous contours" as it finds them - any horizontal or vertical lines, as well as dots, are separated out into a separate list of tuples, with a tuple being defined as the contour, the contour's x-position, and the contour's y-position. Because ambiguous symbols are entirely built out of ambiguous contours, looping through the ambiguous contour list and comparing their positions can allow construction of ambiguous symbols. As an example, suppose that a dot is selected. A dot can be part of 5 different ambiguous symbols: a lowercase i (i), an exclamation point (!), a colon (:), a semicolon (;), or a division sign. Each of these symbols is different in some way, so an if-tree will be able to distinguish between them. If another dot has an x-position nearly identical to the chosen dot, and a y-position that is close to the dot, the character is probably a colon. Using this method, all fuzzy symbols are added to the list of discovered contours.

#### 1.5: Overall Classification

As a method of increasing the accuracy of the recognition, the program creates a ratio between symbols that are alphabetic and symbols that are numeric. This allows the program to distinguish between equations and sentences, and classify the characters accordingly. Once a sequence of symbols is classified as alphabetic or numeric, the ROIs are passed into either or two SVMs trained only on the respective class of symbol. The results are published to a ROS topic for the second part of Math Problem Solver to use.

### Part 2: Solving 

After a string of digits is received from Part 1, there is a calculator part of the program that parses and solves whatever expression is given to it. 

#### 2.1 Parsing for Sense

First, the program checks that the expression it was given makes sense. If there's anything the program can't handle, like a double operation sign (++), two different variables in an equation (x+y=1), more than one instance of a variable (x^2=x), a non-mathematical character (@, &, $), or a weird variable (v, k, c), then it will not execute the rest of the program. Instead it raises an error and prints what the problem is.

There are a couple other things that the calculator needs to parse first. For instance, for a human it would be obvious that 5x and 5(x) both mean 5*x; however, the program still needs to go in and insert a multiplication sign to understand what that means. Part of the code goes in and puts multiplication signs between variables/parenthesis and their coefficients. Similarly, a caret (^) that would be input by a user needs to be switched to (**) for the calculator to evaluate it. 

Since this program only deals with math, having letters like 'l' or 'o' doesn't make sense. Therefore, the program takes those letters (if the image processing wrongly interprets them) and changes them to the digit or operation most likely to be misrepresented. 

#### 2.2 Solving Algebra

Whereas a function for evaluating mathematical expressions is built into Python, a function for solving an algebraic expression is not. Python can not inherently solve 'x+1=2', let alone something more complicated like '(x+2)/3=1'. How Edwin will approach this problem is to break the equation into a tree based on the order of operations, and then solve it segment by segment until only the variable is left.

For a more complicated expression, such as 3x+1, the nodes must follow the order of operations:

![](/uploads/3x 1tree.PNG)

Unless parentheses are involved, such as in (x+2)/3:

![](/uploads/(x 2)div3tree.PNG)

##### 2.2.1 Building the Tree

At the most basic level, building the tree is a simple idea. Find the next operator in the reverse order of operations, and create a node at that operator. 

As a first step, it would check if there are any minuses or pluses. If there are, it finds the rightmost one (order of operations goes left-to-right, so it must solve right-to-left) and divides the equation into two parts at the operator. The tree is stored as a tuple with three parts: (operator, left side, right side). For example, the equation 3x+1 would become (+, 3x, 1). This process is recursive; it repeats itself for the second and third element of this tuple until there are no more operators. 

If there are more no pluses or minuses, it checks for multiplication and division. If there were any minuses or pluses in the previous iteration, this element would be inside a tuple, creating a tuple inside of a tuple. For this example, the tree would become like so: (+, (\\*, 3, x), 1). After it has found all of those, it moves on to exponents. The process repeats until there are no more operators in the second or third elements of any tuple. Thereby an equation like 1/2+3x^2-6 would make a tree like so: (-, (+, (/, 1, 2), (\\*, 3, (^, x, 2))), 6).  

##### 2.2.2 Special Case: Parentheses 

Parenthesis cannot be handled the same as a normal operation. If an operation is in parenthesis, regardless of where it is in the order of operations, it would hypothetically go first. Therefore, the code must check for instances of parenthesis to properly build a tree. 

The code deals with parenthesis by replacing the expression inside with a placeholder variable. In the previous example (x+2)/3, the code would see that there is a parenthesis. It would then choose a placeholder variable, such as 'p' or 'q' and replace the expression with that placeholder. Thereby the new equation is p/3. It then stores 'p' as a key in a dictionary with 'x+2' as the value, so that it can be retrieved when it is needed again. The code then makes the rest of the equation into a tree: (/, p, 3). Once the placeholder variable is by itself, the code makes the parenthesis expression into a tree: (+, x, 2), and replaces the placeholder variable with it to create the correct tree: (/, (+, x, 2), 3). 

It can do this with multiple parenthesis because there is a counter that keeps track of which open parenthesis go with which close parenthesis. The counter starts at 1 when it hits an open parenthesis. Then, it works along the expression until it finds another parenthesis. If it finds another parenthesis, it adds 1 if it is an open, and subtracts one if it is a close; therefore, when the counter is 0, that is the matching close parenthesis to the given open parenthesis. 

Nested parenthesis might store placeholder variables in their expression. With an expression like ((1+x)/2)\\*3, a placeholder variable 'p' would first be given to 1+x, the inner parenthesis. Then it would assign a placeholder variable 'q' to the value p/2. The parsed expression looks like q\\*3. To make a tree, it goes through these stages: 

(\\*, q, 3)

(\\*, (/, p, 2), 3)

(\\*, (/, (+, 1, x), 2), 3)

##### 2.2.3 Solving the Tree

Once a tree has been built from both sides, solving is relatively easy. The code takes the outermost node (operation) from the variable side and applies the opposite to the side without the variable. For example, if the full equation was 3x+1=7, the tree for the variable side would be (+, (\\*, 3, x), 1) and the tree for the non-variable side would simply be 7. The code would take the outermost node, '+', and apply the opposite of that operation to the other side. So the variable side tree would become (\\*, 3, x), or 3\\*x, and the non-variable side tree would become (-, 7, 1), or 7-1. The code changes the tree into a string each time, and solves for that string. Thereby, it realizes that 7-1 is 6, and changes the tree to be 6 instead of (-, 7, 1). It repeats the same process again, this time taking the multiplication from the variable side and applying division to the non-variable side. The variable tree changes from (\\*, 3, x) to x, and the non-variable tree changes from 6 to (/, 6, 3), which it then solves to be 2. Finally it sees that the variable is by itself on one side and comes to the conclusion that x=2. 

## Links

* Math Programming article: [K Nearest Neighbors and Handwritten Data Classification](https://jeremykun.com/2012/08/26/k-nearest-neighbors-and-handwritten-digit-classification/ "K Nearest Neighbors and Handwritten Data Classification")
* [MNIST Data Set](http://yann.lecun.com/exdb/mnist/ "MNIST Data Set")