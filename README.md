# KinectV2-Kalman
Implementation of Linear Kalman Filter on Kinect V2 skeleton tracking data using MATLAB.

## Description
Smooth out the noise in Kinect's skeleton tracking algorithm using simple linear Kalman Filtering.
Tested on MATLAB R2018a and Microsoft Kinect V2 for Windows.

## Instructions

Simply run the color_kalman_kinectv2.m or the depth_kalman_kinectv2.m file to for the kalman filtering of Kinect V2.
The pointcloud.m is simply a way to compute a pointcloud in MATLAB using the Kinect V2 Sensor. Keep in mind
that the Kinect V2 is pretty slow in MATLAB when the skeleton tracking is enabled.

## Requirements

Full list of **all** requirements

* [MATLAB R2018a](https://www.mathworks.com/products/matlab.html)
* [Image Acquisition Toolbox Support Package for Kinect For Windows Sensor](https://www.mathworks.com/help/supportpkg/kinectforwindowsruntime/index.html?s_tid=CRUX_lftnav)

## Examples of Code
***

# Color

Example of Kalman and Kinect Joint Tracking, where the red circle represents 
the kalman Joint Tracking and the green circle the kinect's joint value:

(Color Tracking)[img/color.png] 

# Depth

Same as the Color Example

(Depth Tracking)[img/depth.png]

# Plots

Plots comparing the kinect's values to the kalman estimation.

(Kalman-Kinect Comparison)[img/plot.png]
