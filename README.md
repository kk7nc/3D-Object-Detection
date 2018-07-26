# Weighted Unsupervised Learning for 3D Object Detection

Referenced paper : [Weighted Unsupervised Learning for 3D Object Detection](https://arxiv.org/pdf/1602.05920.pdf)


3D Object Detection:
=====================
This paper introduces a novel weighted unsupervised
learning for object detection using an RGB-D camera. This
technique is feasible for detecting the moving objects in the noisy
environments that are captured by an RGB-D camera. The main
contribution of this paper is a real-time algorithm for detecting
each object using weighted clustering as a separate cluster. In a
preprocessing step, the algorithm calculates the pose 3D position
X, Y, Z and RGB color of each data point and then it calculates
each data point’s normal vector using the point’s neighbor. After
preprocessing, our algorithm calculates k-weights for each data
point; each weight indicates membership. Resulting in clustered
objects of the scene.

![Object_Detection](http://kowsari.net/onewebstatic/Overview_Object.png)
Pipeline of 3D Object detection using RGB-D camera has two main parts: 1) Preprocessing including Mapping, Back-Projection,  Normal  Generating,  Background  removal  and  2)  Clustering  including  assigned  initial  weight,  distance  calculation,update weight and assign color, and finally visualization to illustrate the results.

Results:
=====================

![Object_Detection](http://kowsari.net/onewebstatic/OBJECT%20(1).jpg)
Kinect color frame (RGB) with resolution of 1920 X 1080; b) Kinect depth frame with resolution of 512 X 424; c) Proposed method object detection using k= 15 clusters, and after 15 iterations.


![Object_Detection](http://kowsari.net/onewebstatic/OBJECT%20(3).jpg)

a) Kinect color frame (RGB) with resolution of 1920 X 1080; b) Kinect depth frame with resolution of 512 X 424; c)  Proposed  method  object  detection  using  k=  7  clusters,  and  after  10  iterations.  Memory  consumption  is  320  MB  and  framerate is 8.1±0.2FPS.



![Object_Detection](http://kowsari.net/onewebstatic/OBJECT%20(2).jpg)
Results  of  segmenting  scene  objects  using  proposed  algorithm;a)  Segmentation  of  small  duck;b)  Segmentation  anddetection  of  piece  of  red  paper;c)  Object  detection  of  a  box;d)  Shows  handy  bag;e)  Segmentation  of  box,  the  border  of  thebox  has  lower  weight  and  it  will  be  completed  after  several  iteration;f)  Representation  of  moving  object,  segmentation  of  aperson;g) Segmentation of basketball.


Citations
---------

```

@article{Kowsari2016,
title = {Weighted Unsupervised Learning for 3D Object Detection},
journal = {International Journal of Advanced Computer Science and Applications}
doi = {10.14569/IJACSA.2016.070180},
url = {http://dx.doi.org/10.14569/IJACSA.2016.070180},
year = {2016},
publisher = {The Science and Information Organization},
volume = {7},
number = {1},
author = {Kamran Kowsari and Manal H. Alassaf},
}

```



