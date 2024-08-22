<div align="center">
  <h1>SOLiD-PyICP-SLAM</h1>
  <a href="https://github.com/sparolab/solid/tree/master"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="https://ieeexplore.ieee.org/abstract/document/10629042"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://www.alphaxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/alphaXiv-2408.07330-darkred" alt="alphaXiv" /></a>
  <a href="https://www.youtube.com/watch?v=4sAWWfZTwLs"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />
</div>

* SOLiD (Spatially Organized and Lightweight global Descriptor for LiDAR Place Recognition) based Full Python based SLAM
* We estimate odometry using Point2Plane ICP in Open3D and optimize the pose graph using GTSAM.
* Prerequisite
  * [SOLiD](https://github.com/sparolab/solid) (Spatially Organized and Lightweight global Descriptor for LiDAR Place Recognition)
  * GTSAM
    <pre>
    <code>
    $ pip3 install gtsam
    </code>
    </pre>  

  * OPEN3D
    <pre>
    <code>
    $ pip3 install open3d
    </code>
    </pre>  

  * FFMPEG
    <pre>
    <code>
    $ sudo apt install ffmpeg
    </code>
    </pre>  

## Run
0. Download SOLiD-PyICP-SLAM.
<pre>
<code>
$ git clone https://github.com/sparolab/SOLiD-PyICP-SLAM.git
</code>
</pre>  

1. Download KITTI in Datasets Folder.
<pre>
<code>
$ mkdir Datasets
</code>
</pre>  

2. Just RUN!!
<pre>
<code>
$ python3 main.py
</code>
</pre>  

## Results
In sequence 00, 02, 05 and 08.  
<img src="result/00.gif" width="200"/> <img src="result/02.gif" width="200"/> <img src="result/05.gif" width="200"/> <img src="result/08.gif" width="200"/>

## Special Thanks
Thank you ![Giseop Kim](https://github.com/gisbi-kim/PyICP-SLAM) for providing base code.
