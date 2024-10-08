<div align="center">
  <h1>SOLiD-PyICP-SLAM</h1>
  <a href="https://github.com/sparolab/solid/tree/master"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="https://ieeexplore.ieee.org/abstract/document/10629042"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://www.alphaxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/alphaXiv-2408.07330-darkred" alt="alphaXiv" /></a>
  <a href="https://www.youtube.com/watch?v=4sAWWfZTwLs"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />

This repository is the [SOLiD](https://github.com/sparolab/solid)-based Full Python SLAM for Narrowing your FOV with **SOLiD**: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition. The results below are in order of KITTI 00, 02, 05, and 08 sequences.

  <a href="https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko" target="_blank">Hogyun Kim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=wL8VdUMAAAAJ&hl=ko" target="_blank">Jiwon Choi</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=UPg-JuQAAAAJ&hl=ko" target="_blank">Taehu Sim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=9mKOLX8AAAAJ&hl=ko" target="_blank">Giseop Kim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko" target="_blank">Younggun Cho</a><sup>†</sup>

**[Spatial AI and Robotics Lab (SPARO)](https://sites.google.com/view/sparo/%ED%99%88?authuser=0&pli=1)**


  <p align="center">
    <img src="result/00.gif" alt="animated" width="24%" />
    <img src="result/02.gif" alt="animated" width="24%" />
    <img src="result/05.gif" alt="animated" width="24%" />
    <img src="result/08.gif" alt="animated" width="24%" />
  </p>

</div>

## SOLiD based Full Python based SLAM
* **SOLiD (Spatially Organized and Lightweight global Descriptor for LiDAR Place Recognition)** is a lightweight and fast LiDAR global descriptor for FOV constraints situations that are limited through fusion with other sensors or blocked by robot/sensor operators including mechanical components or solid-state LiDAR (e.g. Livox).
* We estimate odometry using Point2Plane ICP in Open3D and optimize the pose graph using GTSAM.
* Purpose
  * This implementation is fully Python-based so slow and underperforming, but for educational purposes.
* Cpp version will be revealed soon.
	* (TBD) Integrated with A-LOAM: [SOLiD-A-LOAM](https://github.com/sparolab/SOLiD-A-LOAM.git)
	* (TBD) Integrated with LOAM-LIVOX: [SOLiD-LOAM-LIVOX](https://github.com/sparolab/SOLiD-A-LOAM.git)
* Scan Context fails in KITTI 08, which exists a lane-level reverse loop, but SOLiD detects this loop.

## Prerequisite
  * [SOLiD](https://github.com/sparolab/solid)
  * [GTSAM](https://pypi.org/project/gtsam/)
  * [OPEN3D](https://pypi.org/project/open3d/)
  * [FFMPEG](https://phoenixnap.com/kb/install-ffmpeg-ubuntu)

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

## Supplementary
* [Arxiv](https://arxiv.org/abs/2408.07330#)
* [Paper](https://ieeexplore.ieee.org/abstract/document/10629042)
* [Video](https://www.youtube.com/watch?v=4sAWWfZTwLs)
* [Project page](https://sites.google.com/view/lidar-solid)

## Main Contribution
* [Hogyun Kim](https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko)
* [Jiwon Choi](https://scholar.google.com/citations?user=wL8VdUMAAAAJ&hl=ko)
* [Taehu Sim](https://scholar.google.com/citations?user=UPg-JuQAAAAJ&hl=ko)
* [Giseop Kim](https://scholar.google.com/citations?user=9mKOLX8AAAAJ&hl=ko)
* [Younggun Cho](https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko)

## QnA
* If you have a question, you utilize a [alphaXiv](https://www.alphaxiv.org/abs/2408.07330) and comment here.

## Special Thanks
Thank you [Giseop Kim](https://github.com/gisbi-kim/PyICP-SLAM) and [MyeongHwan Jeon](https://github.com/MyungHwanJeon/PyICP-SLAM) for providing the base code.

## Citation
<pre>
<code>
@article{kim2024narrowing,
  title={Narrowing your FOV with SOLiD: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition},
  author={Kim, Hogyun and Choi, Jiwon and Sim, Taehu and Kim, Giseop and Cho, Younggun},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}</code>
</pre>  

