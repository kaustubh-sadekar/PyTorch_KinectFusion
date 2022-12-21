---
layout: default
---

# PyTorch Implementation of Kinect Fusion Algorithm For 3D Reconstruction

## Project in brief
---

Kinect Fusion is a real-time 3D reconstruction algorithm that generates a 3D model of a given scene using a sequence of depth images. There are some standard opensource implementations for the Kinect Fusion algorithm available online. However, most of them are written in C++. This project aims to create a complete python implementation of the Kinect Fusion algorithm and use **vectorization with PyTorch** to enable **GPU acceleration** capabilities.

<p align='center'>
  <img src='/images/KinFu_LivingRoom.gif' width=800>
</p>
<p align='center'>
    3D Reconstruction results on the <a href="http://redwood-data.org/indoor/dataset.html" target="_blank">livingroom sequence of ICL-NUIM dataset</a> using the implementation code. 
</p>




## Key Steps Of The Kinect Fusion Algorithm
---

1. Capture **depth map** of the scene.
2. Apply **bilateral filtering** to the depth map.
3. Obtain 2.5D representation (vertex map/ pointcloud) by **back-projecting depth map** to 3D coordinate space.
4. Obtain **normal vector** for each projected point p by computing cross product (p1 - p)x(p2 - p). Where p1 and p2 are the neighboring vertices.
5. Store the depth map information by reconstructing the surface using the **Truncated Signed Distance Function (TSDF)** for MxMxM voxel grid representing a KxKxK cubic meter space. (Store the TSDF values for each voxel in the voxel grid)
6. Capture the next depth map and apply steps 2,3 and 4 to it. Let the 2.5D pointcloud obtained be pcl_i and the normal map obtained be nmap_i.
7. Using **ray casting**, obtain global pointcloud pcl_global and global normal map nmap_global. 
8. Apply **(Iterative closest Point) ICP algorithm** to find the camera's pose (rotation and translation) when it captured the depth map in step 5.
9. Use the camera pose information obtained in the above step to **fuse the TSDF information** obtained in step 5  with the global TSDF.
10. Repeat step 6 to step 9 until all depth maps are fused.


### Vectorized implementation to extract 2.5D ordered pointcloud from an RGB-D Image
---

In the case of ordered pointcloud, the memory layout of the points is closely related to the spatial layout. This property can be used to speed up the process of calculating normals.


The vectorized implementaiton computes 3D points for each pixel **u** = (x, y) in the incoming depth map **D**<sub>i</sub>(**u**) parallely.
Here K is the 3x3 intrinsic camera matrix. Corresponding 3D points in the camera’s coordinate space are calculated as follows: **vi**(**u**) = **D**<sub>i</sub>(u) **K**−1[u, 1]. This results in a single vertex map (2.5 D pointcloud) **V**<sub>i</sub> computed in parallel. 

The <a href="https://pytorch.org/docs/stable/generated/torch.meshgrid.html" target="_blank">meshgrid method of PyTorch</a> was used for the GPU accelerated vectorized implementation.



### PyTorch Implementation of Iterative Closest Point (ICP) Algorithm

ICP is commonly used for the registration of 3D pointclouds. ICP calculates the rigid transformation that best aligns a source pointcloud to a target pointcloud. In Kinect fusion the source pointcloud is the current 2.5D pointcloud obtained using back-projection, and the target pointcloud is obtained from the reconstructed global 3D model. Hence in this case, the transformation returned by ICP provides the instantaneous global pose of the camera. This is called **Frame-to-model tracking**. 

Kinect Fusion uses **Fast ICP** which combines **projective data-association** and **point-plane ICP** algorithm. The objective function for Fast ICP is as follows:

<p align='center'>
  <img src='/images/math/FastICP_objfun.png' width=450>
</p>

The algorithm for Fast ICP is as follows:

<p align='left'>
  <img src='/images/math/FastICP.png' width=450>
</p>

I used PyTorch for vectorized implementation and to enable GPU acceleration capabilities for Fast ICP. The following result illustrates the process of aligning two pointclouds using the vectorized implementation. The pointclouds for the demo were generated using a car model from <a href="https://shapenet.org/" target="_blank">ShapeNet dataset</a>  and the RGB-D images were generated using the renderer provided by <a href="https://pytorch3d.org/docs/renderer" target="_blank">PyTorch3D</a>.

{% include feature_row %}


### PyTorch Implementation of TSDF Fusion Algorithm

Fusing TSDF information is as simple as averaging or weighted sum. It is also easy to optimize. These qualities make TSDF a good choice of representation for the reconstructed 3D model.

<p align='center'>
  <img src='/images/depth2tsdf.png' width=450>
</p>
<p align='center'>
  Figure showing how voxels vox1 and vox2 can be projected to the image to obtain corresponding depth values. TSDF(vox1) = d0 - d1 and TSDF(vox2) = d0 - d2.
</p>

The TSDF values are updated using weighted averaging only for the voxels that are captured by the camera. A vectorized implementation for this task was written, which provided a mask for all the valid voxels. This, in a way, represents the ray casting step of Kinect fusion. The mask obtained in this stage is used to find the target pointcloud for the fast ICP step. 

The PyTorch implementation is inspired by <a href="https://github.com/andyzeng/tsdf-fusion-python" target="_blank">this project</a>, but instead of using marching cubes, I have written a vectorized thresholding based method which further reduces the computation time.

It is important to note that TSDF fusion gives promising results, and most of its components can leverage GPU acceleration capabilities, but at the same time, it is memory expensive. Storing even a 512x512x512 voxel grid representing the 3D volume can require 6GB of GPU RAM. Hence the output depends on the available hardware. Presently I have only implemented TSDF fusion for a fixed size of the voxel grid. 

To improve the reconstruction quality, we can divide the entire 3D space into larger sections and represent each section with an NxNxN voxel grid, and only the section visible to the camera can be transferred to GPU. This is similar to voxel hashing. Open3D provides <a href="http://www.open3d.org/docs/release/python_api/open3d.pipelines.integration.ScalableTSDFVolume.html#open3d.pipelines.integration.ScalableTSDFVolume" target="_blank">ScalableTSDFVolume</a> class to handle this.



<table class="tg">
<thead>
  <tr>
    <th class="tg-7btt" rowspan="2">Voxel Resolution</th>
    <th class="tg-7btt" rowspan="2">Memory Required</th>
    <th class="tg-7btt" colspan="2">Avf. Time To Fuse 200 Frames</th>
  </tr>
  <tr>
    <td class="tg-c3ow">(CPU)</td>
    <td class="tg-c3ow">(GPU)</td>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-c3ow">447 x 398 x 444</td>
    <td class="tg-c3ow">5.24 GB</td>
    <td class="tg-c3ow">140.81 (s)</td>
    <td class="tg-c3ow">7.32 (s)</td>
  </tr>
  <tr>
    <td class="tg-baqh">224 x 199 x 222</td>
    <td class="tg-baqh">2.28 GB</td>
    <td class="tg-baqh">18.26 (s)</td>
    <td class="tg-baqh">2.36 (s)</td>
  </tr>
  <tr>
    <td class="tg-baqh">128 x 114 x 127</td>
    <td class="tg-baqh">1.5 GB</td>
    <td class="tg-baqh">6.31 (s)</td>
    <td class="tg-baqh">0.26 (s)</td>
  </tr>
</tbody>
</table>



<p align='center'>
  <img src='/images/Torch_TSDF_Fusion.gif' width=800>
</p>

<p align='center'>
  GIF showing the global map being reconstructed using the GPU accelerated vectorized implementation of TSDF Fusion using PyTorch. The reconstruction took 10.8 seconds - fusing 1000 RGB-D frames. 
</p>


## Future Work
---

Currently, the implementation is restricted to a fixed 3D volume. For high-quality, large-scale reconstruction, I am currently using the ScalableTSDFVolume method of Open3D. I plan to add a similar facility to the vectorized implementation.

## References
---

1. Izadi, Shahram & Kim, David & Hilliges, Otmar & Molyneaux, David & Newcombe, Richard & Kohli, Pushmeet & Shotton, Jamie & Hodges, Steve & Freeman, Dustin & Davison, Andrew & Fitzgibbon, Andrew. (2011). KinectFusion: Real-time 3D reconstruction and interaction using a moving depth camera. UIST'11 - Proceedings of the 24th Annual ACM Symposium on User Interface Software and Technology. 
2. Li, Fei & Du, Yunfan & Liu, Rujie. (2016). Truncated Signed Distance Function Volume Integration Based on Voxel-Level Optimization for 3D Reconstruction. Electronic Imaging. 2016. 1-6. 
3. Original TSDF Fusion library : <a href="https://github.com/andyzeng/tsdf-fusion-python" target="_blank">https://github.com/andyzeng/tsdf-fusion-python</a>.


*Project page template inspired from [GradSLAM](https://gradslam.github.io/).*
