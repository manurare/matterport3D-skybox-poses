# Depth for skybox images
This repo allows to obtain GT depth for the skybox images (in equirectangular projection) from the [matterport3D dataset](https://niessner.github.io/Matterport/) in any resolution (2048x1024 by default). It performs Structure from Motion (SfM) in each room for all buildings to obtain skybox camera poses. Then, using the provided meshes and the poses we can render the GT depth.

__Notice: by images/views/renderings we are referring always to 360 equirectangular images__

### Required dependencies

1. [openMVG](https://github.com/openMVG/openMVG) (v1.6)
2. [OpenCV 4.2](https://opencv.org/)
3. [openEXR](https://github.com/AcademySoftwareFoundation/openexr) (v3.1.0)

### Instructions
Matterport3D contains a total of 90 sequences (buildings) each with N rooms inside the range [8, 349].
#### Data generation
In order to run SfM per room, we need images of it. We can render S (S=18 by default) synthetics views to find keypoints and matches wrt to the high resolution skybox for that room. This is done with Blender and a good GPU is recommended. We need to render a total of 10912\*18=196416 images! 

1. For the renderings:

	```./blender --background --python blender_scripts/shadeless_textures.py```

	*change ```base_path``` in ```blender_scripts/shadeless_textures.py``` to your own matterport3D directory*.


These are the images which are going to be fed to the SfM pipeline with the addition of the skybox images. These last ones need to be generated using [PanoBasic](https://github.com/yindaz/PanoBasic) Matlab toolbox.


2. Add the skybox images to the ```rendered_textures``` folder generated in 1. Rename each of them to be the highest in the set of images of a room ```<uuid>_S_rgb.png```. By default S=18 then ```<uuid>_18_rgb.png```

#### SfM Pipeline
OpenMVG is run to perform *mini* SfMs: one per room. For each building, we will run N times a SfM experiment with T images (S + 1) in our case (18 + 1): 18 synthetic renderings and 1 HR skyboxi. Step 1 returns as well GT poses for the rendered views therefore in each *mini* experiment thus, we only have one unknown: the skybox image pose.

3. ```python SfM_SequentialPipelinePerRoom.py```
	
	*change paths in ```SfM_SequentialPipelinePerRoom.py``` to your own*


4. This will give us the skybox poses but in a different scale from the GT poses generated in 1 since Bundle Adjustment can *move the world* arbitrarily to reach a solution to the minimization problem. For adjusting scale of generated poses with GT poses:

	```python procrustes_openMVG_matterport.py```

Note that as SfM is not perfect we can not obtain poses for every skybox image due to lack of keypoints, lack of matches etc. Also, the poses can potentially be wrongly estimated but it is possible to remove outliers. To remove outliers:

5. ```python skybox_tripod_relation.py```


Finally, we will obtain the final poses for the skybox images ```skybox_camera_parameters.txt```

#### Render skybox depth
Once skybox camera parameters are known. We can render depth from the mesh using blender. 

6. ```./blender --background erp_cube.blend --python blender_scripts/skybox_depth_SfM.py``` 
