# CLI for [Supervoxel for 3D point clouds](https://github.com/yblin/Supervoxel-for-3D-point-clouds)

## Build
```
mkdir build && cd build
cmake ..
make
```

## Methods

### [BPSS](https://github.com/yblin/Supervoxel-for-3D-point-clouds)
```bibtex
@article{lin.BetterBoundaryPreserved.2018,
	title = {Toward Better Boundary Preserved Supervoxel Segmentation for {{3D}} Point Clouds},
	author = {Lin, Yangbin and Wang, Cheng and Zhai, Dawei and Li, Wei and Li, Jonathan},
	year = {2018},
	month = sep,
	journal = {ISPRS Journal of Photogrammetry and Remote Sensing},
	series = {{{ISPRS Journal}} of {{Photogrammetry}} and {{Remote Sensing Theme Issue}} ``{{Point Cloud Processing}}''},
	volume = {143},
	pages = {39--47},
	issn = {0924-2716},
	doi = {10.1016/j.isprsjprs.2018.05.004}
}

```
### [VCCS](https://github.com/yblin/Supervoxel-for-3D-point-clouds/blob/master/vccs_supervoxel.h)
```bibtex
@inproceedings{papon.VoxelCloudConnectivity.2013,
	title = {Voxel {{Cloud Connectivity Segmentation}} - {{Supervoxels}} for {{Point Clouds}}},
	booktitle = {2013 {{IEEE Conference}} on {{Computer Vision}} and {{Pattern Recognition}}},
	author = {Papon, Jeremie and Abramov, Alexey and Schoeler, Markus and W{\"o}rg{\"o}tter, Florentin},
	year = {2013},
	month = jun,
	pages = {2027--2034},
	issn = {1063-6919},
	doi = {10.1109/CVPR.2013.264}
}
```
### [KNN variant of VCCS](https://github.com/yblin/Supervoxel-for-3D-point-clouds/blob/master/vccs_knn_supervoxel.h)

## Usage
```bash
mkdir build
cd build
cmake ..
make
```

```bash
./supervoxel_segmentation \
	-i="../test_data/test.xyz" \		# input file
	-o="../test_data/segmented.xyz" \	# output file
	-m="bpss" \				# method "bpss", "vccs", or "vccs_knn"
	-n \ 					# (re-)estimate normals
	-k=15 \ 				# neighbors in knn search
	-s=10000 \				# number of supervoxels to estimate
	-r=1.0 \				# resolution (if given, -s is omitted)
	-vccs_vr=0.3 \				# seed_resolutin in VCCS
	-t=32					# number of threads
```
- Normal estimation depends on `-k`
- BPSS depends on `-k`, `-r`/`-s`
- VCCS depends on `-r`, `-vccs_vr`
- VCCS KNN depends on `-k`, `-r`
