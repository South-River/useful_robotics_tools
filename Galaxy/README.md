# Generating Large Convex Polytopes Directly on Point Clouds (2D)
This repository contains the implementation of 2D version of **Galaxy**, which generate point-free polyhedrons efficiently on point clouds.


_An example of output:_


![ptcl](./demo.png)

## Dependencies
1. Eigen
2. OpenCV

## Build
``` bash
mkdir build && cd build
cmake ..
make 
./demo
```
## Publication

If you use our implementation in your academic work, please cite the corresponding [paper](https://arxiv.org/pdf/2010.08744.pdf):  
    
    @article{zhong2020generating,
             title={Generating large convex polytopes directly on point clouds},
             author={Zhong, Xingguang and Wu, Yuwei and Wang, Dong and Wang, Qianhao and Xu, Chao and Gao, Fei},
             journal={arXiv preprint arXiv:2010.08744},
             year={2020}
    }
