Introduction
=

1. pose.txt记录五张图像的相机位置（$ T_{wc} $），其形式是平移向量加旋转四元数：

```math
[x,y,z,q_{x},q_{y},q_{z}]
```
> $ q_{w} $为四元数的实部