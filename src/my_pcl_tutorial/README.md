# Find stem of strawberry with pointcloud from SR300
## Pipeline
'''
1. Color filter with fixed RGB range
2. Statistical outlier removal
3. Find center
4. Min-cut segmentation with center from Step3.
5. Statistical outlier removal
6. PCA
7. Section plane 
8. Find stem
'''

TO DO: Compare the result after changing some parameters
