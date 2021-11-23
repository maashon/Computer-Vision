# Stereo-reconstruction
# Parsa Pourzargham Neptun ID: LUM2CE

In this project, the 3d reconstruction of a scene is implemented using Naive Image patch approach, as well as the dynamic programming approach in the performance.xlsx file, the performance of the models over the dataset with different parameters is presented

The dataset used in the project is the middlebury dataset(2005-2006) from
https://vision.middlebury.edu/stereo/data/

According to the webpage, the focal length for the dataset is 3740 pixels and the baseline is 160 centimeters

the obtained disparity maps from the Naive approach, the DP algorithm and in built opencv SBM algorithm are
compared using SSD,MSQ and SSIM metrics.Also the runtime of all mentioned algorithms are stored in the excel file.

PSNR and SSIM metric calculation is done with a python script seperately.Code for SSIM and PSNR taken from :

SSIM

https://www.pyimagesearch.com/2017/06/19/image-difference-with-opencv-and-python/

PSNR

https://www.geeksforgeeks.org/python-peak-signal-to-noise-ratio-psnr/


screenshots of the 3d shapes recorded in Meshlab are available in data/Screenshots

Main image 

![view1](https://user-images.githubusercontent.com/72257286/142197666-8e938429-14f3-4abc-bb27-819bd87f7bfe.png)

Ground truth Disparity map

![disp1](https://user-images.githubusercontent.com/72257286/142197597-7a58dc91-6f15-44e4-87b6-5cc41c48bccc.png)


DP Disparity map

![output-dp-disparity](https://user-images.githubusercontent.com/72257286/142197692-9ff9785a-78bf-40ca-93e7-fb3a960b7bf5.png)


Naive approach Disparity map 

![output-naive-disparity](https://user-images.githubusercontent.com/72257286/142197734-35c0efdf-da19-4d3a-bfe4-6c69addfd971.png)

3D results 

![Aloe Ground truth](https://user-images.githubusercontent.com/72257286/142197987-d5a18cc9-cd39-4bb1-bd7f-bfee247ac78e.png)
![aloe dp 5000 ](https://user-images.githubusercontent.com/72257286/142197773-8a242579-f010-4572-ada5-09ff3923e4a8.png)
![aloe dp 80000 ](https://user-images.githubusercontent.com/72257286/142197783-b9eb9d91-73c5-476a-be4a-88f74431214a.png)


