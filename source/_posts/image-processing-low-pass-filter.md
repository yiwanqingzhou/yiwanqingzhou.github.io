---
title: OpenCV图像处理2-使用低通滤波进行去噪
date: 2023-08-29 18:06:54
tags: 
- 图像处理
- cv
- opencv
- python
categories:
- cv
keywords:
- 图像处理
- cv
- opencv
- python
- 卷积
- 滤波
- 边缘检测
description:

---



## 低通滤波: 通常使用来去噪

### 常见滤波类型

##### 均值滤波

![math-4](image-processing-low-pass-filter/math-4.png)

##### 高斯滤波 (权重与距离相关)

![math-5](image-processing-low-pass-filter/math-5.png)

##### 中值滤波

顾名思义取中值






### 具体代码例子

##### 高斯的模糊/去噪效果
```python
# Read in the image
image = cv2.imread('images/brain_MR.jpg')

# Convert to grayscale for filtering
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Create a Gaussian blurred image
gray_blur = cv2.GaussianBlur(gray, (9, 9), 0)

# use custom kernal
# gaussian = (1/16)*np.array([[1, 2, 1],
#                            [2, 4, 2],
#                            [1, 2, 1]])
# 
# gray_blur = cv2.filter2D(gray, -1, gaussian)

f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
ax1.set_title('original gray'),ax1.imshow(gray, cmap='gray')
ax2.set_title('blurred image'),ax2.imshow(gray_blur, cmap='gray')
```

![3](image-processing-low-pass-filter/3.png)


##### 与高通滤波配合
```python
# Filter the orginal and blurred grayscale images using filter2D
filtered = cv2.filter2D(gray, -1, sobel_y)
filtered_blurred = cv2.filter2D(gray_blur, -1, sobel_y)

f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
ax1.set_title('original & filtered imnage'),ax1.imshow(filtered, cmap='gray')
ax2.set_title('blurred & filtered image'),ax2.imshow(filtered_blurred, cmap='gray')
```

![4](image-processing-low-pass-filter/4.png)

```python
retval, binary_image = cv2.threshold(filtered_blurred, 50, 255, cv2.THRESH_BINARY)
plt.imshow(binary_image, cmap='gray')
```

![5](image-processing-low-pass-filter/5.png)

```python
filtered_blurred = cv2.filter2D(gray_blur, -1, laplacian)
retval, binary_image = cv2.threshold(filtered_blurred, 5, 255, cv2.THRESH_BINARY)
plt.imshow(binary_image, cmap='gray')
```

![6](image-processing-low-pass-filter/6.png)

