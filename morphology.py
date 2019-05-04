#19，形态学处理 
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import astar
##膨胀
def image_Dilate(image):
    print(image.shape)
    #将图像转化为灰度图像
    gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    #对灰度图像进行二值化处理
    ret,binary=cv.threshold(gray,0,255,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
    #将二值化图像显示
    cv.imshow("binary",binary)
    #设置形态学结构处理的核
    kernel=cv.getStructuringElement(cv.MORPH_RECT,(5,5))
    #对二值图像进行膨胀操作
    dst=cv.dilate(binary,kernel)
    cv.imshow("dilate_demo",dst)
    return dst

img = cv.imread(r'E:\workspace\path-planing\res\roadmap.png')
print(type(img))
# cv.namedWindow("src",0)
cv.imshow("src",img)
dst = image_Dilate(img)
np.savetxt('results/a.csv',dst, fmt='%d',delimiter=',')
print(dst.shape)
print(dst[60:63,60:63])
path = astar.astar(dst, (50,0), (50,50))
print(path)
cv.waitKey(0)
cv.destroyAllWindows() #使用Window name销毁指定窗口
# cv2.imwrite('messigray.png',img)
'''
plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
plt.show()
'''
##腐蚀
'''
def image_Erode(image):
    print(image.shape)
    #将图像转化为灰度图像
    gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    #对灰度图像进行二值化处理
    ret,binary=cv.threshold(gray,0,255,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
    #将二值化图像显示
    cv.imshow("binary",binary)
    #设置形态学结构处理的核
    kernel=cv.getStructuringElement(cv.MORPH_RECT,(3,3))
    #对二值图像进行腐蚀操作
    dst=cv.erode(binary,kernel)
    cv.imshow("erode_demo",dst)
'''