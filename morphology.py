#19，形态学处理 
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import algorithms as alg
##膨胀
def image_Dilate(image):
    print(image.shape)
    #将图像转化为灰度图像
    gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    #对灰度图像进行二值化处理
    ret,binary=cv.threshold(gray,0,255,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
    #将二值化图像显示
    # cv.imshow("binary",binary)
    #设置形态学结构处理的核 矩形：MORPH_RECT; 交叉形：MORPH_CORSS; 椭圆形： MORPH_ELLIPSE;
    kernel=cv.getStructuringElement(cv.MORPH_ELLIPSE,(10,10))
    #对二值图像进行膨胀操作
    dst=cv.dilate(binary,kernel)
    # cv.imshow("dilate_demo",dst)
    return dst

img = cv.imread('res/roadmap.png')


gray=cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#对灰度图像进行二值化处理
ret,org=cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
#设置形态学结构处理的核 矩形：MORPH_RECT; 交叉形：MORPH_CORSS; 椭圆形： MORPH_ELLIPSE;
kernel=cv.getStructuringElement(cv.MORPH_ELLIPSE, (10,10))
#对二值图像进行膨胀操作
dst=cv.dilate(org, kernel)
# np.savetxt('results/a.csv',dst, fmt='%d',delimiter=',')
start = (50,0)
goal = (50,999)
path = alg.astar(dst, start, goal)
path_x = [t[0] for t in path]
path_y = [t[1] for t in path]
plt.subplot(311), plt.imshow(org,'gray'),plt.title('ORIGIN')
plt.subplot(312), plt.imshow(dst,'gray'),plt.plot(path_y,path_x,'--r'),plt.title('RESULT (%d,%d)->(%d,%d)'%(start[0],start[1],goal[0],goal[1]))
plt.subplot(313), plt.imshow(org,'gray'),plt.plot(path_y,path_x,'--r'),plt.title('RESULT (%d,%d)->(%d,%d)'%(start[0],start[1],goal[0],goal[1]))
plt.show()



'''
# print(type(img))
# cv.namedWindow("src",0)
# cv.imshow("src",img)
# print(path)
# cv.waitKey(0)
# cv.destroyAllWindows() #使用Window name销毁指定窗口
# cv2.imwrite('messigray.png',img)
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