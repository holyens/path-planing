import time
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import algorithms as alg
plt.rcParams['animation.ffmpeg_path']='D:/DevSoft/ffmpeg-win64-static/bin/ffmpeg'

i=2
img = cv.imread('res/roadmap-%d.jpg'%i)
# 灰度化
gray=cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# 灰度图像二值化
ret,org=cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
org1 = org
org1[0,:]=255;org1[-1,:]=255 #添加边界
# 形态学膨胀
kernel=cv.getStructuringElement(cv.MORPH_ELLIPSE, (10,10))  #设置形态学结构处理的核 矩形：MORPH_RECT; 交叉形：MORPH_CORSS; 椭圆形： MORPH_ELLIPSE
dst=cv.dilate(org1, kernel)
# 原始道路地图 & 形态学膨胀后的道路地图
fig1 = plt.figure()
plt.subplot(311),plt.imshow(255-org,'gray'),plt.title('ORIGIN')
plt.subplot(312),plt.imshow(255-dst,'gray') #,plt.plot(path_x,path_y,'--r'),plt.title('RESULT (%d,%d)->(%d,%d)'%(start[0],start[1],goal[0],goal[1]))
# 设置起点和目标点
(height,width)=(dst.shape[0],dst.shape[1])
start = (height//2, 0)
goal = (height//2, width-1)
# 使用路径规划算法寻找路径
time_0 = time.perf_counter()
(path1,mask1) = alg.bfs(dst, start, goal)
elapsed = (time.perf_counter() - time_0)
print("BFS Time used:",elapsed)
print('path length: %d'%(len(path1)-1))
print('orgi_',np.sum(org==0))
print('mask_',np.sum(mask1!=0))

time_0 = time.perf_counter()
(path2,mask2) = alg.astar(dst, start, goal)
elapsed = (time.perf_counter() - time_0)
print("A*  Time used:",elapsed)
print('path length: %d'%(len(path2)-1))
print('orgi_',np.sum(org==0))
print('mask_',np.sum(mask2!=0))



plt.subplot(313),plt.imshow(mask2,'gray')
# 计算并打印路径长度
############## 可视化 ###############
path_x = [t[1] for t in path2]
path_y = [t[0] for t in path2]
# 绘制路径并模拟
fig3 = plt.figure()
plt.imshow(255-org,'gray'),plt.plot(path_x,path_y,'--r'),plt.title('(%d,%d)->(%d,%d)'%(start[0],start[1],goal[0],goal[1]))
fig3.set_size_inches(20, 2)
line, = plt.plot(path_x[0],path_y[0], '.g',markersize=12) 
def init():
    line.set_xdata(path_x[0])
    line.set_ydata(path_y[0])
    return line
def animate(i):
    line.set_xdata(path_x[i*2])
    line.set_ydata(path_y[i*2])
    return line
# Set up formatting for the movie files
ani = animation.FuncAnimation(fig=fig3,
                            func=animate,
                            frames=int(len(path_x)/2),
                            init_func=init,
                            interval=10,
                            blit=False)

mywriter = animation.FFMpegWriter()
ani.save('res/MovWave2.mp4', writer=mywriter)
plt.show()
