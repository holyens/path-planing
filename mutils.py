# Author: Shengen Wei
# Path-planning algorithms
# including: A*, BFS, Dijkstra
# 

import numpy
from matplotlib import pyplot as plt
from matplotlib import animation


def save_video(filename, roadmap, path, start, goal, show_ani=False,alg=''):
    plt.rcParams['animation.ffmpeg_path']='D:/DevSoft/ffmpeg-win64-static/bin/ffmpeg'
    ############## 可视化 ###############
    path_x = [t[1] for t in path]
    path_y = [t[0] for t in path]
    # 绘制路径并模拟
    fig = plt.figure()
    plt.imshow(255-roadmap,'gray'),plt.plot(path_x,path_y,'--r'),plt.title('%s: (%d,%d) to (%d,%d)'%(alg,start[0],start[1],goal[0],goal[1]))
    fig.set_size_inches(20, 2)
    line, = plt.plot(path_x[0],path_y[0], '.g',markersize=12) 
    def init():
        line.set_xdata(path_x[0])
        line.set_ydata(path_y[0])
        return line
    def animate(i):
        line.set_xdata(path_x[i*2])
        line.set_ydata(path_y[i*2])
        return line
    

    if show_ani:
        #plt.show()
        plt.savefig(filename)
        plt.close()
    else:
        # Set up formatting for the movie files
        ani = animation.FuncAnimation(fig=fig,
                                    func=animate,
                                    frames=int(len(path_x)/2),
                                    init_func=init,
                                    interval=10,
                                    blit=False)
        mywriter = animation.FFMpegWriter()
        ani.save(filename, writer=mywriter)

    #plt.close()

if __name__ == "__main__":


    pass