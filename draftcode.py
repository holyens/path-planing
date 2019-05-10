from collections import defaultdict
from heapq import *

def dijkstra2(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] #+ heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] !=0:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False




def dijkstra(edges, f, t):
    g = defaultdict(list)
    for l,r,c in edges:
        g[l].append((c,r))

    q, seen, mins = [(0,f,())], set(), {f: 0}
    while q:
        (cost,v1,path) = heappop(q)
        if v1 not in seen:
            seen.add(v1)
            path = (v1, path)
            if v1 == t: return (cost, path)

            for c, v2 in g.get(v1, ()):
                if v2 in seen: continue
                prev = mins.get(v2, None)
                next = cost + c
                if prev is None or next < prev:
                    mins[v2] = next
                    heappush(q, (next, v2, path))

    return float("inf")

if __name__ == "__main__":
    edges = [
        ("A", "B", 7),
        ("A", "D", 5),
        ("B", "C", 8),
        ("B", "D", 9),
        ("B", "E", 7),
        ("C", "E", 5),
        ("D", "E", 15),
        ("D", "F", 6),
        ("E", "F", 8),
        ("E", "G", 9),
        ("F", "G", 11)
    ]

    print("=== Dijkstra ===")
    print(edges)
    print("A -> E:")
    print(dijkstra(edges, "A", "E"))
    print("F -> G:")
    print(dijkstra(edges, "F", "G"))


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
# np.savetxt('results/a.csv',dst, fmt='%d',delimiter=',')

##writer = animation.writers['ffmpeg']
##writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)


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