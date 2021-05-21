import numpy as np
#构造有向图Graph
class Graph:
    def __init__(self,graph,labels):  #labels为标点名称
        self.Arcs=graph
        self.VertexNum=graph.shape[0]
        self.labels=labels
def Dijkstra(self,Vertex,EndNode):  #Vertex为源点，EndNode为终点
    Dist=[[] for i in range(self.VertexNum)] #存储源点到每一个终点的最短路径的长度
    Path=[[] for i in range(self.VertexNum)] #存储每一条最短路径中倒数第二个顶点的下标
    flag=[[] for i in range(self.VertexNum)] #记录每一个顶点是否求得最短路径
    index=0
    #初始化
    while index<self.VertexNum:
        Dist[index]=self.Arcs[Vertex][index]
        flag[index]=0
        if self.Arcs[Vertex][index]<float('inf'):  #正无穷
            Path[index]=Vertex
        else:
            Path[index]=-1 #表示从顶点Vertex到index无路径
        index+=1
    flag[Vertex]=1
    Path[Vertex]=0
    Dist[Vertex]=0
    index=1
    while index<self.VertexNum:
        MinDist=float('inf')
        j=0
        while j<self.VertexNum:
            if flag[j]==0 and Dist[j]<MinDist:
                tVertex=j  #tVertex为目前从V-S集合中找出的距离源点Vertex最断路径的顶点
                MinDist=Dist[j]
            j+=1
        flag[tVertex]=1
        EndVertex=0
        MinDist=float('inf') #表示无穷大，若两点间的距离小于MinDist说明两点间有路径
        #更新Dist列表，符合思想中第三条
        while EndVertex<self.VertexNum:
            if flag[EndVertex]==0:
                if self.Arcs[tVertex][EndVertex]<MinDist and Dist[
                    tVertex]+self.Arcs[tVertex][EndVertex]<Dist[EndVertex]:
                    Dist[EndVertex]=Dist[tVertex]+self.Arcs[tVertex][EndVertex]
                    Path[EndVertex]=tVertex
            EndVertex+=1
        index+=1
    vertex_endnode_path=[] #存储从源点到终点的最短路径
    return Dist[EndNode],start_end_Path(Path,Vertex,EndNode,vertex_endnode_path)
#根据本文上述定义的Path递归求路径
def start_end_Path(Path,start,endnode,path):
    if start==endnode:
        path.append(start)
    else:
        path.append(endnode)
        start_end_Path(Path,start,Path[endnode],path)
    return path
#ksp迭代算法
def ksp(path,Path):
    a = np.zeros((12))
    a[0] = 10000
    p=0
    s=[]
    PATH=[]
    short = 0
    #对一个路径依次进行偏离路径迭代
    for i in range(len(path)-1):
        store=Path
        m=Path[i]
        n=Path[i+1]
        #print(G.labels.index(Path[i]),G.labels.index(Path[i+1]))
        temp=graph[G.labels.index(Path[i])][G.labels.index(Path[i+1])]
        graph[G.labels.index(Path[i])][G.labels.index(Path[i+1])]=float('inf')  #路径设置为无穷
        dist, path = Dijkstra(G, G.labels.index(start), G.labels.index(endnode))
        # 将更新路径写入返回值中
        if dist >=1000:
            PATH = Path
            s = path
            short = dist
            break
        Path = []
        for j in range(len(path)):
            Path.append(G.labels[path[len(path) - 1 - j]])
        #print(Path)
        #print(dist)
        a[i+1]=dist
        #print(a)
        if a[i+1] <= a[p]:
            short = dist
            PATH = Path
            p = i + 1
            s = path
        graph[G.labels.index(m)][G.labels.index(n)]=temp
        Path=store
    #print(short)
    #print(PATH)
    if PATH==[]:
        PATH=Path
        s=path
        short=dist
    return PATH,short,s

if __name__=='__main__':
    #float('inf')表示无穷
    graph=np.array([[0,6,5,float('inf'),float('inf'),float('inf')],
                    [float('inf'),0,2,8,float('inf'),float('inf')],
                    [float('inf'),float('inf'),0,float('inf'),3,float('inf')],
                    [float('inf'),float('inf'),7,0,float('inf'),9],
                    [float('inf'),float('inf'),float('inf'),float('inf'),0,9],
                    [float('inf'),float('inf'),float('inf'),float('inf'),0]])
    G=Graph(graph,labels=['a','b','c','d','e','f'])
    '''
    start= input('请输入源点\n')
    endnode= input('请输入终点\n')
    dist,path=Dijkstra(G,G.labels.index(start),G.labels.index(endnode))
    Path=[]
    for i in range(len(path)):
        Path.append(G.labels[path[len(path)-1-i]])
    print('从顶点{}到顶点{}的最短路径为：\n{}\n最短路径长度为：{}'.format(start,endnode,Path,dist))
    '''

    start = input('请输入源点\n')
    endnode = input('请输入终点\n')
    dist, paths = Dijkstra(G, G.labels.index(start), G.labels.index(endnode))
    #得到最佳路径
    for j in range(1):
        Paths = []
        for i in range(len(paths)):
            Paths.append(G.labels[paths[len(paths) - 1 - i]])
        print("最佳路径",Paths)
        print("路径消耗",dist,"\n")
#设置多路径个数
k=10
Pathstorage=[]
cost=[]
#调用ksp算法，得到偏离路径
for i in range(k):
    if i == 0:
        Path0, dist0, s = ksp(path=paths, Path=Paths)
        cost.append(dist0)
        Pathstorage.append(Path0)
        if dist0 > 10000:
            Pathstorage.pop()
            cost.pop()
        # for j in range(len(Pathstorage)):
        #     if bool == 1:
        #         break
        #     if len(Pathstorage[j]) == len(Pathstorage[i]):
        #         for f in range(0, len(Pathstorage[j])):
        #             if Pathstorage[j][f] != Pathstorage[i][f]:
        #                 break
        #             else:
        #                if f==(len(Pathstorage[j])-1):
        #                     Pathstorage.pop()
        #                     bool = 1
        #                     break

    else:
        bool1=0
        Pathi=Path0
        Path0,dist0,s = ksp(path=s,Path=Pathi)
        Pathstorage.append(Path0)
        cost.append(dist0)
        if dist0>10000:
            Pathstorage.pop()
            cost.pop()
            bool1 = 1
        #将相同的的路径弹出
        #print(i,bool)
        for j in range(0,len(Pathstorage)-1):
            if bool1 == 1:
                bool1=0
                break
            len1=len(Pathstorage)-1
            #print(i)
            #print(Pathstorage)
            if len(Pathstorage[j]) == len(Pathstorage[len1]):
                for f in range(0, len(Pathstorage[j])):
                    #print(f)
                    if Pathstorage[j][f] != Pathstorage[len1][f]:
                        break
                    else:
                       if f==(len(Pathstorage[j])-1):
                            Pathstorage.pop()
                            cost.pop()
                            bool1 = 1
                            break
#把最佳路径存储进去
Pathstorage.append(Paths)
cost.append(dist)
print("偏离路径集合",Pathstorage)
print("路径消耗",cost)
#Pathstorage.pop()
#print(Pathstorage)