#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <queue> 

using namespace std;

class CNode
{
public:

    CNode() : xPos(0), yPos(0), travelCost(0) {}
    CNode(int x, int y) : xPos(x), yPos(y), travelCost(0) {}
    CNode(int x, int y, int cost) : xPos(x), yPos(y), travelCost(cost) {}

    inline CNode& operator=(const CNode& target)
    {
        if (*this != target)
        {
            xPos = target.xPos;
            yPos = target.yPos;
            travelCost = target.travelCost;
        }

        return *this;
    }

    inline bool operator==(const CNode& target) const
    {
        return xPos == target.xPos && yPos == target.yPos;
    }

    inline bool operator!=(const CNode& target) const
    {
        return !(*this == target);
    }

    inline bool operator<(const CNode& target) const
    {
        return target.travelCost < travelCost;
    }

    int xPos, yPos, travelCost;
};

class CPath
{
public:

    typedef vector<CNode> nodeList;

    nodeList Find(const CNode& startNode, const CNode& endNode, int mapArray[][20])
    {
        nodeList finalPath, openList, closedList;

        finalPath.push_back(startNode);
        openList.push_back(startNode);
        closedList.push_back(startNode);

        while (!openList.empty())
        {
            // Check each node in the open list
            for (size_t i = 0; i < openList.size(); ++i)
            {
                if (openList[i].xPos == endNode.xPos && openList[i].yPos == endNode.yPos)
                    return finalPath;

                priority_queue<CNode> nodeQueue;

                // Get surrounding nodes
                for (int x = -1; x <= 1; ++x)
                {
                    for (int y = -1; y <= 1; ++y)
                    {
                        const int current_x = openList[i].xPos + x;
                        const int current_y = openList[i].yPos + y;

                        bool alreadyCheckedNode = false;
                        for (size_t i = 0; i < closedList.size(); ++i)
                        {
                            if (current_x == closedList[i].xPos && current_y == closedList[i].yPos)
                            {
                                alreadyCheckedNode = true;
                                break;
                            }
                        }

                        if (alreadyCheckedNode)
                            continue;

                        // Ignore current coordinate and don't go out of array scope
                        if (current_x < 0 || current_x > 20 || current_y < 0 ||current_y > 20 || (openList[i].xPos == current_x && openList[i].yPos == current_y))
                            continue;

                        // Ignore walls
                        if (mapArray[current_x][current_y] == '#')
                            continue;

                        const int xNodeDifference = abs(current_x - (openList[i].xPos));
                        const int yNodeDifference = abs(current_y - (openList[i].yPos));            

                        // Diagonal?
                        const int direction = xNodeDifference == 1 && yNodeDifference == 1 ? 14 : 10;

                        const int xDistance = abs(current_x - endNode.xPos);
                        const int yDistance = abs(current_y - endNode.yPos);
                        int heuristic = 10 * (xDistance + yDistance);

                        nodeQueue.push(CNode(current_x, current_y, heuristic));
                    }
                }

                if (!nodeQueue.empty())
                {
                    // Add the nearest node
                    openList.push_back(nodeQueue.top());
                    finalPath.push_back(nodeQueue.top());

                    // Put into closed list
                    while (!nodeQueue.empty())
                    {
                        closedList.push_back(nodeQueue.top());
                        nodeQueue.pop();
                    }
                }
            }
        }

        return finalPath;
    }
};

int mapArray[20][20] =
{
    { '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' },
    { '#', 'A', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
    { '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'B', '#' },
    { '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' },

};

/*int main()
{
    srand(time(NULL));

    // create empty map
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++) map[x][y]=0;
    }

    // fillout the map matrix with a '+' pattern
    for(int x=n/8;x<n*7/8;x++)
    {
        map[x][m/2]=1;
    }
    for(int y=m/8;y<m*7/8;y++)
    {
        map[n/2][y]=1;
    }
    
    // randomly select start and finish locations
    int xA, yA, xB, yB;
    switch(rand()%8)
    {
        case 0: xA=0;yA=0;xB=n-1;yB=m-1; break;
        case 1: xA=0;yA=m-1;xB=n-1;yB=0; break;
        case 2: xA=n/2-1;yA=m/2-1;xB=n/2+1;yB=m/2+1; break;
        case 3: xA=n/2-1;yA=m/2+1;xB=n/2+1;yB=m/2-1; break;
        case 4: xA=n/2-1;yA=0;xB=n/2+1;yB=m-1; break;
        case 5: xA=n/2+1;yA=m-1;xB=n/2-1;yB=0; break;
        case 6: xA=0;yA=m/2-1;xB=n-1;yB=m/2+1; break;
        case 7: xA=n-1;yA=m/2+1;xB=0;yB=m/2-1; break;
    }

    cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
    cout<<"Start: "<<xA<<","<<yA<<endl;
    cout<<"Finish: "<<xB<<","<<yB<<endl;
    // get the route
    clock_t start = clock();
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
    cout<<"Route:"<<endl;
    cout<<route<<endl<<endl;

    // follow the route on the map and display it 
    if(route.length()>0)
    {
        int j; char c;
        int x=xA;
        int y=yA;
        map[x][y]=2;
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c); 
            x=x+dx[j];
            y=y+dy[j];
            map[x][y]=3;
        }
        map[x][y]=4;
    
        // display the map with the route
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++)
                if(map[x][y]==0)
                    cout<<".";
                else if(map[x][y]==1)
                    cout<<"O"; //obstacle
                else if(map[x][y]==2)
                    cout<<"S"; //start
                else if(map[x][y]==3)
                    cout<<"R"; //route
                else if(map[x][y]==4)
                    cout<<"F"; //finish
            cout<<endl;
        }
    }
    getchar(); // wait for a (Enter) keypress  
    return(0);
}*/
