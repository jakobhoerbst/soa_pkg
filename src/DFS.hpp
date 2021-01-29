/*  
    Maze Solving Algorithm:
    DFS - Depth First Search Algorithm 
    Version 1
    Autor: Hoerbst
    Contributor: Gmeiner
 */

#include "ros/ros.h"
                   
using namespace std;

/**
    * \brief Logic Depth-First Search Algorithm
**/
class DFSClass{
/**
    * \brief Depth-first search algorithm
    * this class holds the main logic for solving the maze. 
    * @param scan [360] Description
    * @param nextPos [2] Description
**/
public: 
    DFSClass(float scan[360], float nextPos[2]);
    bool handleNode();
    /// Pointer to main for currentPose
    float *currentPose; 
    int getNodeNumber();

private:
    const float width           = 1.25;     /*!< Maze Setup Value - grid length of one segment */    
    const float pathWidth       = 1.1;      /*!< Maze Setup Value - width for traversing with waffle */
    const float exitDistance    = 2.5;      /*!< treshold for scan feedback */
    const double closed         = 0.8;      /*!< Values from using cv::Mat for Visualization */
    const double current        = 0.8;      /*!< Values from using cv::Mat for Visualization */
    const double visited        = 0.4;      /*!< Values from using cv::Mat for Visualization */

    struct nodestruct
    {
        double x;                           /*!< x-Value of node  */ 
        double y;                           /*!< y-Value of node  */
        double dir[5];                      /*!< Directions: Dead end, right, down, left, up  */                       
        int move;                           /*!< previous orientation */
    };
    vector<nodestruct> graph;               /*!< directed graph of nodestructs - this vector holds main information for the algorithm*/

    struct coordinatesStruct
    {
        double x;                           /*!< x-value of visited node  */ 
        double y;                           /*!< y-value of visited node  */
    };
    vector<coordinatesStruct> visitedVector; /*!< vector of visited nodes */

    // pointer to main 
    float *scanResult;                      /*!< pointer to scanResults form main */
    float *nextPosition;                    /*!< pointer to next Position form main */

    // orientation 
    int     orientation = 0;                        /*!< ... of ...  */
    float   orientedDistances[5] = {0,0,0,0,0};     /*!< ... of ...  */

    //
    bool    newMovement = true; 
    bool    escaped = false; 

    // METHODS
    void directions();
    bool checkExit();
    int getOrientation();
    void setStatus(vector<nodestruct> &node, double newStatus);
    void scan(vector<nodestruct> &node);
    bool checkIfVisited(vector<nodestruct> &node);
    void correctNodePosition(vector<nodestruct> &node);
    void printNode(vector<nodestruct> currentNode);

};

DFSClass::DFSClass(float scan[360], float nextPos[2]): scanResult(scan), nextPosition(nextPos)
{
    graph.push_back(nodestruct());
    visitedVector.push_back(coordinatesStruct()); 
}

////////////////////////////      getNodeNumber     ////////////////////////////
/**
 * \brief getNodeNumber
 * returns size of vector (Number of used nodes) 
 **/
int DFSClass::getNodeNumber()
{
    return graph.size();
}

////////////////////////////       handleNode       ////////////////////////////
/**
 * \brief handleNode 
 * main method of DFS algorithm. Decides for next movement 
 **/
bool DFSClass::handleNode()
{
    cout << "__________________________________" << endl; 
    cout << "DFS algorithm:" << endl; 

    getOrientation();
    directions();
    if(checkExit())
    {
        cout << endl; 
        cout << "----------------------------------" << endl; 
        cout << "            EXIT FOUND" << endl; 
        cout << "----------------------------------" << endl;
        cout << endl << endl << "press \"q\" in the main terminal to close" << endl;  
        escaped = true;
        return true; 
    }

    // scan if new node is reached
    if(newMovement)
    {
        scan(graph);
        newMovement = false; 
        setStatus(graph, visited);
        correctNodePosition(graph);
    }

    // set previous direction (necessary for first node)
    int prevDirection; 
    if(graph.size()<2)
        prevDirection = 1; 
    else
        prevDirection = graph[graph.size()-2].move;

    // deciding next movement (prefered: keep previous direction)
    for(int i = 0; i < 4; i++)
    {
        if((prevDirection+i)>4)
            prevDirection -= 4; 

        if(graph[graph.size()-1].dir[(prevDirection+i)] == 0)
        { 
            graph[graph.size()-1].move = (prevDirection+i); 
            newMovement = true; 
            break; 
        }
    }    
    if(!newMovement)
        graph[graph.size()-1].move = 0;

    // print the current node
    printNode(graph);

    //move
    int motion = graph[graph.size()-1].move;
    switch(motion)
    {
        case 0: // move back 
            cout << "  - DEAD END -" << endl; 
            graph.pop_back();
            setStatus(graph, closed);
            
            break;
        case 1: // move right
            graph.push_back(nodestruct());
            graph[graph.size()-1].x = graph[graph.size()-2].x + width;
            graph[graph.size()-1].y = graph[graph.size()-2].y;
            newMovement = !checkIfVisited(graph);         
            break;
        case 2: // move dowm
            graph.push_back(nodestruct());
            graph[graph.size()-1].x = graph[graph.size()-2].x;
            graph[graph.size()-1].y = graph[graph.size()-2].y - width;
            newMovement = !checkIfVisited(graph); 
            break;
        case 3: // move left
            graph.push_back(nodestruct());
            graph[graph.size()-1].x = graph[graph.size()-2].x - width;
            graph[graph.size()-1].y = graph[graph.size()-2].y;
            newMovement = !checkIfVisited(graph);
            break;
        case 4: // move up
            graph.push_back(nodestruct());
            graph[graph.size()-1].x = graph[graph.size()-2].x;
            graph[graph.size()-1].y = graph[graph.size()-2].y + width;
            newMovement = !checkIfVisited(graph); 
            break; 
    }

    // next node
    nextPosition[0] = graph[graph.size()-1].x;
    nextPosition[1] = graph[graph.size()-1].y;

    return false; 
}

////////////////////////////       directions       ////////////////////////////
// calculates mean distances to fall for front, right, back, left side of turtle
/**
 * \brief Calculates mean distances
 *
 *  Calculates mean distances to fall for front, right, back, left side of turtle
 **/
void DFSClass::directions()
{
    float dirDistance[5] = {0,0,0,0,0}; 
    int range = 10; 

    for(int j = 0; j < 4; j++)
    {
        for(int i = j*90-range; i < j*90+range; i++)
        {
            if(i<0)
                dirDistance[j+1] += scanResult[i+360];
            else
                dirDistance[j+1] += scanResult[i];
        } 
        dirDistance[j+1] = dirDistance[j+1]/(2*range);
    }

    for(int i = 1; i < 5; i++)
    {
        if( (orientation+(i-1)) > 4 )
            orientedDistances[orientation+(i-1)-4] = dirDistance[i];
        else
            orientedDistances[orientation+(i-1)] = dirDistance[i]; 
    }   

}

/**
 * \brief Checks if exit is reached yet
 *
 *  This Method checks if the robot has reached one exit of the maze by ...
 **/
bool DFSClass::checkExit()
{
    int counter = 0; 
    for(int i = 1; i < 5; i++)
    {
        if(orientedDistances[i] > exitDistance) 
            counter ++;  
    }   

    if(counter >= 4) 
        return true; 

    return false; 
}


////////////////////////////    get orientation     ////////////////////////////
/**
 * \brief Gets the current Orientation
 **/
int DFSClass::getOrientation()
{
  //  cout << "orientation difference: " << orientation - initOrientation << endl; 
  //  cout << "initial Orientation" << initOrientation << endl; 
    //r, d, l, u    

    int toleranceAngle = 45; 
        
    if(abs(currentPose[2]) < toleranceAngle) 
        orientation = 1; 
    else if(currentPose[2] < 90+toleranceAngle && currentPose[2] > 90-toleranceAngle) 
        orientation = 4; 
    else if(abs(currentPose[2]) < 180 && abs(currentPose[2]) > 180-toleranceAngle) 
        orientation = 3; 
    else if(currentPose[2] < -90+toleranceAngle && currentPose[2] > -90-toleranceAngle) 
        orientation = 2;      
    else
        cout << "ERROR at finding orientation" << endl;  

    //cout << "\torientation: " << orientation << endl; 
    return orientation;
}

/*! \brief Brief description.
 */
void DFSClass::setStatus(vector<nodestruct> &node, double newStatus){

    int motion = 0; 

    if(newStatus == closed)
        motion = node[node.size()-1].move;
    else if(newStatus == visited) 
        motion = node[node.size()-2].move;
    else
        cout << "- ERROR: newStatus -" << endl;  

    switch(motion)
    {
        case 0: 
            break;
        case 1: 
            if(newStatus == closed)
                node[node.size()-1].dir[1] = newStatus;
            else
                node[node.size()-1].dir[3] = newStatus; 
            break; 
        case 2: 
            if(newStatus == closed)
                node[node.size()-1].dir[2] = newStatus;
            else            
                node[node.size()-1].dir[4] = newStatus; 
            break; 
        case 3: 
            if(newStatus == closed)
                node[node.size()-1].dir[3] = newStatus;
            else
                node[node.size()-1].dir[1] = newStatus; 
            break;
        case 4: 
            if(newStatus == closed)
                node[node.size()-1].dir[4] = newStatus;
            else            
                node[node.size()-1].dir[2] = newStatus; 
            break; 
    }

}

/*! \brief scan
 * uses scan values and writes to vector if there is a wall or not for all directions
 */
void DFSClass::scan(vector<nodestruct> &node)
{
    for(int i = 1; i < 5; i++)
    {
        if(orientedDistances[i] > pathWidth)
            node[node.size()-1].dir[i] = 0;
        else 
            node[node.size()-1].dir[i] = 1; 
    }

    node[node.size()-1].x = currentPose[0];
    node[node.size()-1].y = currentPose[1];
}

/*! \brief checkIfVistied 
 * checks the vector with visited nodes and compares if the new node has already been visited
 */
bool DFSClass::checkIfVisited(vector<nodestruct> &node)
{
    coordinatesStruct currentPos = {node[node.size()-1].x, node[node.size()-1].y};

    for(int i = 0; i < (visitedVector.size()-1); i++){
        if(visitedVector[i].x == node[node.size()-1].x && visitedVector[i].y == node[node.size()-1].y)
        {
            cout << " - VISITED BEFORE -" << endl; 
            //visumaze.at<double>(NV[NV.size()-1].y,NV[NV.size()-1].x) = 0.1;    
            node.pop_back();
            //nodelist.pop_back();
            setStatus(node, closed);
            return 1; 
        }
    }
    visitedVector.push_back(currentPos);
    return 0;
}

/*! \brief correctNodePosition 
 * turtlebot is not always on the exact position. therefore this position is corrected before it is saved to the graph 
 */
void DFSClass::correctNodePosition(vector<nodestruct> &node)
{
    float minimumPosition[2] = {-5.625, -5.625};
    
    for(int i = 0; i < 10; i++){
        if(abs(node[node.size()-1].x-(minimumPosition[0]+width*i)) < 0.2)
        {
            node[node.size()-1].x = (minimumPosition[0]+width*i);
        }
        if(abs(node[node.size()-1].y-(minimumPosition[1]+width*i)) < 0.2)
        {
            node[node.size()-1].y = (minimumPosition[1]+width*i);
        }
    }
}

/*! \brief printNode
 * print the current node to the terminal (for debugging) 
 */
void DFSClass::printNode(vector<nodestruct> currentNode)
{
    cout << "  node: " << currentNode.size();
    cout << " pos: ("  << currentNode[currentNode.size()-1].x;
    cout << ", "  << currentNode[currentNode.size()-1].y;
    cout << ")\tr: " << currentNode[currentNode.size()-1].dir[1] << 
            "\td: " << currentNode[currentNode.size()-1].dir[2] << 
            "\tl: " << currentNode[currentNode.size()-1].dir[3] << 
            "\tu: " << currentNode[currentNode.size()-1].dir[4] << 
            "\tmove: ";
    //dead end, r, d, l, u    
    switch(currentNode[currentNode.size()-1].move){
        case 0: 
            cout << "back" << endl; 
            break;
        case 1: 
            cout << "r" << endl; 
            break; 
        case 2: 
            cout << "d" << endl; 
            break; 
        case 3: 
            cout << "l" << endl; 
            break; 
        case 4:
            cout << "u" << endl; 
            break; 
        default: 
            cout << endl;    
            break;
    }
}
