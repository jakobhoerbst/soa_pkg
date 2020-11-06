/*
    jakob hoerbst
    03.11.2020
    MMR3 - Mechatronik & Robotik - SOA

    SOURCES: 
    struct from function
    https://www.programiz.com/c-programming/c-structure-function
    struct
    http://www.cplusplus.com/doc/tutorial/structures/
    vector
    https://www.geeksforgeeks.org/vector-in-cpp-stl/
    getline 
    http://www.cplusplus.com/reference/string/string/getline/
    read from file 
    https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
*/

#include <iostream> 
#include <opencv2/opencv.hpp> 
#include <opencv2/xfeatures2d.hpp>
#include <chrono>
#include <opencv2/core/types.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <sstream>

using namespace std; 

static double closed = 0.8; 
static double current = 0.8;
static double visited  = 0.4;
cv::Mat visumaze; 

struct nodestruct{
    int x; 
    int y; 

    double dir[5]; //r, d, l, u

    int move; 
};
 
////////////////////////////          show          ////////////////////////////
void show(cv::Mat mazeshow){
    cv::namedWindow("maze", CV_WINDOW_NORMAL); 
    cv::resizeWindow("maze", 900, 900); 
    cv::imshow("maze", mazeshow); 
}

////////////////////////////         scan           ////////////////////////////
struct nodestruct scan(cv::Mat mazescan, struct nodestruct node){
    
    node.dir[4] = mazescan.at<double>(node.y-2,node.x); 
    node.dir[1] = mazescan.at<double>(node.y,node.x+2); 
    node.dir[2] = mazescan.at<double>(node.y+2,node.x); 
    node.dir[3] = mazescan.at<double>(node.y,node.x-2);   

    return node;
}

////////////////////////////     checkForExit       ////////////////////////////
int checkForExit(struct nodestruct env){

    for(int i = 1; i < 5; i++){
        if(!(env.dir[i] == closed || env.dir[i] == current || env.dir[i] == visited || env.dir[i] == 0 || env.dir[i] == 1)) 
        return 1;
    }
    return 0;
}

////////////////////////////        getMaze         ////////////////////////////
void getMaze(string filename, double arr[45][45]){

    std::ifstream inputfile("maze.csv");
    std::string line;
    int value = 0;  
    int x = 0; 
    int y = 0;

    while(std::getline(inputfile, line)){
        stringstream ss(line); 

        string valuestring; 
        while(getline(ss,valuestring, ';')){

            if(valuestring == "0") 
                arr[y][x] = 0.0; 
            
            else
                arr[y][x] = 1.0; 
            
            std::cout.flush(); 
            x ++; 

        }
        arr[0][0] = 0; 
        arr[y][44] = 0; 
        y ++; 
        x = 0; 

    }
    inputfile.close();

}

////////////////////////////       setStatus        ////////////////////////////
vector<nodestruct> setStatus(vector<nodestruct> node, double newStatus){

    int motion = 0; 

    if(newStatus == closed)
        motion = node[node.size()-1].move;
    else if(newStatus == visited) 
        motion = node[node.size()-2].move;
    else
        cout << "- ERROR -" << endl;  

    switch(motion){
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

    return node;
}

////////////////////////////       printNode        ////////////////////////////
void printNode(vector<nodestruct> currentNode){

    cout << "node: " << currentNode.size();
    cout << " \tx: " << currentNode[currentNode.size()-1].x;
    cout << "\ty: " << currentNode[currentNode.size()-1].y;
    cout << "\tu: " << currentNode[currentNode.size()-1].dir[4] << 
            "\tr: " << currentNode[currentNode.size()-1].dir[1] << 
            "\td: " << currentNode[currentNode.size()-1].dir[2] << 
            "\tl: " << currentNode[currentNode.size()-1].dir[3] << 
            "\tmove: " << currentNode[currentNode.size()-1].move << endl;

}

////////////////////////////     checkIfVisited     ////////////////////////////
bool checkIfVisited(vector<nodestruct> &NV){
    visumaze.at<double>(NV[NV.size()-2].y,NV[NV.size()-2].x) = visited;    

    for(int i = 0; i < (NV.size()-1); i++){
        if(NV[i].x == NV[NV.size()-1].x && NV[i].y == NV[NV.size()-1].y){
            cout << " - VISITED BEFORE -" << endl; 
            visumaze.at<double>(NV[NV.size()-1].y,NV[NV.size()-1].x) = 0.1;    
            NV.pop_back();
            //nodelist.pop_back();
            NV = setStatus(NV, closed);
            return 1; 
        }
    }
    return 0;
}


////////////////////////////                        ////////////////////////////
////////////////////////////          main          ////////////////////////////
////////////////////////////                        ////////////////////////////
int main(){
 
    // get maze to Mat from csv file 
    double mazearr[45][45];
    getMaze("maze.csv", mazearr);
    cv::Mat maze(45, 45, CV_64F, mazearr);
    visumaze = maze; 

    // tree: vector of struct
    vector<nodestruct> nodelist; 
    nodelist.push_back(nodestruct());

    // spawn to random position within the maze
    std::srand (time(NULL));

    int rand01 = std::rand()%10 + 1;
    int rand02 = std::rand()%10 + 1;
    
    nodelist[0].x = 4*rand01; 
    nodelist[0].y = 4*rand02; 
    nodelist[0].move = 0; 

    int move = 0;
    bool newMovement = true; 
        
    while(1){
        //cout << "_________________________________________________" << endl; 

        // show maze simulation
        visumaze.at<double>(nodelist[nodelist.size()-1].y,nodelist[nodelist.size()-1].x) = current;    
        show(visumaze); 
        visumaze.at<double>(nodelist[nodelist.size()-1].y,nodelist[nodelist.size()-1].x) = 0; 
        
        // scan if new node is reached
        if(newMovement){
            nodelist[nodelist.size()-1] = scan(maze, nodelist[nodelist.size()-1]);
            newMovement = false; 
            nodelist = setStatus(nodelist, visited);
        }
        
        // check if maze is solved
        if(checkForExit(nodelist[nodelist.size()-1])){
            cout << " - EXIT FOUND - " << endl; 
            break;
        }
        
        // set previous direction (necessary for first node)
        int prevDirection; 
        if(nodelist.size()<2)
            prevDirection = 1; 
        else
            prevDirection = nodelist[nodelist.size()-2].move;
 
        // deciding next movement (prefered: keep previous direction)
        for(int i = 0; i < 4; i++){
            if((prevDirection+i)>4)
                prevDirection -= 4; 

            if(nodelist[nodelist.size()-1].dir[(prevDirection+i)] == 0){ 
                nodelist[nodelist.size()-1].move = (prevDirection+i); 
                newMovement = true; 
                break; 
            }
          
        } 
        if(!newMovement)
            nodelist[nodelist.size()-1].move = 0;


        printNode(nodelist); 

        //move
        int motion = nodelist[nodelist.size()-1].move;
        switch(motion){
            case 0: // move back 
                cout << " - DEAD END -" << endl; 
                nodelist.pop_back();
                nodelist = setStatus(nodelist, closed);
                
                break;
            case 1: // move right
                nodelist.push_back(nodestruct());
                nodelist[nodelist.size()-1].x = nodelist[nodelist.size()-2].x + 4;
                nodelist[nodelist.size()-1].y = nodelist[nodelist.size()-2].y;
                newMovement = !checkIfVisited(nodelist);         
                break;
            case 2: // move dowm
                nodelist.push_back(nodestruct());
                nodelist[nodelist.size()-1].x = nodelist[nodelist.size()-2].x;
                nodelist[nodelist.size()-1].y = nodelist[nodelist.size()-2].y + 4;
                newMovement = !checkIfVisited(nodelist); 
                break;
            case 3: // move left
                nodelist.push_back(nodestruct());
                nodelist[nodelist.size()-1].x = nodelist[nodelist.size()-2].x - 4;
                nodelist[nodelist.size()-1].y = nodelist[nodelist.size()-2].y;
                newMovement = !checkIfVisited(nodelist);
                break;
            case 4: // move up
                nodelist.push_back(nodestruct());
                nodelist[nodelist.size()-1].x = nodelist[nodelist.size()-2].x;
                nodelist[nodelist.size()-1].y = nodelist[nodelist.size()-2].y - 4;
                newMovement = !checkIfVisited(nodelist); 
                break; 

        }


        cv::waitKey(100);
        //cv::waitKey(0);    
    }
 
    return 0; 
}
