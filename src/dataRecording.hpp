/*

next steps: 

sources: 
time
https://www.tutorialspoint.com/cplusplus/cpp_date_time.htm

*/
#include <fstream>

#include <ctime>


using namespace std;


class dataRecording{
    public:
        dataRecording(string argument);
        int start();
        void writeSuccess(int usedNodes, long duration);

       
    private:
        ofstream csvFile;
        string fileName = "dataRecording/testResult_";
        int openFile();
        void closeFile();

};

dataRecording::dataRecording(string argument){

    fileName += argument;
    fileName += ".csv"; 

}

int dataRecording::openFile(){

	csvFile.open(fileName, ios::in | ios::app);
    if (!csvFile.is_open()) 	{  
        cout << "Could not load CSV-File: " << fileName << endl;
        cin.get();
        return 0;
    }  

}

void dataRecording::closeFile(){

    csvFile.close(); 

}


int dataRecording::start(){

    time_t now = time(NULL);
    tm *ltm = localtime(&now);

    int year = 1900 + ltm->tm_year;
    int month = 1 + ltm->tm_mon;
    int day = ltm->tm_mday;
    int hour =  ((ltm->tm_hour)+9)%24;
    int minute = ltm->tm_min;
    int second = ltm->tm_sec;

    openFile(); 
    csvFile << year << "." << month << "." << day << " ";
    csvFile << hour << ":" << minute << ":" << second << "; "; 
    closeFile();
}

void dataRecording::writeSuccess(int usedNodes, long duration){

    openFile(); 
	csvFile << usedNodes << "; " << int(duration/60) << ":" << (duration%60) << "; ESCAPED;";	
    closeFile();

}


/*
// Write data to csv
unsigned int counterNodes = navigation.cntNode; 
cout << "Counter Nodes: " << navigation.cntNode << endl;
csvDataInput << "Counter Nodes: ";
csvDataInput << counterNodes;
csvDataInput.close();
*/
