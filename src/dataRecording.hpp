/*

sources: 
time
https://www.tutorialspoint.com/cplusplus/cpp_date_time.html
https://en.cppreference.com/w/cpp/chrono

*/

#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>

using namespace std;

/*! \brief dataRecoring
 * used to collect data from the test runs and save it to csv file 
 */
class dataRecording{
    public:
        dataRecording(string argument);
        int start();
        void writeSuccess(int usedNodes, long duration,std::chrono::duration<double> durationReal);

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

/*! \brief start
 * first data for new run is current date and time 
 */
int dataRecording::start(){
    time_t now = time(NULL);
    tm *ltm = localtime(&now);

    int year = 1900 + ltm->tm_year;
    int month = 1 + ltm->tm_mon;
    int day = ltm->tm_mday;
    int hour =  ltm->tm_hour;
    int minute = ltm->tm_min;
    int second = ltm->tm_sec;

    openFile(); 
    csvFile << "\n" << year << "." << month << "." << day << " ";
    csvFile << hour << ":" << minute << ":" << second << "; "; 
    closeFile();
}
/*! \brief writeSuccess
 * if turtlebot escaped from the maze following data is written to the csv file: 
 * ° used nodes (excl. the nodes that where explored but closed due to dead ends
 * ° duration (simulated time)
 * ° "ESCAPED" if the robot solved the maze 
 * ° duration (real time)
 */
void dataRecording::writeSuccess(int usedNodes, long duration,std::chrono::duration<double> durationReal){
    std::cout << std::endl << "duration: " << durationReal.count() << "s"<< std::endl;

    openFile(); 
	csvFile << usedNodes << "; " << int(duration/60) << ":" << (duration%60) << "; ESCAPED; ";
    csvFile << (long(durationReal.count())/60) << ":" << (long(durationReal.count())%60) <<";";
    closeFile();
}

