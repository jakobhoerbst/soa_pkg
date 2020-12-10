/*
    PID Tuner for waffle
    input: ist-position = double currentPose[3]
    output: soll-position = double desiredPose[3]
    source: https://gist.github.com/bradley219/5373998
    -> Fur jeden pose[i] muss es gemacht werden, vereinfachen!
    -> Testen, testen, testen
    -> momentan noch falsch verlinkt, es wird eine main gesucht, die es nicht gibt in der make
*/
/*
EXAMPLE Use: 
    PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
        double dStartPos = currentPose[i], dEndPos = desiredPose[i];
        for (int i = 0; i < 100; i++) 
        {
            double dIncrement = pid.calculate(0, val);
            std::cout << "i: " << i << "   dEndPos: " << dEndPos << "   dIncrement: " << dIncrement << std::endl;
            dEndPos += dIncrement;
        }
*/

#include <iostream>
#include <cmath>

using namespace std;

class pidTuner
{
    public:
        pidTuner(double dt, float fkp, float fki, float fkd, double dmin, double dmax);
        // dmin = minimum value of tuner
        // dmax = maximum value of tuner
        double dCalculate (double dSetPoint, double dProcessValue); // next to array? 

    private:
        double dT, dMIN, dMAX;
        float fKP, fKI, fKD;
        double dCalcError;
        double dIntegral;


};

/**
 * Implementation
 */
pidTuner::pidTuner( double dt, float fkp, float fki, float fkd, double dmin, double dmax) :
    dT(dt),
    fKP(fkp),
    fKI(fki),
    fKD(fkd),
    dMIN(dmin),
    dMAX(dmax),
    dCalcError(0),
    dIntegral(0)
{
}

double pidTuner::dCalculate( double dSetPoint, double dProcessValue )
{
    // Check if divide by 0
    if(dT == 0.0)
    {
        cout << "Impossible to create a PID regulator with a null loop interval time" << endl;
        return -1;
    }
    // Calculate error
    double dError = dSetPoint - dProcessValue;

    // Proportional term
    double dPout = fKP * dError;

    // Integral term
    dIntegral += dError * dT;
    double Iout = fKI * dIntegral;

    // Derivative term
    double dDerivative = (dError - dCalcError) / dT;
    double Dout = fKD * dDerivative;

    // Calculate total output
    double dOutput = dPout + Iout + Dout;

    // Restrict to max/min
    if( dOutput > dMAX )
        dOutput = dMAX;
    else if( dOutput < dMIN )
        dOutput = dMIN;

    // Save error to previous error
    dCalcError = dError;

    return dOutput;
}
