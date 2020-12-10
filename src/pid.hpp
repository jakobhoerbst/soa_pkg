/*
    PID Tuner for waffle
    input: ist-position = double currentPose[3]
    output: soll-position = double desiredPose[3]
    source: https://gist.github.com/bradley219/5373998
    -> Fur jeden pose[i] muss es gemacht werden, vereinfachen!
    -> Testen, testen, testen

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
        pidTuner(float fdt, float fkp, float fki, float fkd, float fmin, float fmax);
        // dmin = minimum value of tuner
        // dmax = maximum value of tuner
        float dCalculate (float fSetPoint, float fProcessValue); // next to array? 

    private:
        float fDT, fMIN, fMAX;
        float fKP, fKI, fKD;
        float fCalcError;
        float fIntegral;


};

pidTuner::pidTuner( float fdt, float fkp, float fki, float fkd, float fmin, float fmax) :
    fDT(fdt),
    fKP(fkp),
    fKI(fki),
    fKD(fkd),
    fMIN(fmin),
    fMAX(fmax),
    fCalcError(0),
    fIntegral(0)
{
}

float pidTuner::dCalculate( float fSetPoint, float fProcessValue )
{
    // Check if divide by 0
    if(fDT == 0.0)
    {
        cout << "Impossible to create a PID regulator with a null loop interval time" << endl;
        return -1;
    }
    // Calculate error
    float fError = fSetPoint - fProcessValue;

    // Proportional term
    float fPout = fKP * fError;

    // Integral term
    fIntegral += fError * fDT;
    float fIout = fKI * fIntegral;

    // Derivative term
    float fDerivative = (fError - fCalcError) / fDT;
    float fDout = fKD * fDerivative;

    // Calculate total output
    float fOutput = fPout + fIout + fDout;

    // Restrict to max/min
    if( fOutput > fMAX )
        fOutput = fMAX;
    else if( fOutput < fMIN )
        fOutput = fMIN;

    // Save error to previous error
    fCalcError = fError;

    return fOutput;
}
