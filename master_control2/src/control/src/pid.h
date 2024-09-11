#define time_qual 0

#ifndef PID_H
#define PID_H

#include <chrono>
#include <cmath>


class PID{
    public:
        PID(double,double,double);
        void setP(double);
	    void setI(double);
	    void setD(double);
        void setPID(double, double, double);
        void setSetpoint(double);
        double getOutput(double);
        void setOutputRampRate(double);
        void setI_err_max(double);
        void reset();

    private:
        void init();
        double kp;
        double ki;
        double kd;
        double SetPoint;
        double error_Sum;
        double error_Sum_max;
        double Last_value;
        double I_err_max;

        double outputRampRate;
        double clamp(double,double,double);
        std::chrono::steady_clock::time_point last_time;      
};



#endif