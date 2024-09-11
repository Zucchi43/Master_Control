#define time_qual 0

#ifndef MF_H
#define MF_H

#include <chrono>
#include <cmath>


class MF{
    public:
        MF(double,double,double,double,int,double);
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

        double Ts;
        int N;
        int n;
        double I_1;
        double I_2;
        double phi;
        double alpha;
        double last_value;
        double last_output;
        double dref;  
};



#endif