#include "model_free.h"

/**
 * @brief Construct a new PID::PID object with initial values for P I and D. Adapted code from https://github.com/tekdemo/MiniPID 
 * 
 * @param P_ Proportional term
 * @param I_ Integrative term
 * @param D_ Derivative Term
 */
MF::MF(double P_,double I_,double D_,double Ts_, int N_,double alpha_){
    init();
    kp =P_; ki =I_; kd =D_;Ts= Ts_; N = N_; alpha = alpha_;
}

/**
 * @brief Init function for the controller.
 * 
 */
void MF::init(){
    error_Sum = 0;
    error_Sum_max = 0;
    I_err_max = 0;
    Last_value = 0;
    SetPoint = 0;
    kp=0;ki=0;kd=0;
    n=0;
    I_1 =0; I_2 =0; phi =0;
    error_Sum =0; last_value =0;last_output=0;
    last_time = std::chrono::steady_clock::now();

}

void MF::reset(){
    error_Sum =0;
    Last_value = 0;
    I_1 =0; I_2 =0; phi =0;
    error_Sum =0; last_value =0;last_output=0;
    last_time = std::chrono::steady_clock::now();
}

/**
 * @brief Set P term for PID control
 * 
 * @param P_ New Proportional term
 */
void MF::setP(double P_){
    kp = P_;
}

/**
 * @brief Set I term. Scales the error Sum to compensate for the change if it happens during operation.
 * 
 * @param I_ New Integrative Term
 */
void MF::setI(double I_){
    if(ki!=0){
        error_Sum = error_Sum*ki/I_;
    }
    ki = I_;
}


/**
 * @brief Set Derivative Term
 * 
 * @param D_ New Derivative Term
 */
void MF::setD(double D_){
    kd = D_;
}


void MF::setPID(double P_, double I_, double D_){
    setP(P_); setI(I_);setD(D_);

}
void MF::setSetpoint(double SetPoint_){
    SetPoint = SetPoint_;
}

/**
 * @brief Set a maximum value that the outputs can change. It prevents the controller from exploding with a high signal
 * 
 * @param Rate 
 */
void MF::setOutputRampRate(double Rate){
    outputRampRate = Rate;
}

/**
 * @brief Set the max value for the integrative error. It also updates the value for the maximum Integrative term value
 * 
 * @param I_err_max_ 
 */
void MF::setI_err_max(double I_err_max_){
    I_err_max = I_err_max_;
    if(ki!=0){
        error_Sum_max = I_err_max/ki;

    }
}

/**
 * @brief Main function for the controller. First it calculates the elapsed time. Calculate the errors. Calculates the p, i, d output terms. Calculate the outputs. Clamp for the Output ramp. Return
 * 
 * @param Current_value Double value coming from the CAN network
 * @return double Output value of the controller. The PWM scaling its done on the main program.
 */
double MF::getOutput(double Current_value){

    double Output = 0; 
    auto current_time = std::chrono::steady_clock::now();

    auto elapsed_time = std::chrono::duration<double>(current_time - last_time).count();//SECONDS
    if(time_qual == 0) elapsed_time = 1; //IF NOT USING TIME CALCULATION FOR THE LOOP SET THE TIME TO 1 to IGNORE IT



    //Calculate the integrative estimation. If done update the phi estimation. At the end, it uses the current phi to calculate the output with PID

    double Tau = n*Ts;
    double T = N*Ts;


    //PHI ESTIMATION
    if(n == 0 || n==N){
        I_1 += T *Current_value;
        I_2 += 0;
    }
    else{
        I_1 += 2*((T - 2*Tau)*Current_value);
        I_2 += 2*alpha*Tau*(T-Tau)*last_output;
    }
    if( n== N){
        I_1 = I_1*Ts/2;
        I_2 = I_2*Ts/2;
        phi = -6*(I_1 +I_2)/(T*T*T);
        I_1 = T *Current_value;
        I_2 = 0;
        n=0;
    }

    //Calculate the errors
    double error = SetPoint - Current_value;
    double p_error = (kp)*error; //Proportional term


    error_Sum += (  error );//Integrative term
    //Limit the error sum
    if(error_Sum_max!=0 && (error_Sum) > error_Sum_max) error_Sum = error_Sum_max;

    double i_err = (ki * elapsed_time)* error_Sum;
    //Limit the I term directly also
    if(I_err_max!=0){
        i_err = clamp(i_err,-I_err_max,I_err_max);
    }
    //AntiWindup check

    double d_error = (kd/elapsed_time)*(Current_value - Last_value); //Derivative term

    Output = - (phi -dref + p_error + i_err - d_error)/alpha;

    //Clamp the output according to ramp rate. This avoid high spikes in the output
    if(outputRampRate!=0){
        Output = clamp(Output,Output-outputRampRate,Output+outputRampRate);
    }
    //Update values for next calc
    Last_value = Current_value;
    last_time = current_time;


    dref = 0;//Only is differente from 0 if SetSetpoint is triggered during the loop
    last_output = Output;
    last_value = Current_value;
    n++;

    return Output;
}



double MF::clamp(double num, double lower_lim, double upper_lim){
    if(num > upper_lim) return upper_lim;
    else if(num < lower_lim) return lower_lim;
    return num;
}


