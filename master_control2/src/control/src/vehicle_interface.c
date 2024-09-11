
// Server side C/C++ program to demonstrate Socket
// programming

#include "vehicle_interface.h"

void manual_auto(int val){
	pinMode(MAN_AUTO,OUTPUT);
	digitalWrite(MAN_AUTO, val);
}

void blink (const int led, int value){

   softPwmWrite(led, value+2);
   delay(1);
}


void WAKE(){
	pinMode(WAKEUP, OUTPUT);
	digitalWrite(WAKEUP, HIGH);
}


void pulse_hearth(){
	digitalWrite(HEARTHBEAT, !digitalRead(HEARTHBEAT));
	//printf("%d",digitalRead(HEARTHBEAT));
}


void setup_pwm(const int pwm_pin, int freq){
	pinMode(pwm_pin,OUTPUT);
	softPwmCreate(pwm_pin,0,10000/freq); 
}



void truck_sleep(){
    blink(PWM_ACEL_P,0);
    blink(PWM_ACEL_N,0);
    blink(PWM_FREIO,0);
    blink(PWM_DIR,0);
    blink(PWM_ESQ,0);
    manual_auto(0);
//printf("CAMINHAO FOI A MIMIR");
}



void pwm_update(int accel, int steer, int brake){
    accel /= 2;
    brake /= 2;
    blink(PWM_ACEL_P,accel);
    blink(PWM_ACEL_N,50-accel);
    if(brake>0){
        blink(PWM_FREIO,brake);
        blink(PWM_ACEL_P,0);
        blink(PWM_ACEL_N,0);
}
    if(steer>=0){
        blink(PWM_DIR,steer);
        blink(PWM_ESQ,0);
}
    if(steer<0){
        blink(PWM_ESQ,steer*(-1));
        blink(PWM_DIR,0);
}

}

void setup_PWM_ports(){

    wiringPiSetup();
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(1920);
    //pwmSetRange(200);
    setup_pwm(PWM_ESQ,100);
    setup_pwm(PWM_DIR,100);
    setup_pwm(PWM_ACEL_N,200);
    setup_pwm(PWM_ACEL_P,200);
    setup_pwm(PWM_FREIO,200);

}

