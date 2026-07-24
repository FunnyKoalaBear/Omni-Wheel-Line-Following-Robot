#include "mbed.h"
#include "HCSR04.h"
#include <cmath>
#include <algorithm>
#include <chrono> 

PwmOut pwm5(D0);
PwmOut pwm6(D1);
PwmOut pwm7(PA_12_ALT0);
PwmOut pwm8(PB_0_ALT0);

AnalogIn S0(A0); //right side 
AnalogIn S1(A1);
AnalogIn S2(A2);
AnalogIn S3(A3);
AnalogIn S4(A4); //left side, right and left mix up 
AnalogIn S5(A5); //leftMost Side 
AnalogIn S6(A6); //rightMost Side 



//Distance Sensor 
//VCC - 3V3, GND - GND, Echo - D5, Trig - D6 
HCSR04 sensor(D5, D6);


//const float threshold = 0.5f;
float minSpeed = 0.2f;
float baseSpeed= 0.2f;
float maxSpeed = 0.5f;

const float fast= 0.5f;
const float slow=0.1f;
const float stop=0.0f;


//to balance motor speed
const float LEFT_MOTOR_SCALE = 0.97f; //0.89
const float RIGHT_MOTOR_SCALE = 1.0f;


void stopAll() {
    pwm5.write(0.0f); //right motor, CW, frontwrds 
    pwm6.write(0.0f);
    pwm7.write(0.0f); //left motor, CW, frontwards 
    pwm8.write(0.0f);
}

void drive(float left, float right) {
    left = left * LEFT_MOTOR_SCALE;
    right = right * RIGHT_MOTOR_SCALE;

    if (left > 1.0f) left = 1.0f;
    if (left < -1.0f) left = -1.0f; 
    if (right > 1.0f) right = 1.0f;
    if (right < -1.0f) right = -1.0f;

    // Left Motor (Allows Reversing)
    if (left >= 0.0f) {
        pwm7.write(left); 
        pwm8.write(0.0f); 
    } else {
        pwm7.write(0.0f); 
        pwm8.write(fabs(left)); // Write absolute positive value to reverse pin
    }

    // Right Motor (Allows Reversing)
    if (right >= 0.0f) {
        pwm5.write(right);
        pwm6.write(0.0f);
    } else {
        pwm5.write(0.0f);
        pwm6.write(fabs(right)); // Write absolute positive value to reverse pin
    }
}

int maneuverCnt = 1; 
bool waiting_for_maneuver_2 = true;

int maneuver(int number) {
    if (number == 1) {
        
        // 1. DEEPER DODGE LEFT
        // Lowered left to 0.1 and raised right to 0.5 for a sharper exit angle.
        // Increased time from 500ms to 700ms to travel much further outward.
        drive(0.1, 0.6);
        ThisThread::sleep_for(500ms);  //700

        // 2. STRAIGHT TO PASS
        // Increased to 800ms to ensure the whole robot clears the object 
        // now that it is starting from further away.
        drive(0.3, 0.3);
        ThisThread::sleep_for(300ms); 
        
        // 3. SHALLOW ARC RIGHT TO MERGE
        // Left motor stays faster to gently steer it back toward the line.
        drive(0.35, 0.15); //0.3
        
        // Force a tiny delay so it commits to the turn before looking for the line
        ThisThread::sleep_for(200ms);

        while (true) {
            // Still checking only the middle sensors for a straight merge!
            if (S1.read() > 0.6f || S2.read() > 0.6f || S3.read() > 0.6f) {
                break; 
            }
            ThisThread::sleep_for(10ms); 
        }
        
        stopAll();        
        return 1; 
    }
    else if(number == 3) {


        drive(-0.3, -0.3);
        ThisThread::sleep_for(600ms); 
                // 1. DEEPER DODGE LEFT
        // Lowered left to 0.1 and raised right to 0.5 for a sharper exit angle.
        // Increased time from 500ms to 700ms to travel much further outward.
        drive(0.1, 0.6);
        ThisThread::sleep_for(500ms);  //700

        // 2. STRAIGHT TO PASS
        // Increased to 800ms to ensure the whole robot clears the object 
        // now that it is starting from further away.
        drive(0.3, 0.3);
        ThisThread::sleep_for(600ms); 
        
        // 3. SHALLOW ARC RIGHT TO MERGE
        // Left motor stays faster to gently steer it back toward the line.
        drive(0.35, 0.15); //0.3
        
        // Force a tiny delay so it commits to the turn before looking for the line
        ThisThread::sleep_for(200ms);

        while (true) {
            // Still checking only the middle sensors for a straight merge!
            if (S1.read() > 0.6f || S2.read() > 0.6f || S3.read() > 0.6f) {
                break; 
            }
            ThisThread::sleep_for(10ms); 
        }
        
        stopAll();        
        return 1; 

    }

    else {
        // 1. REVERSE
        // Both motors negative to back up straight. 
        // Adjust time so it backs up far enough to clear the obstacle for the turn.
        
        baseSpeed = 0.2f; 
        waiting_for_maneuver_2 = false; // Stop the 20-second timer from slowing it down again

        drive(-0.3, -0.3);
        ThisThread::sleep_for(600ms); 
        
        // 2. TURN LEFT (Face left)
        // Left motor negative, right motor positive creates a zero-radius spin to the left.
        // Adjust the sleep time until the robot is pointed exactly where you want it.
        drive(-0.3, 0.3);
        ThisThread::sleep_for(300ms); 
        
        // 3. DRIVE FORWARD TO NEXT LINE
        drive(0.3, 0.3);
        
        // Blind timer to ensure the robot gets completely off the starting line 
        // before it starts looking for the destination line.
        ThisThread::sleep_for(200ms);
        
        while (true) {
            // Check middle sensors to catch the diagonal track
            if (S1.read() > 0.6f || S2.read() > 0.6f || S3.read() > 0.6f) {
                break; 
            }
            ThisThread::sleep_for(10ms); 
        }
        
        stopAll();
        return 1;
    }
    
    return -1;
}


//variables for EOT check 
using namespace std::chrono;
Timer t;
int state = false; 
bool previously_on_line = false;

//PID, error, integral, derivative

float Proportional;
float Integral; 
float Derivative; 
float current_error = 0; 
float setpoint = 0; 
float position; 
float last_error = 0; 

float weighted_sum = 0.0f;
int active_sensors = 0;

float Kp = 0.14f; //0.2 
float Ki = 0.00f; 
float Kd = 0.24f;
float correction = 0.0f;

bool empty; 
bool currently_on_line;



int main() {
    pwm5.period(0.005f);
    pwm7.period(0.005f);    
    pwm6.period(0.005f);
    pwm8.period(0.005f);    

    //pwm5.period(0.0005f); // 2kHz
    //pwm7.period(0.0005f);
    // pwm5.period(0.001f);
    // pwm7.period(0.001f); 
    
    //timer to slow down robot at maneuver 2. 
    Timer run_timer;
    run_timer.start();


    Timer dist_timer;
    dist_timer.start();
    sensor.trigger(); // Initial trigger for the distance sensor

    while (true) {
        float leftMost = S5.read()*200; 
        float left = S0.read(); //right side -2
        float leftmid = S1.read(); //-1
        float middle = S2.read(); //0
        float rightmid = S3.read(); //1
        float right = S4.read(); //left side 2
        float rightMost = S6.read();   



        // bool currently_on_line = (right > 0.6 && left > 0.6);
        bool currently_on_line = (right > 0.6 && left > 0.6);

        weighted_sum = 0.0f;
        active_sensors = 0; 


        //end of track EOT check 

        if (currently_on_line && !previously_on_line) {
            if (state == false) {
                //first perpendicular line passed, start timer.
                t.reset();
                t.start();
                state = true;
                //printf("Right aand left positive detected!\n");
            }
            else {
                //second perpendicular line passed, compare timer. 
                t.stop();
                long long elapsed_time_ms = duration_cast<milliseconds>(t.elapsed_time()).count();
                //printf("Line detected at: %llu ms\n", elapsed_time_ms);
                //printf("State is true!\n");
                //printf("The time taken was %llu milliseconds\n", elapsed_time_ms);

                if (elapsed_time_ms < 53) {
                    // DEBOUNCE: Time is too short. This is sensor noise on the same line.
                    // Do nothing. Do NOT change state, do NOT stop timer. 
                    // Let it keep running for the real second line.
                }
                if (elapsed_time_ms > 53 && elapsed_time_ms < 300) { //52
                    //printf("Second signal, stopping car.\n");
                    //printf("The time taken was %llu milliseconds\n", elapsed_time_ms);
                    drive(stop, stop);
                    ThisThread::sleep_for(5s);
                    state = false; 
                    //printf("Strating loop again!\n");
                }

                else {
                    //took too long
                    t.reset();
                    t.start();
                    state = true; 
                    //printf("Timeout! Treating current line as new first line.\n"); 
                }

            } 
        }

        previously_on_line = currently_on_line;
        
        
        //obstacle check 
        //sensor.trigger(); //read distance 

        if (duration_cast<milliseconds>(dist_timer.elapsed_time()).count() > 60) {

            uint32_t distance = sensor.distance_cm();
            //printf("Distance read is: %lu \n", static_cast<unsigned long>(distance));

            if (distance <= 20 && distance > 0) {
                //printf(" -> Obstacle Detected \r\n");
                //printf("Starting Maneuver\r\n");
                maneuver(maneuverCnt);
                maneuverCnt++;  
                // printf("Maneuver ended\r\n");
            }
            
            sensor.trigger(); // Trigger for the NEXT reading
            dist_timer.reset();
        }        


        // //PID
        // if (right > 0.6)        {weighted_sum += -2.0f; active_sensors++;}
        // if (rightmid > 0.6)     {weighted_sum += -1.0f; active_sensors++;}
        // if (middle > 0.6)       {weighted_sum +=  0.0f; active_sensors++;}
        // if (leftmid > 0.6)      {weighted_sum +=  1.0f; active_sensors++;} 
        // if (left > 0.6)         {weighted_sum +=  2.0f; active_sensors++;} 
        // if (rightMost > 0.6)    {weighted_sum +=  -3.0f; active_sensors++;} 
        // if (leftMost > 0.6)     {weighted_sum +=  3.0f; active_sensors++;} 

        if (right > 0.6)        {weighted_sum += -2.0f; active_sensors++;}
        if (rightmid > 0.6)     {weighted_sum += -1.0f; active_sensors++;}
        if (middle > 0.6)       {weighted_sum +=  0.0f; active_sensors++;}
        if (leftmid > 0.6)      {weighted_sum +=  1.0f; active_sensors++;} 
        if (left > 0.6)         {weighted_sum +=  2.0f; active_sensors++;} 
        if (rightMost > 0.6)    {weighted_sum +=  -3.0f; active_sensors++;} 
        if (leftMost > 0.6)     {weighted_sum +=  3.0f; active_sensors++;}

        
        if (waiting_for_maneuver_2 && duration_cast<seconds>(run_timer.elapsed_time()).count() >= 20) {
            baseSpeed = minSpeed;
        }

        if (active_sensors > 0) {
            position = weighted_sum/active_sensors;
        }
        else {
            //lost line, for the blank line space 
            if (fabs(last_error) <= 0.5) {
                drive(baseSpeed, baseSpeed);
            }
            else if (last_error > 0.5) {
                // Lost it off the left side, spin left in place to catch it
                drive(-0.15f, 0.15f); 
            }
            else if (last_error < -0.5) {
                // Lost it off the right side, spin right in place to catch it
                drive(0.15f, -0.15f);
            }
            else {
                stopAll();
                continue;
            }
        }

        float speed_penalty = fabs(current_error) * 0.06f; 
        
        float current_speed = baseSpeed - speed_penalty;

        // Clamp the speed so it never stops completely, maintaining some forward drive
        if (current_speed < 0.05f) {
            current_speed = 0.05f; 
        }
        
        // Reduce speed only for this specific cycle if the error is large
        if (fabs(last_error) >= 1.0) {
            current_speed = baseSpeed * 0.6f;
        }

        current_error = setpoint - position; 
        Proportional = current_error;
        Integral += current_error; 
        Derivative = current_error - last_error;
        last_error = current_error; 

        correction = Kp*Proportional + Ki*Integral + Kd*Derivative;

        //make code thaat slows down after some time         

        drive(current_speed+correction, current_speed-correction); 

        // printf("IR LeftMost: %.5f Left: %.5f LeftMid: %.5f Middle: %.5f RightMid: %.5f Right: %.5f RightMost: %.5f \r\n",
        // leftMost, left, leftmid, middle, rightmid, right, rightMost);
        
        thread_sleep_for(5); 
        // printf("\n");

        if (maneuverCnt == 4) maneuverCnt = 0;  

    }
}

//code before PID


