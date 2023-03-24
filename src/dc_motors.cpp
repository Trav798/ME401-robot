
#include <Arduino.h>
#include "common.h"
#include "dc_motors.h"

#include <PID_v1.h>

#if defined(DCMOTOR1) || defined(DCMOTOR2)

int pidSampleTime = 10; // milliseconds
const int PWMFreq = 500; /* 5 KHz */
const int PWMResolution = 12;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

    #ifdef DCMOTOR1
        
        static portMUX_TYPE setpoint1Mutex = portMUX_INITIALIZER_UNLOCKED;
        static portMUX_TYPE measure1Mutex = portMUX_INITIALIZER_UNLOCKED;
        static portMUX_TYPE output1Mutex = portMUX_INITIALIZER_UNLOCKED;



        // Global variables used by the controller. 
        //    measure - the measured value from the encoders
        //    output - the controller output effort from the PID computation
        //    setpoint - the desired value of the output, sometimes called the reference 
        //                (in this case we are doing rotational position control)
        // **IMPORTANT**: the units of setpoint and input need to be the same so that the error computation 
        //                inside the PID object has the same units

        // Variables for measurement, output, and setpoint of DC motor 1 
        double measure1=0, output1=0, setpoint1=0;

        // TODO: STUDENTS NEED TO FIND THE RIGHT CONTROL GAINS FOR GOOD POSITION CONTROL
        // You can set the gains manually here, or inside main.cpp using function setPIDGains1(kp, ki, kd)
        double kp1=0.0,ki1=0.0,kd1=0.0;

        
        // 
        // double ku1 = 0.0; double tu1 = 0.0;
        // double kp1=0.0*ku1,ki1=0.0*kp1/tu1,kd1=kp1*tu1*0.0;
        

        PID pidMotor1(&measure1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);



        // Variables for DC motor 1 (DO NOT CHANGE)
        const int MOT1_CH_IN1 = 4;
        const int MOT1_CH_IN2 = 5;

        const char ENC1_CHANA_PIN = 14;
        const char ENC1_CHANB_PIN = 27;
        const char MOT1_IN1_PIN = 13;
        const char MOT1_IN2_PIN = 12;

        volatile long position1 = 0;
        volatile bool  lastEnc1A = 0;
        volatile bool  lastEnc1B = 0;
        volatile bool  newEnc1A = 0;
        volatile bool  newEnc1B = 0;
        volatile bool error1;
    #endif

    #ifdef DCMOTOR2
        static portMUX_TYPE setpoint2Mutex = portMUX_INITIALIZER_UNLOCKED;
        static portMUX_TYPE measure2Mutex = portMUX_INITIALIZER_UNLOCKED;

        // Variables for measurement, output, and setpoint of DC motor 1 
        double measure2=0, output2=0, setpoint2=0;

        // TODO: STUDENTS NEED TO FIND THE RIGHT CONTROL GAINS FOR GOOD POSITION CONTROL
        // double kp1=0.0,ki1=0.0,kd1=0.0;

        double ku1 = 0.0; double tu1 = 0.0;
        double kp1=0.0*ku1,ki1=0.0*kp1/tu1,kd1=kp1*tu1*0.0;

        PID pidMotor2(&measure2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);

        // Variables for DC motor 2
        const int MOT2_CH_IN1 = 6;
        const int MOT2_CH_IN2 = 7;

        const char ENC2_CHANA_PIN = 0;
        const char ENC2_CHANB_PIN = 4;
        const char MOT2_IN1_PIN = 15;
        const char MOT2_IN2_PIN = 2;

        volatile long position2 = 0;
        volatile bool  lastEnc2A = 0;
        volatile bool  lastEnc2B = 0;
        volatile bool  newEnc2A = 0;
        volatile bool  newEnc2B = 0;
        volatile bool error2;
    #endif

    // Set up the hardware timer for PID
    hw_timer_t * encoder_timer = NULL;
    void IRAM_ATTR onEncoderTimer() {


    // DC MOTOR1 quadrature encoder (DO NOT CHANGE) 
    #ifdef DCMOTOR1
        newEnc1A = digitalRead(ENC1_CHANA_PIN);
        newEnc1B = digitalRead(ENC1_CHANB_PIN);
    
        position1 += (newEnc1A ^ lastEnc1B) - (lastEnc1A ^ newEnc1B);
        
        if((lastEnc1A ^ newEnc1A) & (lastEnc1B ^ newEnc1B))
        {
            error1 = true;
            // Serial.println("MISSED STEPS");
        }
    
        lastEnc1A = newEnc1A;
        lastEnc1B = newEnc1B;   
    #endif

    #ifdef DCMOTOR2
        newEnc2A = digitalRead(ENC2_CHANA_PIN);
        newEnc2B = digitalRead(ENC2_CHANB_PIN);
    
        position2 += (newEnc2A ^ lastEnc2B) - (lastEnc2A ^ newEnc2B);
        
        if((lastEnc2A ^ newEnc2A) & (lastEnc2B ^ newEnc2B))
        {
            error2 = true;
            // Serial.println("MISSED STEPS");
        }
    
        lastEnc2A = newEnc2A;
        lastEnc2B = newEnc2B;   
    #endif



    }

    // Initialize timer interrupt 
    void setupEncoderInterrupt() {
        encoder_timer = timerBegin(0, 80, true);
        timerAttachInterrupt(encoder_timer, &onEncoderTimer, true);
        timerAlarmWrite(encoder_timer, 10, true);
        timerAlarmEnable(encoder_timer);
    }

    // Function that runs for timer interrupt
    void dcMotorsInterruptCoreTask( void * pvParameters ){
    

        setupEncoderInterrupt();

        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();

        const TickType_t xFrequency = pidSampleTime;
        UBaseType_t uxSavedInterruptStatus;

        while(1)
        {

        #ifdef DCMOTOR1
            uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

            // TODO: CONVERT POSITION FROM ENCODER TICKS TO ANGULAR POSITION. 
            measure1 = position1;

            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
            
            // PID COMPUTATION
            pidMotor1.Compute();
        
            if (output1 > 0)
            {
                ledcWrite(MOT1_CH_IN1, abs(output1));
                ledcWrite(MOT1_CH_IN2, 0);
            }
            else
            {
                ledcWrite(MOT1_CH_IN2, abs(output1));
                ledcWrite(MOT1_CH_IN1, 0);
            }  
        #endif

        #ifdef DCMOTOR2
            
            uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
            measure2 = position2;      
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

            pidMotor2.Compute();
        
            if (output2 > 0)
            {
                ledcWrite(MOT2_CH_IN1, abs(output2));
                ledcWrite(MOT2_CH_IN2, 1);
            }
            else
            {
                ledcWrite(MOT2_CH_IN2, abs(output2));
                ledcWrite(MOT2_CH_IN1, 1);
            }  
        #endif
        
            vTaskDelayUntil( &xLastWakeTime, xFrequency);
        }
    }


#endif




void setupDCMotors(void) 
{
#ifdef DCMOTOR1
    pinMode(ENC1_CHANA_PIN, INPUT_PULLDOWN);
    pinMode(ENC1_CHANB_PIN, INPUT_PULLDOWN);
    //setupEncoderInterrupt();

    ledcSetup(MOT1_CH_IN1, PWMFreq, PWMResolution);
    ledcSetup(MOT1_CH_IN2, PWMFreq, PWMResolution);
  
    ledcAttachPin(MOT1_IN1_PIN, MOT1_CH_IN1);
    ledcAttachPin(MOT1_IN2_PIN, MOT1_CH_IN2);
  
    ledcWrite(MOT1_CH_IN1, 0);
    ledcWrite(MOT1_CH_IN2, 0);

    pidMotor1.SetMode(AUTOMATIC);
    pidMotor1.SetSampleTime(pidSampleTime);
    pidMotor1.SetOutputLimits(-4095,4095);

#endif

#ifdef DCMOTOR2
    pinMode(ENC2_CHANA_PIN, INPUT_PULLDOWN);
    pinMode(ENC2_CHANB_PIN, INPUT_PULLDOWN);
    //setupEncoderInterrupt();

    ledcSetup(MOT2_CH_IN1, PWMFreq, PWMResolution);
    ledcSetup(MOT2_CH_IN2, PWMFreq, PWMResolution);
  
    ledcAttachPin(MOT2_IN1_PIN, MOT2_CH_IN1);
    ledcAttachPin(MOT2_IN2_PIN, MOT2_CH_IN2);
  
    ledcWrite(MOT2_CH_IN1, 0);
    ledcWrite(MOT2_CH_IN2, 0);

    pidMotor2.SetMode(AUTOMATIC);
    pidMotor2.SetSampleTime(pidSampleTime);
    pidMotor2.SetOutputLimits(-4095,4095);
#endif

#if defined(DCMOTOR1) || defined(DCMOTOR2)    
    xTaskCreatePinnedToCore(
                    dcMotorsInterruptCoreTask,   /* Function to implement the task */
                    "dcMotorsInterruptCoreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    controlCore);  /* Core where the task should run */

  Serial.println("PID motor control task created...");
#endif

}

#ifdef DCMOTOR1
void setSetpoint1(double setpoint)
{
    taskENTER_CRITICAL(&setpoint1Mutex);
    setpoint1 = setpoint;
    taskEXIT_CRITICAL(&setpoint1Mutex);
}

void setPIDgains1(double kp, double ki, double kd)
{
    kp1 = kp;
    ki1 = ki;
    kd1 = kd;

    pidMotor1.SetTunings(kp1,ki1,kd1);
}


double getSetpoint1(void)
{
    double retval = 0;
    taskENTER_CRITICAL(&setpoint1Mutex);
    retval = setpoint1;
    taskEXIT_CRITICAL(&setpoint1Mutex);
    return retval;
}

double getError1(void)
{
    double setpoint = getSetpoint1();
    double position = getPosition1();
    return setpoint-position;
}

double getPosition1(void)
{
    double retval = 0;
    taskENTER_CRITICAL(&measure1Mutex);
    retval = measure1;
    taskEXIT_CRITICAL(&measure1Mutex);

    return retval;
}

double getOutput1(void)
{
    double retval = 0;
    taskENTER_CRITICAL(&output1Mutex);
    retval = output1;
    taskEXIT_CRITICAL(&output1Mutex);
    return retval;

}

#endif

#ifdef DCMOTOR2
void setSetpoint2(double setpoint)
{
    taskENTER_CRITICAL(&setpoint2Mutex);
    setpoint2 = setpoint;
    taskEXIT_CRITICAL(&setpoint2Mutex);
}

void setPIDgains2(double kp, double ki, double kd)
{
    kp2 = kp;
    ki2 = ki;
    kd2 = kd;

    pidMotor2.SetTunings(kp2,ki2,kd2);
}

double getSetpoint2(void)
{
    double retval = 0;
    taskENTER_CRITICAL(&setpoint2Mutex);
    retval = setpoint2;
    taskEXIT_CRITICAL(&setpoint2Mutex);
    return retval;
}

double getError2(void)
{
    double setpoint = getSetpoint2();
    double position = getPosition2();
    return setpoint-position;
}

double getPosition2(void)
{
    double retval = 0;
    taskENTER_CRITICAL(&measure2Mutex);
    retval = measure2;
    taskEXIT_CRITICAL(&measure2Mutex);

    return retval;
}

double getOutput2(void)
{
    double retval = 0;
    taskENTER_CRITICAL(&output2Mutex);
    retval = output2;
    taskEXIT_CRITICAL(&output2Mutex);
    return retval;

}

#endif

