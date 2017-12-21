#include "mbed.h"
#include "QEI.h"
#include "DigitalIn.h"
#include "AnalogIn.h"
/*PWM for MOTOR DECLARATION*/
PwmOut RightMotorForward(PB_10); //pa7
PwmOut RightMotorBackward(PC_7);//pb6 //PC_7
PwmOut LeftMotorForward(PB_6); //pc7
PwmOut LeftMotorBackward(PA_7); //pb10 //PA_7

//seems like ideal left and ideal right are...
/*IDEAL IR LEFT RIGHT VALUES */
double idealRight=.45;
double idealLeft=.32;



double TOOCLOSE=0.85; //will have to adjust TOOCLOSE double constant for when IR is too close.
/*IR DECLARATIONS*/
AnalogIn FrontLeftIRReceiver(PC_1); //pb0
AnalogIn FrontRightIRReceiver(PA_4); //pc11
AnalogIn BackLeftIRReceiver(PC_0); //pb7
AnalogIn BackRightIRReceiver(PA_0); //pc10

DigitalOut FrontLeftIRTransmitter(PB_0);
DigitalOut FrontRightIRTransmitter(PC_11);
DigitalOut BackLeftIRTransmitter(PB_7);
DigitalOut BackRightIRTransmitter(PC_10);

//for future consideration, choose resistor for corresponding pulse time for the IR (so turn off at intervals not reading IR values)

/*PID ENCODER CONSTANTS*/
double Kp=.0005;
double Kd=.0005;
double Ki=.000;
double integral=0;
int errorValue=0;
int prevErrorValue=0;
double errorValueIR=0;
Timer timer;
void PIDStraightEncoder();
/*PID IR CONSTANTS AND FUNCTION*/
double pController(int error);
double dController(int error);
double pControllerIR(double error);
double dControllerIR(double error);
double iController(int error);
double KpIR=1.2;
double KdIR=1.1;
double KiIR=.000;



// in the function , adjust these and make it vary off of encoder and ir.

/*MOTOR CONSTANTS*/
double RightMotorSpeed=0.12;
double LeftMotorSpeed=0.13;

/*QEI MOTOR CLOCK COUNTS*/
int revolutionWheelRight=0;
int revolutionWheelLeft=0;
QEI rightWheelEncoder(PB_3, PA_15, NC, revolutionWheelRight);
QEI leftWheelEncoder(PA_1, PC_4, NC, revolutionWheelLeft);
/*FUNCTION DECLARATIONS*/
void motorStop();
void turnLeft();
void turnRight();
bool wallDETECTED();
bool wallOnBothSides();
bool wallOnRightSide();
bool wallOnLeftSide();
/*IR CONSTANTS*/
double FrontLeftIRvalue=0;
double FrontRightIRvalue=0;
double BackLeftIRvalue=0;
double BackRightIRvalue=0;

/*IR THRESHOLD CONSTANTS FOR TURNS*/
double RightIRTHRESHOLD=0.86;
double LeftIRTHRESHOLD=0.86;

/*TURN CONSTANTS*/  //count how many encoder turns is a 90 degree turn and use that as the delay
double AdjustedRightMotorSpeedturnLeft=0.25;
double AdjustedLeftMotorSpeedturnLeft=0.25;
double AdjustedRightMotorSpeedturnRight=0.25;
double AdjustedLeftMotorSpeedturnRight=0.25;

//random pid I constant helper , but i did turn off I so idk...
int df=45;

int main()
{
    timer.start();
    
    while(1) {
        RightMotorForward= RightMotorSpeed;
        LeftMotorForward=LeftMotorSpeed;
        FrontLeftIRTransmitter = 1;
        BackLeftIRTransmitter=1;
        FrontRightIRTransmitter=1;
        BackRightIRTransmitter=1;
        FrontLeftIRvalue= FrontLeftIRReceiver.read();
        FrontRightIRvalue= FrontRightIRReceiver.read();
        BackLeftIRvalue= BackLeftIRReceiver.read();
        BackRightIRvalue= BackRightIRReceiver.read();
        FrontLeftIRTransmitter = 0;
        BackLeftIRTransmitter=0;
        FrontRightIRTransmitter=0;
        BackRightIRTransmitter=0;
        
        //printf("Leftfront = %f\n", ( FrontLeftIRvalue));
        //printf("Leftback = %f\n", (BackLeftIRvalue));
        //printf("Rightfront = %f\n", (  FrontRightIRvalue));
        //printf("Rightback = %f\n", (BackRightIRvalue));
        
        PIDStraightEncoder();
        // wait(1);
        
        if(wallDETECTED())
        {
            // motorStop();
            //turn right? turn left?
            if(wallOnBothSides())
            {
                motorStop();
                //         printf("STOPPING!");
            }
            else if(wallOnRightSide())
            {
                //turn left
                turnLeft();
                //      printf("TURNINGLEFT!");
            }
            else if(wallOnLeftSide())
            {
                //turn right
                //     printf("WE IN TURN RIGHT");
                turnRight();
                //    printf("TURNED RIGHT!");
                //wait(5);
            }
        }
    }
}

void PIDStraightEncoder() //could just call it PIDStraight and have both PID having effect concurrently
{
    //double storeRightMotorForward=0;
    //double storeLeftMotorForward=0;
    
    int error=rightWheelEncoder.getPulses()-leftWheelEncoder.getPulses(); //positive error means rightEncoder has more pulses so right wheel is faster.
    
    // storeRightMotorForward=0.5*(RightMotorSpeed -(pController(error)+dController(error)+iController(error)));
    //storeLeftMotorForward=0.5*(LeftMotorSpeed+pController(error)+dController(error)+iController(error));
    //error seems to oscillate from 0.04->0.7ish and 0.04->-0.6ish
    //actually we only sneed back right - back left... why am i adding front!!!
    //if positive, too close to the right, I want the right IR value to be adjusted by going
    
    
    
    double errorIR=((BackRightIRvalue)-(BackLeftIRvalue));//   -(idealRight-idealLeft)   ; //0.4 is the early offset //positive error means rightEncoder has more pulses so right wheel is faster.
    //storeRightMotorForward= storeRightMotorForward+ 1*((pControllerIR(errorIR)+dControllerIR(errorIR)+iController(errorIR))); //may need to make sepearate versions for PIDStraightIR version of how motor affected based on IR READINGS
    //storeLeftMotorForward=storeLeftMotorForward- 1*((pControllerIR(errorIR)+dControllerIR(errorIR)+iController(errorIR)));
    //dont accumulate.
    //so keep base motor speed, but ok so motorfroward
    //RightMotorSpeed=storeRightMotorForward;
    //LeftMotorSpeed=storeLeftMotorForward;
    //maybe have a value that scales it down to a value from prev + or - max 0.02-0.03?
    ///PROBLEM IS its changing the speeds too fast.
    
    
    
    
    //  double rmfv=RightMotorSpeed+storeRightMotorForward;
    //  double lmfv=LeftMotorSpeed+storeLeftMotorForward;
    
    RightMotorForward=RightMotorSpeed + pControllerIR(errorIR)+dControllerIR(errorIR);
    LeftMotorForward=LeftMotorSpeed - pControllerIR(errorIR)-dControllerIR(errorIR);
    
    
    //RightMotorSpeed
    /*
     printf("RIGHT MOTOR FORWARD bs  IS ");
     printf("%f \n",rmfv);
     printf("LefT MOTOR FORWARD bs IS ");
     printf("%f \n",lmfv);
     
     printf("RIGHT MOTOR FORWARD add on FROM IR IS ");
     printf("%f \n",storeRightMotorForward);
     printf("Left MOTOR FORWARD add on FROM IR IS ");
     printf("%f \n",storeLeftMotorForward);
     printf("%f \n",errorIR);
     */
    return;
    
}
double pControllerIR(double error)
{
    //error should be defined as the difference vs how far the difference of ideal is.
    double correction=KpIR*error;
    // printf("Pcontroller value is %f\n", correction);
    return correction;
}
double dControllerIR(double error)
{
    //errorValueIr=?
    double changeError=error-errorValueIR;
    double dt=timer.read_ms();
    timer.reset();
    errorValueIR=error;
    // printf("Dcontroller changeError is %f\n", changeError);
    double correction=KdIR*changeError;
    //   printf("Dcontroller value is %f\n", correction);
    
    return correction;
}

double pController(int error)
{
    double correction=Kp*error;
    // printf("Pcontroller value is %f\n", correction);
    return correction;
}

double dController(int error)
{
    int changeError=error-errorValue;
    double dt=timer.read_us();
    timer.reset();
    errorValue=error;
    double correction=Kd*changeError/dt;
    // printf("Dcontroller value is %f\n", correction);
    
    return correction;
}
double iController(int error)
{
    integral+=error;
    double correction=Ki*integral;
    correction/=df;
    // printf("Icontroller value is %f\n", correction);
    
    return correction;
}
void turnLeft()
{
    //turn based on encoders and sensors
    RightMotorForward=0;
    LeftMotorForward=0;
    
    RightMotorForward=AdjustedRightMotorSpeedturnLeft;//some slower value
    LeftMotorBackward=AdjustedLeftMotorSpeedturnLeft; //some faster value
    // timer.reset();
    wait(0.1);
    RightMotorForward=0;
    LeftMotorBackward=0;
}
void turnRight()
{ //turn based on encoders and sensors
    RightMotorForward=0;
    LeftMotorForward=0;
    RightMotorBackward=AdjustedRightMotorSpeedturnRight;//some slower value
    LeftMotorForward=AdjustedLeftMotorSpeedturnRight; //some faster value
    //timer.reset();
    wait(0.2);
    RightMotorBackward=0;
    LeftMotorForward=0;
    //for a right turn, make right wheel slower, left wheel faster.
    // double rmf=AdjustedRightMotorSpeedturnRight;
    
    // printf("HI im turning right here are my motor right speed");
    // printf("%f \n",rmf);
    
    //   wait(5);
    
    //int numRightTurns=rightWheelEncoder.getPulses();
    //int numLeftTurns=leftWheelEncoder.getPulses();
    
    //adjustedspeeds are basically encoder ticks scaled somehow for a motor.
    //right
    //return;
}
/* BackLeftIRvalue are the closest values
 BackRightIRvalue
 */
bool wallOnBothSides()
{
    bool ans=false;
    if((BackLeftIRvalue>=LeftIRTHRESHOLD) &&(BackRightIRvalue>= RightIRTHRESHOLD))     //both IR values are high for theres a wall on both sides)
    {
        ans=true;
    }
    return ans;
}
bool wallOnRightSide()
{
    bool ans=false;
    if((BackRightIRvalue>= RightIRTHRESHOLD) &&wallDETECTED()  )           //) //if one IR values is low (the right front and right back are high showing stuff there)
    {
        ans=true;
    }
    return ans;
}
bool wallOnLeftSide()
{
    bool ans=false;
    if((BackLeftIRvalue>= LeftIRTHRESHOLD)&&wallDETECTED() )     //)//if IR value on the Left side of the car is high showing stuff
    {
        ans=true;
    }
    return ans;
}
void motorStop()
{
    RightMotorForward=0.0;
    LeftMotorForward=0.0;
    wait(10000);
}
bool wallDETECTED()
{
    bool ans=false;
    if(FrontLeftIRvalue>=(TOOCLOSE)||FrontRightIRvalue>=(TOOCLOSE))
    {
        ans =true;
    }
    return ans;
}





