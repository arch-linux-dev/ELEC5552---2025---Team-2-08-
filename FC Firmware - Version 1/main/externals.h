#ifndef GLOBALS_H
#define GLOBALS_H

extern volatile float RatePitch, RateRoll, RateYaw;
extern float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

extern float PAngleRoll; 
extern float PAnglePitch;
extern float IAngleRoll; 
extern float IAnglePitch;
extern float DAngleRoll; 
extern float DAnglePitch;

extern float PRateRoll;
extern float IRateRoll;
extern float DRateRoll;

extern float PRatePitch;
extern float IRatePitch;
extern float DRatePitch;
 
extern float PRateYaw;
extern float IRateYaw;
extern float DRateYaw;

extern volatile float PtermRoll;
extern volatile float ItermRoll;
extern volatile float DtermRoll;
extern volatile float PIDOutputRoll;
extern volatile float PtermPitch;
extern volatile float ItermPitch;
extern volatile float DtermPitch;
extern volatile float PIDOutputPitch;
extern volatile float PtermYaw;
extern volatile float ItermYaw;
extern volatile float DtermYaw;
extern volatile float PIDOutputYaw;
extern volatile float KalmanGainPitch;
extern volatile float KalmanGainRoll;

extern volatile int ReceiverValue[6]; 

extern volatile float AccX, AccY, AccZ;
extern volatile float AngleRoll, AnglePitch;

extern volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;


#endif