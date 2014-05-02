//
//  FlightController.m
//  CMMotionTest
//
//  Created by junpeiwada on 2014/04/28.
//  Copyright (c) 2014年 soneru. All rights reserved.
//
#import <CoreMotion/CoreMotion.h>
#import "FlightController.h"

typedef struct {
    float P;
    float I;
    float D;
}PID_CONST;

#define PITCH 0
#define ROLL 1
#define YAW 2


@interface FlightController (){
    float motorPower[4]; // モーターの起動パワー
    float attiData[3]; // 現在の姿勢
    float targetAttiData[3]; //目標の姿勢
    float diff[4]; // 各軸と目標とのズレ
    
    float rotationRate[3]; // 回転の角速度
    
    
    // PID制御のテスト用の値
    float integral;
    float delta_T;
    float axisPID[3];
    PID_CONST pid[3];
}
@property (nonatomic) CMMotionManager *manager;




@end

@implementation FlightController


-(void)setup{
    PID_CONST val;
    val.P = 0.3;
    val.I = 0.001;
    val.D = 0.5;
    pid[0] = val;
    
    val.P = 0.3;
    val.I = 0.001;
    val.D = 0.5;
    pid[1] = val;
    
    val.P = 0.3;
    val.I = 0.001;
    val.D = 0.5;
    pid[2] = val;
    
    delta_T = 0.02;
    
    targetAttiData[PITCH] = 0;
    targetAttiData[ROLL] = 0;
    targetAttiData[YAW] = 0;
    
    self.throttle = 0.2;
    
    diff[0] = diff[1];
    diff[1] = targetAttiData[0] - attiData[0];
    
    // 更新間隔の指定
    self.manager = [[CMMotionManager alloc] init];
    self.manager.deviceMotionUpdateInterval = 1 / 100;  // 秒
    
    CMDeviceMotionHandler handler = ^(CMDeviceMotion *motion, NSError *error) {
        attiData[PITCH] = motion.attitude.pitch;
        attiData[ROLL] = motion.attitude.roll;
        attiData[YAW] = motion.attitude.yaw;
        
        rotationRate[PITCH] = motion.rotationRate.x;
        rotationRate[ROLL] = motion.rotationRate.y;
        rotationRate[YAW] = motion.rotationRate.z;
        
        [self calcMotorPower];
        
        NSNotification *notification = [NSNotification notificationWithName:@"ChangeAtti" object:self];
        [[NSNotificationCenter defaultCenter] postNotification:notification];
    };
    
    // 電子コンパスを使わない
    [self.manager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryCorrectedZVertical
                                                      toQueue:[NSOperationQueue currentQueue] withHandler:handler];
}


-(void)calcMotorPower{
    
    float Throttle = self.throttle;
    
    float rateEffect = 0.4;
    
    // このコードは水平を保とうとする
    if (attiData[ROLL] > 0){
        motorPower[0] = Throttle - (targetAttiData[ROLL] - attiData[ROLL] + rotationRate[ROLL]*rateEffect) * self.pFactor;
        motorPower[1] = Throttle - (targetAttiData[ROLL] - attiData[ROLL] + rotationRate[ROLL]*rateEffect) * self.pFactor;
        motorPower[2] = Throttle + (targetAttiData[ROLL] - attiData[ROLL] - rotationRate[ROLL]*rateEffect) * self.pFactor;
        motorPower[3] = Throttle + (targetAttiData[ROLL] - attiData[ROLL] - rotationRate[ROLL]*rateEffect) * self.pFactor;
    }else{
        motorPower[0] = Throttle - fabs((targetAttiData[ROLL] - attiData[ROLL] + rotationRate[ROLL]*rateEffect)) * self.pFactor;
        motorPower[1] = Throttle - fabs((targetAttiData[ROLL] - attiData[ROLL] + rotationRate[ROLL]*rateEffect)) * self.pFactor;
        motorPower[2] = Throttle + fabs((targetAttiData[ROLL] - attiData[ROLL] - rotationRate[ROLL]*rateEffect)) * self.pFactor;
        motorPower[3] = Throttle + fabs((targetAttiData[ROLL] - attiData[ROLL] - rotationRate[ROLL]*rateEffect)) * self.pFactor;
    }
    
    if (attiData[PITCH] > 0){
        motorPower[0] = (motorPower[0] - attiData[PITCH] + rotationRate[PITCH]*rateEffect) * self.pFactor;
        motorPower[1] = (motorPower[1] + attiData[PITCH] + rotationRate[PITCH]*rateEffect) * self.pFactor;
        motorPower[2] = (motorPower[2] + attiData[PITCH] + rotationRate[PITCH]*rateEffect) * self.pFactor;
        motorPower[3] = (motorPower[3] - attiData[PITCH] + rotationRate[PITCH]*rateEffect) * self.pFactor;
    }else{
        motorPower[0] = (motorPower[0] + fabs(attiData[PITCH] + rotationRate[PITCH]*rateEffect)) * self.pFactor;
        motorPower[1] = (motorPower[1] - fabs(attiData[PITCH] + rotationRate[PITCH]*rateEffect)) * self.pFactor;
        motorPower[2] = (motorPower[2] - fabs(attiData[PITCH] + rotationRate[PITCH]*rateEffect)) * self.pFactor;
        motorPower[3] = (motorPower[3] + fabs(attiData[PITCH] + rotationRate[PITCH]*rateEffect)) * self.pFactor;
    }
    
    
}
-(float)currentMotorPower:(int)index{
    
    if (index > 4){
        return 1;
    }
    
    int MaxMotor = 50;
    
    int value = motorPower[index] * MaxMotor;
    if (value >= MaxMotor){
        value = MaxMotor;
    }
    if (value <= 0){
        value = 1;
    }
    
    return value;
}

// 目標の姿勢を設定
-(void)setTargetAttiWithPitch:(float)pitch roll:(float)roll yaw:(float)yaw{
    targetAttiData[PITCH] = pitch;
    targetAttiData[ROLL] = roll;
    targetAttiData[YAW] = yaw;
}

-(void)sim{
    [self PID];
    attiData[0] = attiData[0] + axisPID[0] * 0.03;
}

-(void)setPID:(float)vP valueI:(float)vI valueD:(float)vD{
    PID_CONST val;
    val.P = vP;
    val.I = vI;
    val.D = vD;
    pid[0] = val;
    
    attiData[PITCH] = 0;
    attiData[ROLL] = 0;
    attiData[YAW] = 0;
    
    diff[0] = diff[1];
    diff[1] = targetAttiData[0] - attiData[0];
    
    integral = 0;
}

-(float)currentAtti{
    return attiData[0];
}

-(void)PID{
    for(int axis=0;axis<1;axis++) {
        diff[0] = diff[1];
        diff[1] = targetAttiData[axis] - attiData[axis];
        
        integral += (diff[1] + diff[0] / 2 * delta_T);
        
        float PTerm,ITerm,DTerm;
        PTerm = diff[1]  * pid[axis].P;
        ITerm = integral * pid[axis].I;
        DTerm = (diff[1] - diff[0]) / delta_T * pid[axis].D;
        
        axisPID[axis] =  PTerm + ITerm - DTerm;
    }
}


@end
