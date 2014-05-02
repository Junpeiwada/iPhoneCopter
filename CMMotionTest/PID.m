//
//  PID.m
//  CMMotionTest
//
//  Created by junpeiwada on 2014/05/02.
//  Copyright (c) 2014年 soneru. All rights reserved.
//

#import "PID.h"

@implementation PID{
    float integral;
    float cur,old;
    float kp,ki,kd;
    float differential_limit;
    float pid_limit;
}

-(void)initParameters{
//    interval = 0.05;
    differential_limit = 200;
    pid_limit = 300;
    kp = 0.5;
    ki = 0.5;
    kd = 0.2;
    integral = 0;
    old = 0;
};
-(void)setupParamWithKP:(float)KP
                     KI:(float)KI
                     KD:(float)KD
                PID_LIM:(float)PID_LIM
            INTGRAL_LIM:(float)DIFFERENTIAL_LIM{
    
    pid_limit = PID_LIM;
    differential_limit = DIFFERENTIAL_LIM;
    kp = KP;
    ki = KI;
    kd = KD;
    integral = 0;
    old = 0;
}
-(float)calc:(float)value target:(float)target interval:(float)interval{
    float differential;
    cur = value - target ;
    integral += (( cur + old ) * interval) / 2.0f;
    
    differential = ( kd * ( cur - old ) / interval );
    if ( differential < -differential_limit ) differential = -differential_limit;
    if ( differential > differential_limit ) differential = differential_limit;
    
    float pid = ( kp * cur ) + ( ki * integral ) + differential;
    if ( pid < -pid_limit ) {pid = -pid_limit;}
    if ( pid > pid_limit ) {pid = pid_limit;}
    
    old = cur;
    return pid;
}
-(void)reset{
    integral = 0;
    old = 0;
}

@end
