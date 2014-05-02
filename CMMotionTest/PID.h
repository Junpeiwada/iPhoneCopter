//
//  PID.h
//  CMMotionTest
//
//  Created by junpeiwada on 2014/05/02.
//  Copyright (c) 2014年 soneru. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface PID : NSObject
-(void)initParameters;
-(void)setupParamWithKP:(float)KP
                     KI:(float)KI
                     KD:(float)KD
                PID_LIM:(float)PID_LIM
            INTGRAL_LIM:(float)DIFFERENTIAL_LIM;
-(float)calc:(float)value target:(float)target interval:(float)interval;
-(void)reset;
@end
