//
//  FlightController.h
//  CMMotionTest
//
//  Created by junpeiwada on 2014/04/28.
//  Copyright (c) 2014年 soneru. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface FlightController : NSObject{
    
}
-(void)setup;
-(void)sim;

-(float)currentAtti;
-(void)setPID:(float)vP valueI:(float)vI valueD:(float)vD;
-(void)setTargetAttiWithPitch:(float)pitch roll:(float)roll yaw:(float)yaw;
-(float)currentMotorPower:(int)index;

@property (nonatomic) float pFactor;

@end
