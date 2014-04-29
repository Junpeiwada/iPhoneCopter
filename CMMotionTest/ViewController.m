//
//  ViewController.m
//  CMMotionTest
//
//  Created by junpeiwada on 2014/04/26.
//  Copyright (c) 2014年 soneru. All rights reserved.
//

#import "ViewController.h"
#import <CoreMotion/CoreMotion.h>
#import "GraphView.h"



@interface ViewController (){



}
@property (weak, nonatomic) IBOutlet GraphView *motionGraph1;
@property (weak, nonatomic) IBOutlet GraphView *motionGraph2;
@property (weak, nonatomic) IBOutlet GraphView *motionGraph3;
@property (weak, nonatomic) IBOutlet GraphView *motionGraph4;
@property (weak, nonatomic) IBOutlet GraphView *motionGraph5;

@property (weak, nonatomic) IBOutlet GraphView *motionGraph6;
@property (weak, nonatomic) IBOutlet UILabel *deviceMotionMagnetometerCalibrationAccuracyLabel;
@property (weak, nonatomic) IBOutlet UILabel *magLabel;

@property (weak, nonatomic) IBOutlet UIScrollView *scrollView;

@property (nonatomic) CMMotionManager *manager;
@property (weak, nonatomic) IBOutlet UIImageView *pitchImage;
@property (weak, nonatomic) IBOutlet UIImageView *rollImage;
@property (weak, nonatomic) IBOutlet UIImageView *yawImage;
@end


@implementation ViewController

+ (CGFloat) degreesToRadians:(CGFloat) degrees {return degrees * M_PI / 180;};
+ (CGFloat) radiansToDegrees:(CGFloat) radians {return radians * 180/M_PI;};

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.scrollView.contentSize = CGSizeMake(320, 1000);
    
    // インスタンスの生成
    self.manager = [[CMMotionManager alloc] init];
    
    
    // 更新間隔の指定
    self.manager.deviceMotionUpdateInterval = 1 / 100;  // 秒
    
    // ハンドラ
    CMDeviceMotionHandler handler = ^(CMDeviceMotion *motion, NSError *error) {
        //double timestamp = motion.timestamp;
        
        /* accelerometer */
        [self.motionGraph1 addX:motion.gravity.x y:motion.gravity.y z:motion.gravity.z];
        [self.motionGraph2 addX:motion.userAcceleration.x y:motion.userAcceleration.y z:motion.userAcceleration.z];
        [self.motionGraph3 addX:motion.rotationRate.x y:motion.rotationRate.y z:motion.rotationRate.z];
        [self.motionGraph4 addX:[[self class] degreesToRadians:motion.magneticField.field.x]
                              y:[[self class] degreesToRadians:motion.magneticField.field.y]
                              z:[[self class] degreesToRadians:motion.magneticField.field.z]];
        
        [self.motionGraph5 addX:motion.attitude.roll
                              y:motion.attitude.pitch
                              z:motion.attitude.yaw];
        
        
        
        
        self.rollImage.transform = CGAffineTransformMakeRotation(motion.attitude.roll);
        self.pitchImage.transform = CGAffineTransformMakeRotation(-motion.attitude.pitch);
        self.yawImage.transform = CGAffineTransformMakeRotation(motion.attitude.yaw);
        
        
        switch (motion.magneticField.accuracy) {
            case CMMagneticFieldCalibrationAccuracyUncalibrated:
                self.deviceMotionMagnetometerCalibrationAccuracyLabel.text = @"Uncalibrated";
                break;
            case CMMagneticFieldCalibrationAccuracyLow:
                self.deviceMotionMagnetometerCalibrationAccuracyLabel.text = @"Low";
                break;
            case CMMagneticFieldCalibrationAccuracyMedium:
                self.deviceMotionMagnetometerCalibrationAccuracyLabel.text = @"Medium";
                break;
            case CMMagneticFieldCalibrationAccuracyHigh:
                self.deviceMotionMagnetometerCalibrationAccuracyLabel.text = @"High";
                break;
        }
        
        
        
        // コンパスの向きの対応と、現在の向き、ジャイロの向きをいい感じに揃える実装が必要
        
        self.magLabel.text = [NSString stringWithFormat:@"atti:%.1lf ",motion.attitude.yaw];
        
    };
    
    // 電子コンパスを使わない
    [self.manager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryCorrectedZVertical
                                                      toQueue:[NSOperationQueue currentQueue] withHandler:handler];
    
    // 電子コンパスとGPSを使って、真北を探す
//    [self.manager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXTrueNorthZVertical
//                                                      toQueue:[NSOperationQueue currentQueue] withHandler:handler];
    
    // 電子コンパスを使って補正する
//    [self.manager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
//                                                      toQueue:[NSOperationQueue currentQueue] withHandler:handler];
    
    

    
}


#pragma mark Member

//
//- (CGFloat) rawHeadingFromX:(CGFloat)xVec Y:(CGFloat)yVec
//{
//    
//    /*
//     to obtain this X and Y we really need to use the original mag XYZ and do some kind of matrix multiplication with the rotation matrix for the device.
//     Here we are only using the original X and Y values, so this only works if the device is held flat.
//     */
//    
//    //http://stackoverflow.com/questions/11383968/which-ios-class-code-returns-the-magnetic-north/11384054#11384054
//    //http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
//    
//    CGFloat rawHeading = 0;
//    if (yVec > 0) rawHeading = 90.0 - atan(xVec/yVec)*180.0/M_PI;
//    if (yVec < 0) rawHeading = 270.0 - atan(xVec/yVec)*180.0/M_PI;
//    if (yVec == 0 && xVec < 0) rawHeading = 180.0;
//    if (yVec == 0 && xVec > 0) rawHeading = 0.0;
//    rawHeading -=90;
//    return rawHeading;
//}



- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
