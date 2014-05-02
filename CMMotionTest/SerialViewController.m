//
//  SerialViewController.m
//  CMMotionTest
//
//  Created by junpeiwada on 2014/04/28.
//  Copyright (c) 2014年 soneru. All rights reserved.
//

#import "SerialViewController.h"
#import "GraphView.h"
#import "PID.h"
#import "FlightController.h"

#define MODEM_STAT_ON_COLOR [UIColor colorWithRed:0.0/255.0 green:255.0/255.0 blue:0.0/255.0 alpha:1.0]
#define MODEM_STAT_OFF_COLOR [UIColor colorWithRed:157.0/255.0 green:157.0/255.0 blue:157.0/255.0 alpha:1.0]

typedef enum CableConnectState
{
	kCableNotConnected,
	kCableConnected,
	kCableRequiresPasscode
} CableConnectState;

@interface SerialViewController (){
    // シリアルポート廻り
    RscMgr *rscMgr;
    CableConnectState cableState;
    int rxCount;
	int txCount;
    int errCount;
    
	UInt8 txLoopBuff[4096];
    
    float PIDp,PIDi,PIDd;
    
    FlightController *cont;
    
    
    PID *pidController;
    
    NSMutableString *readBuff;
}
@property (weak, nonatomic) IBOutlet UILabel *labelCTS;
@property (weak, nonatomic) IBOutlet UILabel *labelDSR;
@property (weak, nonatomic) IBOutlet UILabel *labelCD;
@property (weak, nonatomic) IBOutlet UILabel *labelRI;
@property (weak, nonatomic) IBOutlet UILabel *labelStatus;
@property (weak, nonatomic) IBOutlet GraphView *graph;
@property (weak, nonatomic) IBOutlet UILabel *labelP;
@property (weak, nonatomic) IBOutlet UILabel *labelI;
@property (weak, nonatomic) IBOutlet UILabel *labelD;
@property (weak, nonatomic) IBOutlet UILabel *labelReadCount;

@property (weak, nonatomic) IBOutlet UILabel *labelFR;
@property (weak, nonatomic) IBOutlet UIView *viewFR;
@property (weak, nonatomic) IBOutlet UILabel *labelRR;
@property (weak, nonatomic) IBOutlet UIView *viewRR;
@property (weak, nonatomic) IBOutlet UILabel *labelRL;
@property (weak, nonatomic) IBOutlet UIView *viewRL;
@property (weak, nonatomic) IBOutlet UILabel *labelFL;
@property (weak, nonatomic) IBOutlet UIView *viewFL;
@property (weak, nonatomic) IBOutlet UIStepper *pFactorStepper;
@property (weak, nonatomic) IBOutlet UILabel *labelPFactor;
@end

@implementation SerialViewController
- (IBAction)changePfactor:(id)sender {
    cont.pFactor =self.pFactorStepper.value;
    self.labelPFactor.text =[NSString stringWithFormat:@"%f",cont.pFactor];
}

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    
    return self;
}

-(void)updateGraph{
    self.labelP.text = [NSString stringWithFormat:@"P %f",PIDp];
    self.labelI.text = [NSString stringWithFormat:@"I %f",PIDi];
    self.labelD.text = [NSString stringWithFormat:@"D %f",PIDd];
    

    
    for (int i = 0; i <100; i ++) {
        [self.graph addX:0 y:0 z:0];
    }
    
    [pidController reset];
    float position = 0.0;
    
    for (int i = 0; i <280 * 2; i +=2 ) {
        float sousaryou = 0.0;
        sousaryou = [pidController calc:position target:30 interval:0.2];
        
        position = position - sousaryou * 0.1;
        
        [self.graph addX:position /30 y:sousaryou /100 z:0];
        
        if (i > 100){
            NSLog(@"");
        }
    }
}


- (IBAction)changeP:(id)sender {
    UIStepper *v = sender;
    PIDp = v.value;
    
    [cont setPID:PIDp valueI:PIDi valueD:PIDd];
    [pidController setupParamWithKP:PIDp KI:PIDi KD:PIDd PID_LIM:300 INTGRAL_LIM:300];

    [self updateGraph];
}
- (IBAction)changeI:(id)sender {
    UIStepper *v = sender;
    PIDi = v.value;
    
    [cont setPID:PIDp valueI:PIDi valueD:PIDd];
    [pidController setupParamWithKP:PIDp KI:PIDi KD:PIDd PID_LIM:300 INTGRAL_LIM:300];

    [self updateGraph];
}
- (IBAction)changeD:(id)sender {
    UIStepper *v = sender;
    PIDd = v.value;
    
    [cont setPID:PIDp valueI:PIDi valueD:PIDd];
    [pidController setupParamWithKP:PIDp KI:PIDi KD:PIDd PID_LIM:300 INTGRAL_LIM:300];

    [self updateGraph];
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    // シリアルアクセサリ初期化
    cableState = kCableNotConnected;
    rscMgr = [[RscMgr alloc] init];
    [rscMgr setDelegate:self];
    
    cont = [[FlightController alloc]init];
    [cont setup];
    cont.pFactor =self.pFactorStepper.value;
    
    readBuff = [NSMutableString new];
    
    NSNotificationCenter *nc = [NSNotificationCenter defaultCenter];
    [nc addObserver:self selector:@selector(updateUI) name:@"ChangeAtti" object:nil];
    
    
    
    [self updateGraph];
    [self portStatusChanged];
    
    
    pidController = [PID new];
    [pidController initParameters];
    [pidController setupParamWithKP:PIDp KI:PIDi KD:PIDd PID_LIM:300 INTGRAL_LIM:300];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

/*
 #pragma mark - Navigation
 
 // In a storyboard-based application, you will often want to do a little preparation before navigation
 - (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
 {
 // Get the new view controller using [segue destinationViewController].
 // Pass the selected object to the new view controller.
 }
 */


#pragma mark RSC delegate

- (void) cableConnected:(NSString *)protocol{
    // 接続されたか、アプリケーションがバックグラウンドから復帰した
    cableState = kCableConnected;
    
    [rscMgr open];
    [rscMgr setBaud:57600];
}

- (void) cableDisconnected{
    // 接続が外されたか、アプリケーションがバックグラウンドへ行った。
    cableState = kCableNotConnected;
}

- (void) portStatusChanged{
    // シリアルポートのステータスが変更された
    // getModemStatusかgetPortStatusで今の状態を取得できる
    
    int modemStatus = [rscMgr getModemStatus];
    static serialPortStatus portStat;
    
    [self.labelCTS setTextColor:(modemStatus & MODEM_STAT_CTS) ? MODEM_STAT_ON_COLOR : MODEM_STAT_OFF_COLOR];
    [self.labelRI  setTextColor:(modemStatus & MODEM_STAT_RI)  ? MODEM_STAT_ON_COLOR : MODEM_STAT_OFF_COLOR];
    [self.labelDSR setTextColor:(modemStatus & MODEM_STAT_DSR) ? MODEM_STAT_ON_COLOR : MODEM_STAT_OFF_COLOR];
    [self.labelCD  setTextColor:(modemStatus & MODEM_STAT_DCD) ? MODEM_STAT_ON_COLOR : MODEM_STAT_OFF_COLOR];
    
    if (cableState == kCableConnected){
        self.labelStatus.text = @"Connected";
    }else if (cableState == kCableNotConnected){
        self.labelStatus.text = @"Not Connected";
    }
    
    [rscMgr getPortStatus:&portStat];
}





- (void) readBytesAvailable:(UInt32)length{
    // getDataFromBytesAvailable か getStringFromBytesAvailableで読み込めるバイト数
    int bytesRead;
    
    
	UInt8 rxLoopBuff[kRSC_SerialReadBufferSize];
    for (int i = 0; i < kRSC_SerialReadBufferSize; i++) {
        rxLoopBuff[i] = 0;
    }
    
    
	// Read the data out
	bytesRead = [rscMgr read:(rxLoopBuff) length:length];
	rxCount += bytesRead;
    
    NSString *s = [NSString stringWithUTF8String:(char *)rxLoopBuff];
    
    if (!s){
        return;
    }
    [readBuff appendString:s];
    
    NSArray *lines = [readBuff componentsSeparatedByString:@";"];
    
    
    if (lines.count > 1){
        for (int i = 0; i < lines.count -1; i++) {
            NSString *line = lines[i];
            NSLog(@"%@",line);
            
            NSArray *items  = [line componentsSeparatedByString:@":"];
            
            if (items.count == 9){
                int throttoleValue = [items[0] intValue];
                throttoleValue = throttoleValue - 1008;
                float value = (float)throttoleValue / 936;
                cont.throttle = 5 * value;
            }
        }
    }
    
    
    readBuff = [[lines lastObject] mutableCopy];
    
    
    dispatch_queue_t mainQueue = dispatch_get_main_queue();
    dispatch_async(mainQueue, ^{
        self.labelReadCount.text = [NSString stringWithFormat:@"count:%d",rxCount];
    });
    
    
    [self sendMotorStat];
}

bool sendData = YES;

-(void)sendMotorStat{
    UInt8 sendBuff[5];
    if (sendData){
        sendBuff[0] = [cont currentMotorPower:0];
        sendBuff[2] = [cont currentMotorPower:1];
        sendBuff[3] = [cont currentMotorPower:2];
        sendBuff[1] = [cont currentMotorPower:3];
        
        sendBuff[4] = 0; // 終端文字;
    }else{
        sendBuff[0] = 1;
        sendBuff[2] = 1;
        sendBuff[3] = 1;
        sendBuff[1] = 1;
        sendBuff[4] = 0; // 終端文字;
    }
    
    [rscMgr write:sendBuff length:5];
    
    [self updateUI];
}

-(void)updateUI{
    dispatch_async(dispatch_get_main_queue(), ^(void){
        CGRect rect;
        
        float unitWidth = 100;
        
        self.labelFR.text = [NSString stringWithFormat:@"%d",(UInt8)[cont currentMotorPower:0]];
        rect = self.viewFR.frame;
        rect.size.width = unitWidth * [cont currentMotorPower:0] / 30;
        self.viewFR.frame = rect;
        
        self.labelRR.text = [NSString stringWithFormat:@"%d",(UInt8)[cont currentMotorPower:1]];
        rect = self.viewRR.frame;
        rect.size.width = unitWidth * [cont currentMotorPower:1] / 30;
        self.viewRR.frame = rect;
        
        self.labelRL.text = [NSString stringWithFormat:@"%d",(UInt8)[cont currentMotorPower:2]];
        rect = self.viewRL.frame;
        rect.size.width = unitWidth * [cont currentMotorPower:2] / 30;
        self.viewRL.frame = rect;
        
        self.labelFL.text = [NSString stringWithFormat:@"%d",(UInt8)[cont currentMotorPower:3]];
        rect = self.viewFL.frame;
        rect.size.width = unitWidth * [cont currentMotorPower:3] / 30;
        self.viewFL.frame = rect;
    });
    
}

- (IBAction)sendCommand:(id)sender {
    sendData = !sendData;
}

@end
