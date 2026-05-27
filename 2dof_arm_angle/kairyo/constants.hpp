#pragma once


#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8     //=number of pressure sensor
#define LINK_CHANNEL_NUMBER1 8   //膨らませるリンクの番号(8~11が使用可能)
#define LINK_CHANNEL_NUMBER2 9
#define THE_NUMBER_OF_MARKERS 9

#define DA_Conversion_K 0.1     //Voltage = K * Pressure
#define AD_Conversion_K 25.2    //Pressure = K * Voltage + A
#define AD_Conversion_A -25.4
#define SAMPLING_FREQUENCY 200
#define GRAVITY_ACC 9.80665

#define MINIMUM_PRESSURE 0.0F   //アクチュエータの最小圧力[kPa]
#define MAXIMUM_PRESSURE 60.0F  //アクチュエータの最大圧力[kPa]

#define BASE_PRESSURE 30.0F     //基本圧力[kPa]
#define LINK_PRESSURE 4.0F      //リンク膨張用圧力

#define DEGREE_OF_FREEDOM 2     //ロボットアームの自由度

#define FILE_OUTPUT //ファイル出力するならコメントアウト外す
#define GRAPPING_WEIGHT 0.0  //手先に吊り下げる荷重[kg]


//アームの位置制御で追加(20260525)
#define START_ORIENTATION 0.0
#define TARGET_ORIENTATION 20.0




//gain param
extern double pressureKP[DA_CHANNEL_NUMBER];
extern double pressureKI[DA_CHANNEL_NUMBER];
extern double pressureKD[DA_CHANNEL_NUMBER];
extern double visual_P;
extern double visual_I;
extern double visual_D;
extern double basePressure;
extern double diffPressure;
extern double linkPressure;
extern double targetJointStiffness;

extern double link_angle_deg; //リンクの膨張角度[deg]
extern double link_angle;  //リンクの膨張角度[rad]
extern double link_length;  //リンクの長さ


extern double q_dot ;  //qの微分値
extern bool q_dot_flag; //q_dotで場合分けをするかのフラグ

extern bool joint_stiffness_flag; //trueなら関節剛性の補正あり

extern bool torque_compensate_flag;

//ROS time
extern unsigned int cycle;

//sensor
extern double voltageInput[AD_CHANNEL_NUMBER];
extern double voltageOutput[DA_CHANNEL_NUMBER];
extern double pressureCurrentFiltered[AD_CHANNEL_NUMBER]; //ローパスフィルタ後の圧力値

extern double cutoffFrequencyPressure;      //圧力ローパスフィルタのカットオフ周波数[Hz]
extern double pressureDeviation[DA_CHANNEL_NUMBER];//目標値とセンサ値の圧力の偏差[レギュレータ番号]
extern double pressureP[DA_CHANNEL_NUMBER];        //P項による圧力値[レギュレータ番号] 4,3しか使わない
extern double pressureI[DA_CHANNEL_NUMBER];        //I
extern double pressureD[DA_CHANNEL_NUMBER];        //D

extern double orientationTarget;       //目標関節角[rad]
extern double orientationTarget_deg;   //目標関節角[deg]  
extern double orientationCurrent_raw;  //ローパスフィルタなしでの関節角[rad]
extern double orientationCurrent;      //15Hzローパスフィルタでの関節角[rad]
extern double orientationCurrent2;     //10Hzローパスフィルタでの関節角[rad]
extern double orientationCurrent3;     //5Hzローパスフィルタでの関節角[rad]
extern double orientation_buf;         //前ステップの角度
extern double orientationYaw;          //台座部分の角度
extern double AngleFB_I_threshold ; //目標角度と現在の角度が閾値以上ならI項を入れない
extern double element[3];              //視覚フィードバック制御の各項の値
// double  c[5] = {-0.007298062, 0.06181, -0.181226594, 0.194224742, 0};   //圧力-トルク変換の特性係数
extern double c[5];   //20250813更新

extern float positionMarker[THE_NUMBER_OF_MARKERS][3];   //マーカーの座標
extern float positionArm[5][3];                                //アームについているマーカーの座標
extern float positionOrigin[3];                                //原点の座標（ベースの中心）
extern float positionTarget[3];                             //目標位置の座標


//関数の宣言
void LowPassFilterMarkers();
double FeedbackControl();
void convert_R(const geometry_msgs::msg::TransformStamped& pose, double R[3][3]);
void mul_R(double left[3][3],double right[3][3],double res[3][3]);
void Trans_R(double R[3][3],double res[3][3]);
void print_matrix(double matrix[3][3]);
void print_quartanion(const geometry_msgs::msg::TransformStamped& pose);
void VisualFeedbackControl();
void SetPositionTarget(float x, float y, float z);
void ForwardKinematics(float q1, float q2);
void DetermineImpactDirection();
void DetermineImpactDirection2();
void ImpactDrivenControl(ImpactState& state, int pressure_idx_pos, int pressure_idx_neg);
void PressureFeedbackControl();


void PressureFeedbackReset();
void target_inverse();
double convertPdf(double q , double torque);