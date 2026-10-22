#include <yaml-cpp/yaml.h>
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <thread>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

#include "gamepad.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

// Audio
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

// Loco
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

// static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_CMD_TOPIC = "rt/arm_sdk";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
  std::unique_lock<std::shared_mutex> lock(mutex);
  data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
  std::shared_lock<std::shared_mutex> lock(mutex);
  return data ? data : nullptr;
  }

  void Clear() {
  std::unique_lock<std::shared_mutex> lock(mutex);
  data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

const int G1_NUM_MOTOR = 29;
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// Stiffness for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kp{
  60, 60, 60, 100, 40, 40,      // legs
  60, 60, 60, 100, 40, 40,      // legs
  60, 40, 40,                   // waist
  40, 40, 40, 40,  40, 40, 40,  // arms
  40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kd{
  1, 1, 1, 2, 1, 1,     // legs
  1, 1, 1, 2, 1, 1,     // legs
  1, 1, 1,              // waist
  1, 1, 1, 1, 1, 1, 1,  // arms
  1, 1, 1, 1, 1, 1, 1   // arms
};

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

struct joyInputState {
  std::atomic<bool> reset_to_zero{false};
  std::atomic<bool> emergency_stop{false};
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
  xbit = 1 << 31;
  data = ptr[i];
  for (uint32_t bits = 0; bits < 32; bits++) {
    if (CRC32 & 0x80000000) {
    CRC32 <<= 1;
    CRC32 ^= dwPolynomial;
    } else
    CRC32 <<= 1;
    if (data & xbit) CRC32 ^= dwPolynomial;

    xbit >>= 1;
  }
  }
  return CRC32;
};

std::unordered_map<char, std::string> jointMap({{'1', "Ankle"},
                        {'2', "Knee"},
                        {'3', "Hip"},
                        {'4', "Waist"},
                        {'5', "Wrist"},
                        {'6', "Elbow"},
                        {'7', "Shoulder"}});

std::unordered_map<char, std::string> sideMap({{'0', "Mid Body"},
                         {'1', "Left"},
                         {'2', "Right"}});

std::unordered_map<char, std::string> dirMap({{'0', "Just Move"},
                        {'1', "Roll"},
                        {'2', "Pitch"}, 
                        {'3', "Yaw"}});

std::unordered_map<std::string, int> motorMap({{"111", 5},
                        {"112", 4},
                        {"121", 11},
                        {"122", 10},
                        {"210", 3},
                        {"220", 9},
                        {"311", 1},
                        {"312", 0},
                        {"313", 2},
                        {"321", 7},
                        {"322", 6},
                        {"323", 8},
                        {"401", 13}, // invalid if waist locked
                        {"402", 14}, // invalid if waist locked
                        {"403", 12},
                        {"511", 19},
                        {"512", 20}, // invalid in 23 dof
                        {"513", 21}, // invalid in 23 dof
                        {"521", 26},
                        {"522", 27}, // invalid in 23 dof
                        {"523", 28}, // invalid in 23 dof
                        {"610", 18},
                        {"620", 25},
                        {"711", 16},
                        {"712", 15},
                        {"713", 17},
                        {"721", 23},
                        {"722", 22},
                        {"723", 24}});
std::unordered_map<int, float> maxQMap({{0, 2.85},
                    {1, 2.95},
                    {2, 2.75},
                    {3, 2.85},
                    {4, 0.50},
                    {5, 0.25},
                    {6, 2.85},
                    {7, 0.50},
                    {8, 2.75},
                    {9, 2.85},
                    {10, 0.50},
                    {11, 0.25},
                    {12, 2.60},
                    {13, 0.50},
                    {14, 0.50},
                    {15, 2.65},
                    {16, 2.25},
                    {17, 2.60},
                    {18, 2.05},
                    {19, 1.95},
                    {20, 1.60},
                    {21, 1.60},
                    {22, 2.65},
                    {23, 1.55},
                    {24, 2.60},
                    {25, 2.05},
                    {26, 1.95},
                    {27, 1.60},
                    {28, 1.60}});
std::unordered_map<int, float> minQMap({{0, -2.50},
                    {1, -0.50},
                    {2, -2.75},
                    {3, -0.05},
                    {4, -0.85},
                    {5, -0.25},
                    {6, -2.50},
                    {7, -2.95},
                    {8, -2.75},
                    {9, -0.05},
                    {10, -0.85},
                    {11, -0.25},
                    {12, -2.60},
                    {13, -0.50},
                    {14, -0.50},
                    {15, -3.05},
                    {16, -1.55},
                    {17, -2.60},
                    {18, -1.00},
                    {19, -1.95},
                    {20, -1.60},
                    {21, -1.60},
                    {22, -3.05},
                    {23, -2.25},
                    {24, -2.60},
                    {25, -1.00},
                    {26, -1.95},
                    {27, -1.60},
                    {28, -1.60}});

std::unordered_map<int, float> gainQMap({{0, 0.05},
                    {1, 0.1},
                    {2, 0.01},
                    {3, 0.5},
                    {4, 1.0},
                    {5, 1.0},
                    {6, 0.05},
                    {7, 0.1},
                    {8, 0.01},
                    {9, 0.5},
                    {10, 1.0},
                    {11, 1.0},
                    {12, 0.01},
                    {13, 1.0},
                    {14, 1.0},
                    {15, 0.3},
                    {16, 0.5},
                    {17, 0.3},
                    {18, 0.5},
                    {19, 0.5},
                    {20, 0.5},
                    {21, 0.5},
                    {22, 0.3},
                    {23, 0.5},
                    {24, 0.3},
                    {25, 0.5},
                    {26, 0.5},
                    {27, 0.5},
                    {28, 0.5}});
std::unordered_map<int, float> targetQMap;
std::unordered_map<int, float> resetTargetQMap;
std::unordered_map<int, float> poseTargetQMap;
std::unordered_map<int, float> targetQTolerance;
std::unordered_map<int, std::array<float,3>> motorControlSpecs; // multiplier, qDelta, duration

int motorIndex_ = -1; 
float rangeTime_ = 6.0;   // [3s]
float controlFreq_ = 500; // [500Hz]
float qDelta_ = 0.0;
bool excludeLowerBody_ = true;
int firstUpperBodyJoint_ = 15;





void get_input_motor_index(){
  std::string motor_i;
  
  // Motorized joints present
  std::map<char,bool> jointsPresent;
  for (const auto& m_to_i : motorMap) {
  jointsPresent[m_to_i.first[0]] = true;
  }

  // Input joint indicator
  char jointType;
  std::cout << "\n\n------------------------------------------------------------------\n";
  std::cout << "Enter 0 to RESET G1!" << std::endl;
  std::cout << "OR" << std::endl;
  std::cout << "Enter joint type:" << std::endl;
  for (const auto& i_to_p : jointsPresent){
  std::cout << i_to_p.first << ": " << jointMap[i_to_p.first] << std::endl;
  }
  std::cin >> jointType;
  if(jointType == '0'){
  std::cout << "RESETTING G1..." << std::endl;
  motorIndex_ = -1;
  return;
  }
  if(jointsPresent.find(jointType) == jointsPresent.end()) {
  std::cout << "BAD joint type!" << std::endl;
  return;
  }
  motor_i += jointType;

  // Motorized sides present
  std::map<char,bool> sidesPresent;
  for (const auto& m_to_i : motorMap) {
  if(m_to_i.first[0] != jointType){ continue; }
  sidesPresent[m_to_i.first[1]] = true;
  }
  
  // Input body side indicator
  char bodySide;
  std::cout << "Enter body side:" << std::endl;
  for (const auto& i_to_p : sidesPresent){
  std::cout << i_to_p.first << ": " << sideMap[i_to_p.first] << std::endl;
  }
  std::cin >> bodySide;
  if(sidesPresent.find(bodySide) == sidesPresent.end()) {
  std::cout << "BAD body side!" << std::endl;
  return;
  }
  motor_i += bodySide;

  // Motorized directions present
  std::map<char,bool> dirsPresent;
  for (const auto& m_to_i : motorMap) {
  if((m_to_i.first[0] != jointType) || (m_to_i.first[1] != bodySide)){ continue; }
  dirsPresent[m_to_i.first[2]] = true;
  }
  
  // Input direction indicator
  char rotDir;
  std::cout << "Enter rotation direction:" << std::endl;
  for (const auto& i_to_p : dirsPresent){
  std::cout << i_to_p.first << ": " << dirMap[i_to_p.first] << std::endl;
  }
  std::cin >> rotDir;
  if(dirsPresent.find(rotDir) == dirsPresent.end()) {
  std::cout << "BAD rotation direction!" << std::endl;
  return;
  }
  motor_i += rotDir;
  motorIndex_ = motorMap[motor_i];
  std::cout << "------------------------------------------------------------------\n";
  std::cout << sideMap[bodySide] << ' ' << jointMap[jointType] << ' ' << dirMap[rotDir] << "...\n";
  std::cout << "------------------------------------------------------------------\n";
}

void get_input_more_or_less(){
  if(motorIndex_ == -1){
  return;
  }
  char ml;
  std::cout << "more or less (m/l)? press o to change joint controlled." << std::endl;
  std::cin >> ml;
  if(ml == 'm'){
  qDelta_ = (maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
  std::cout << "Changing by " << qDelta_ << " rad per control call...\n";
  }
  else if(ml == 'l'){
  qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
  std::cout << "Changing by " << qDelta_ << " rad per control call...\n";
  }
  else if(ml == 'o'){
  qDelta_ = 0.0;
  motorIndex_ = -1;
  }
  else{
  return;
  }
}



class G1Example {
 private:
  double time_;
  double control_dt_;  // [2ms]
  double duration_;    // [3 s]
  int counter_;
  Mode mode_pr_;
  uint8_t mode_machine_;
  bool suppress_print_;
  bool suppress_sound_;
  bool ow_active_;

  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_, joyInput_thread_ptr_;

  // unitree::robot::g1::LocoClient loco_client_;
  // unitree::robot::g1::AudioClient sound_client_;

  int control_steps_;
  bool motor_sequence_active_;
  bool reset_sequence_active_;
  bool pose_sequence_active_;
  joyInputState joystick_state_;
  int joystick_open_;
  const char* joystick_device_;

 public:
  G1Example(std::string networkInterface)
    : time_(0.0),
    control_steps_(0),
    control_dt_(1.0/controlFreq_),
    duration_(rangeTime_),
    counter_(0),
    mode_pr_(Mode::PR),
    mode_machine_(0),
    suppress_print_(true),
    suppress_sound_(true),
    ow_active_(false),
    motor_sequence_active_(false),
    reset_sequence_active_(false),
    pose_sequence_active_(false) {
    

    ChannelFactory::Instance()->Init(0, networkInterface);

    // Mute robot
    unitree::robot::g1::AudioClient sound_client_;
    sound_client_.Init();
    sound_client_.SetTimeout(10.0f);
    int32_t ret = sound_client_.SetVolume(0);
    std::cout << "SetVolume to 0% , API ret:" << ret << std::endl;
    sleep(3);

    // Ready for Walk Mode
    unitree::robot::g1::LocoClient loco_client_;
    loco_client_.Init();
    loco_client_.SetTimeout(10.f);
    loco_client_.ZeroTorque();
    sleep(2);
    loco_client_.Damp();
    sleep(1);
    loco_client_.StandUp();
    sleep(5);
    loco_client_.SetFsmId(550510);
    sleep(5);
    loco_client_.SetFsmId(501);
    sleep(5);
    loco_client_.SetBalanceMode(0);
    sleep(3);

    // lumbar joints support / kp kd changes / squat position/ weight changes

    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // create subscriber
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1Example::imuTorsoHandler, this, std::placeholders::_1), 1);
    
    sleep(2);
    reset_latest_targets();
    sleep(2);
    reset_motor_controls();
    sleep(2);

    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Example::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, std::bind(&G1Example::Control, this, &loco_client_));
    joyInput_thread_ptr_ = CreateRecurrentThreadEx("joystick_reader", UT_CPU_ID_NONE, 2000, &G1Example::joyInputReader, this); 
    
    joystick_device_ = "/dev/input/by-id/usb-Sony_Computer_Entertainment_Wireless_Controller-event-joystick";
    joystick_open_ = open(joystick_device_, O_RDONLY);
    

    time_ = 0;
    reset_sequence_active_ = true;
    sleep(7);

    // // per-motor low level  conntrol through menu
    // while(motorIndex_ == -1){
    //   get_input_motor_index();
    //   if(motorIndex_ == -1){
    //     reset_latest_targets();
    //     time_ = 0;
    //     sleep(3);
    //   }
    //   while(motorIndex_ != -1){
    //     get_input_more_or_less();
    //     if (motorIndex_ != -1){
    //       const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    //       if(ms){ std::cout << ms->q.at(motorIndex_) << '\n'; }
    //     }
    //   }
    // }

    // while(true){
    //   if(ow_active_){
    //     std::cout << "Ouchie!\n";
    //     if(!suppress_sound_){
    //       ret = sound_client_.TtsMaker("ouchie!", 1);
    //     }
    //     ow_active_ = false;
    //   }
    //   sleep(2);
    // }

    
    // hi5("high-five", true);
    
    // time_ = 0;
    // reset_sequence_active_ = true;
    // sleep(7);

    // namaste("namaste", true);

    // time_ = 0;
    // reset_sequence_active_ = true;
    // sleep(7);

    readStateFromFile("high-five");
    pose_sequence_active_ = true;
    while(pose_sequence_active_){
      sleep(1);
    }
    sleep(5);

    readStateFromFile("namaste");
    pose_sequence_active_ = true;
    while(pose_sequence_active_){
      sleep(1);
    }
    sleep(5);

    time_ = 0;
    reset_sequence_active_ = true;
    sleep(7);

    while (true) sleep(10);

    close(joystick_open_);

  }

  void imuTorsoHandler(const void *message) {
  IMUState_ imu_torso = *(const IMUState_ *)message;
  auto &rpy = imu_torso.rpy();
  if (counter_ % (int)controlFreq_ == 0)
    if(!suppress_print_){
    printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
    }
  }

  void LowStateHandler(const void *message) {
  LowState_ low_state = *(const LowState_ *)message;
  if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
    std::cout << "[ERROR] CRC Error" << std::endl;
    return;
  }

  // get motor state
  MotorState ms_tmp;
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    ms_tmp.q.at(i) = low_state.motor_state()[i].q();
    ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
    if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
    std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
  }
  motor_state_buffer_.SetData(ms_tmp);

  // get imu state
  ImuState imu_tmp;
  imu_tmp.omega = low_state.imu_state().gyroscope();
  imu_tmp.rpy = low_state.imu_state().rpy();
  imu_state_buffer_.SetData(imu_tmp);

  // update gamepad
  memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
  gamepad_.update(rx_.RF_RX);

  // update mode machine
  if (mode_machine_ != low_state.mode_machine()) {
    if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
    mode_machine_ = low_state.mode_machine();
  }

  // report robot status every second
  if (++counter_ % (int)controlFreq_ == 0) {
    counter_ = 0;
    // IMU
    auto &rpy = low_state.imu_state().rpy();
    if(!suppress_print_){
    printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
    }

    // RC
    if(!suppress_print_){
    printf("gamepad_.A.pressed: %d\n", static_cast<int>(gamepad_.A.pressed));
    printf("gamepad_.B.pressed: %d\n", static_cast<int>(gamepad_.B.pressed));
    printf("gamepad_.X.pressed: %d\n", static_cast<int>(gamepad_.X.pressed));
    printf("gamepad_.Y.pressed: %d\n", static_cast<int>(gamepad_.Y.pressed));
    }

    // Motor
    auto &ms = low_state.motor_state();
    if(!suppress_print_)
    {
    printf("All %d Motors:", G1_NUM_MOTOR);
    printf("\nmode: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].mode());
    printf("\npos: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].q());
    printf("\nvel: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].dq());
    printf("\ntau_est: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].tau_est());
    printf("\ntemperature: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%d,%d;", ms[i].temperature()[0], ms[i].temperature()[1]);
    printf("\nvol: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].vol());
    printf("\nsensor: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u;", ms[i].sensor()[0], ms[i].sensor()[1]);
    printf("\nmotorstate: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].motorstate());
    printf("\nreserve: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u,%u,%u;", ms[i].reserve()[0], ms[i].reserve()[1], ms[i].reserve()[2], ms[i].reserve()[3]);
    printf("\n");
    }
  }
  }

  void LowCommandWriter() {
  LowCmd_ dds_low_command;
  dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
  dds_low_command.mode_machine() = mode_machine_;

  const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
  if (mc) {
    dds_low_command.motor_cmd().at(29).q(0.6); // for arm_sdk
    for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
    if(excludeLowerBody_ && i < firstUpperBodyJoint_){ continue; }
    dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
    dds_low_command.motor_cmd().at(i).tau(mc->tau_ff.at(i));
    dds_low_command.motor_cmd().at(i).q(mc->q_target.at(i));
    dds_low_command.motor_cmd().at(i).dq(mc->dq_target.at(i));
    dds_low_command.motor_cmd().at(i).kp(mc->kp.at(i));
    dds_low_command.motor_cmd().at(i).kd(mc->kd.at(i));
    }

    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    lowcmd_publisher_->Write(dds_low_command);
  }
  }

  void Control(unitree::robot::g1::LocoClient* loco_client) {
    if(joystick_state_.emergency_stop.load()){
      loco_client->Damp(); 
      return;
    }
    if(joystick_state_.reset_to_zero.load()){
      reset_sequence_active_ = true;
      motor_sequence_active_ = false;
      reset_motor_controls();
      joystick_state_.reset_to_zero.store(false);
    }

    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = targetQMap[i];
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }

    if (ms) {
      time_ += control_dt_;
      control_steps_ += 1;
      if (reset_sequence_active_) {
        // [Stage 1 / RESET]: set robot to zero posture
        bool motor_change_flag = false;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          if(excludeLowerBody_ && i < firstUpperBodyJoint_){ continue; }
          if(abs(resetTargetQMap[i] - ms->q.at(i)) > 0.02){
            int multiplier = (resetTargetQMap[i] < ms->q.at(i)) ? (-1) : 1;
            targetQMap[i] += multiplier*gainQMap[i]*motorControlSpecs[i][1];
            targetQMap[i] = std::clamp(targetQMap[i], minQMap[i], maxQMap[i]);
            targetQTolerance[i] = abs(ms->q.at(i) - targetQMap[i]);
            motor_command_tmp.q_target.at(i) = targetQMap[i];
            motor_change_flag = true;
          }
        }
        if(!motor_change_flag){ reset_sequence_active_ = false; }
      } else if(motor_sequence_active_){
        for(int i = 0; i < G1_NUM_MOTOR; i++){
          if(motorControlSpecs[i][2] > 0) {
            motorControlSpecs[i][2] -= control_dt_;
            targetQMap[i] += gainQMap[i]*motorControlSpecs[i][0]*motorControlSpecs[i][1];
            targetQMap[i] = std::clamp(targetQMap[i], minQMap[i], maxQMap[i]);
            targetQTolerance[i] = abs(ms->q.at(i) - targetQMap[i]);
            motor_command_tmp.q_target.at(i) = targetQMap[i];
          }
        }   
      }
      else if(pose_sequence_active_){ 
        bool motor_change_flag = false;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) { 
          if(excludeLowerBody_ && i < firstUpperBodyJoint_){ continue; }
          if(abs(poseTargetQMap[i] - ms->q.at(i)) > 0.02){
            int multiplier = (poseTargetQMap[i] < ms->q.at(i)) ? (-1) : 1;
            targetQMap[i] += multiplier*gainQMap[i]*motorControlSpecs[i][1];
            targetQMap[i] = std::clamp(targetQMap[i], minQMap[i], maxQMap[i]);
            targetQTolerance[i] = abs(ms->q.at(i) - targetQMap[i]);
            motor_command_tmp.q_target.at(i) = targetQMap[i];
            motor_change_flag = true;
            std::cout << i << ' ' << abs(poseTargetQMap[i] - ms->q.at(i)) << ' ' << targetQMap[i] << '\n'; 
          }
        }
        if(!motor_change_flag){ pose_sequence_active_ = false; }
      }
      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }

  void reset_motor_controls(){
    for(int i = 0; i < G1_NUM_MOTOR; i++){
      motorControlSpecs[i][0] = 0.0; 
      motorControlSpecs[i][1] = (maxQMap[i] - minQMap[i])/(rangeTime_ * controlFreq_);
      motorControlSpecs[i][2] = 0.0;
    }
    motor_sequence_active_ = false;
    return;
  }

  void reset_latest_targets(){
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      if(resetTargetQMap.find(i) == resetTargetQMap.end()){
      resetTargetQMap[i] = ms->q.at(i);
      }
      targetQMap[i] = resetTargetQMap[i];
    } 

  }

  void hi5(std::string name="high-five", bool write_to_file=false){
    reset_motor_controls();
    motorControlSpecs[22][0] = -1; motorControlSpecs[22][2] = 9;
    motorControlSpecs[23][0] = -1; motorControlSpecs[23][2] = 8;
    motorControlSpecs[25][0] = -1; motorControlSpecs[25][2] = 2;
    motorControlSpecs[28][0] = -1; motorControlSpecs[28][2] = 2;
    motor_sequence_active_ = true;
    sleep(20);
    if(write_to_file){ writeStateToFile(name); }
    sleep(5);
    reset_motor_controls();
    // motorIndex_ = 22;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(30);
    // motorIndex_ = 23;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(10);
    // motorIndex_ = 25;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(3);
    // motorIndex_ = 28;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(5);
    // time_ = 0;
    // motorIndex_ = -1;
    // sleep(7);
  }

  void namaste(std::string name="namaste", bool write_to_file=false){
    reset_motor_controls();
    motorControlSpecs[15][0] = -1; motorControlSpecs[15][2] = 7;
    motorControlSpecs[22][0] = -1; motorControlSpecs[22][2] = 7;
    

    motorControlSpecs[16][0] = 1; motorControlSpecs[16][2] = 3;
    motorControlSpecs[23][0] = -1; motorControlSpecs[23][2] = 3;
    
    motorControlSpecs[17][0] = -1; motorControlSpecs[17][2] = 7;
    motorControlSpecs[24][0] = 1; motorControlSpecs[24][2] = 7;

    motorControlSpecs[18][0] = -1; motorControlSpecs[18][2] = 8;
    motorControlSpecs[25][0] = -1; motorControlSpecs[25][2] = 8;
    
    motorControlSpecs[21][0] = 1; motorControlSpecs[21][2] = 7;
    motorControlSpecs[28][0] = -1; motorControlSpecs[28][2] = 7;

    motor_sequence_active_ = true;
    sleep(20);
    if(write_to_file){ writeStateToFile(name); }
    sleep(5);
    reset_motor_controls();
    // motorIndex_ = 22;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(30);
    // motorIndex_ = 23;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(10);
    // motorIndex_ = 25;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(3);
    // motorIndex_ = 28;
    // qDelta_ = -(maxQMap[motorIndex_] - minQMap[motorIndex_])/(rangeTime_ * controlFreq_);
    // sleep(5);
    // time_ = 0;
    // motorIndex_ = -1;
    // sleep(7);
  }

  void writeStateToFile(std::string name){
    YAML::Node pose;
    std::string outfile_name = POSE_LIB_DIR + name + ".yaml";
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    for(int i = 0; i < G1_NUM_MOTOR; i++){
      if(excludeLowerBody_ && i < firstUpperBodyJoint_){ continue; }
      pose["pose"][i] = ms->q.at(i);
    }
    if (!std::filesystem::exists(POSE_LIB_DIR)) {
        std::filesystem::create_directories(POSE_LIB_DIR);
    }
    std::ofstream fout(outfile_name);
    fout << pose;
  }

  void readStateFromFile(std::string name){
    std::string outfile_name = POSE_LIB_DIR + name + ".yaml";
    YAML::Node pose = YAML::LoadFile(outfile_name);
    
    for(int i = 0; i < G1_NUM_MOTOR; i++){
      if(excludeLowerBody_ && i < firstUpperBodyJoint_){ 
        poseTargetQMap[i] = 0.0; 
        continue;
      }
      poseTargetQMap[i] = pose["pose"][i].as<float>();
    }
  }

  void joyInputReader() {
    if (joystick_open_ == -1) { return; }
    struct input_event ev;
    ssize_t bytesRead = read(joystick_open_, &ev, sizeof(struct input_event));
    // Ignore incomplete event packets
    if (bytesRead < (ssize_t)sizeof(struct input_event)) {
      return; 
    }
    // Filter for button presses
    if (ev.type == EV_KEY && ev.value == 1) {
      switch(ev.code) {
        case 304: // (A or BTN_SOUTH)
          joystick_state_.reset_to_zero.store(false);
          joystick_state_.emergency_stop.store(false);
          break;
        case 307: // (Y or BTN_NORTH)
          joystick_state_.reset_to_zero.store(true);
          break;
        case 308: // (X or BTN_WEST)
          joystick_state_.emergency_stop.store(true);
          break;
        default:
          joystick_state_.emergency_stop.store(true);
          break;
      }
    } 
  }


};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
  std::cout << "Usage: g1_joint_IO_example <network_interface>" << std::endl;
  exit(0);
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);
  return 0;
}
