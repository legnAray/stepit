#include <stepit/robot/unitree.h>

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <string>

#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef STEPIT_UNITREE_ROBOT
#error "STEPIT_UNITREE_ROBOT not defined."
#endif  // STEPIT_UNITREE_ROBOT

#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
namespace UNITREE_LEGGED_SDK {
uint32_t crc32(uint32_t *ptr, uint32_t len);
}
#endif

namespace stepit {
#if STEPIT_UNITREE_ROBOT != UNITREE_ALIENGO
static std::unique_ptr<unitree::UDP> makeUdp() {
  std::string target_ip{"192.168.123.10"};
  long target_port{8007};
  long local_port{8090};
  getenv("STEPIT_UNITREE_UDP_IP", target_ip);
  getenv("STEPIT_UNITREE_UDP_SERVER_PORT", target_port);
  getenv("STEPIT_UNITREE_UDP_CLIENT_PORT", local_port);
  return std::make_unique<unitree::UDP>(unitree::LOWLEVEL,
                                        static_cast<uint16_t>(local_port),
                                        target_ip.c_str(),
                                        static_cast<uint16_t>(target_port));
}
#endif

#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
static bool envEnabled(const char *name) {
  const char *val = std::getenv(name);
  if (!val) return false;
  std::string s(val);
  for (auto &ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return (s == "1" || s == "true" || s == "yes" || s == "on");
}

class UnitreeApi::RawUdp {
 public:
  RawUdp() {
    std::string target_ip{unitree::UDP_SERVER_IP_BASIC};
    long target_port{unitree::UDP_SERVER_PORT};
    long local_port{unitree::UDP_CLIENT_PORT};
    getenv("STEPIT_UNITREE_UDP_IP", target_ip);
    getenv("STEPIT_UNITREE_UDP_SERVER_PORT", target_port);
    getenv("STEPIT_UNITREE_UDP_CLIENT_PORT", local_port);

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) return;

    int flags = ::fcntl(sock_fd_, F_GETFL, 0);
    ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(static_cast<uint16_t>(local_port));
    local_addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      return;
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(static_cast<uint16_t>(target_port));
    ::inet_pton(AF_INET, target_ip.c_str(), &target_addr_.sin_addr);
  }

  ~RawUdp() {
    if (sock_fd_ >= 0) ::close(sock_fd_);
  }

  void InitCmdData(unitree::LowCmd &cmd) {
    std::memset(&cmd, 0, sizeof(cmd));
    cmd.levelFlag = unitree::LOWLEVEL;
    for (auto &motor_cmd : cmd.motorCmd) {
      motor_cmd.mode = 0x0A;
      motor_cmd.q    = unitree::PosStopF;
      motor_cmd.dq   = unitree::VelStopF;
      motor_cmd.Kp   = 0.0f;
      motor_cmd.Kd   = 0.0f;
      motor_cmd.tau  = 0.0f;
    }
    cmd_buf_ = cmd;
  }

  int SetSend(unitree::LowCmd &cmd) {
    cmd_buf_ = cmd;
    return 0;
  }

  void GetRecv(unitree::LowState &state) { state = state_buf_; }

  int Send() {
    if (sock_fd_ < 0) return -1;
    cmd_buf_.crc = ::UNITREE_LEGGED_SDK::crc32(reinterpret_cast<uint32_t *>(&cmd_buf_),
                                               (sizeof(cmd_buf_) >> 2) - 1);
    ::sendto(sock_fd_, &cmd_buf_, sizeof(cmd_buf_), 0,
             reinterpret_cast<const sockaddr *>(&target_addr_), sizeof(target_addr_));
    return 0;
  }

  int Recv() {
    if (sock_fd_ < 0) return -1;
    sockaddr_in src{};
    socklen_t len = sizeof(src);
    const ssize_t n = ::recvfrom(sock_fd_, &state_buf_, sizeof(state_buf_), MSG_DONTWAIT,
                                 reinterpret_cast<sockaddr *>(&src), &len);
    if (n == static_cast<ssize_t>(sizeof(state_buf_))) {
      return 0;
    }
    return -1;
  }

 private:
  int sock_fd_{-1};
  sockaddr_in target_addr_{};
  unitree::LowCmd cmd_buf_{};
  unitree::LowState state_buf_{};
};
#endif

UnitreeApi::UnitreeApi()
    : RobotApi(getRobotName())
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
    , udp_(nullptr) {
#else
    , udp_(makeUdp()) {
#endif
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  use_raw_udp_ = envEnabled("STEPIT_ALIENGO_RAW_UDP");
  if (use_raw_udp_) {
    raw_udp_ = std::make_unique<RawUdp>();
    raw_udp_->InitCmdData(low_cmd_);
  } else {
    udp_ = std::make_unique<unitree::UDP>(unitree::LOWLEVEL);
    udp_->InitCmdData(low_cmd_);
  }
#else
  if (udp_) udp_->InitCmdData(low_cmd_);
#endif
  low_cmd_.levelFlag = unitree::LOWLEVEL;
  for (auto &motor_cmd : low_cmd_.motorCmd) {
    motor_cmd.mode = 0x0A;  // motor switch to servo (PMSM) mode
    motor_cmd.q    = unitree::PosStopF;
    motor_cmd.dq   = unitree::VelStopF;
    motor_cmd.Kp   = 0.;
    motor_cmd.Kd   = 0.;
    motor_cmd.tau  = 0.;
  }
}

UnitreeApi::~UnitreeApi() = default;

void UnitreeApi::getControl(bool enable) {
  if (!enable) return;
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  if (use_raw_udp_) {
    raw_udp_->SetSend(low_cmd_);
    raw_udp_->Send();
    return;
  }
#endif
  if (udp_) {
    udp_->SetSend(low_cmd_);
    udp_->Send();
  }
}

void UnitreeApi::setSend(LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_cmd_.motorCmd[i].q   = cmd_msg[i].q;
    low_cmd_.motorCmd[i].dq  = cmd_msg[i].dq;
    low_cmd_.motorCmd[i].Kp  = cmd_msg[i].Kp;
    low_cmd_.motorCmd[i].Kd  = cmd_msg[i].Kd;
    low_cmd_.motorCmd[i].tau = cmd_msg[i].tor;
  }
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  if (use_raw_udp_) {
    raw_udp_->SetSend(low_cmd_);
    return;
  }
#endif
  if (udp_) udp_->SetSend(low_cmd_);
}

void UnitreeApi::getRecv(LowState &state_msg) {
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  if (use_raw_udp_) {
    raw_udp_->GetRecv(low_state_);
  } else if (udp_) {
    udp_->GetRecv(low_state_);
  }
#else
  if (udp_) udp_->GetRecv(low_state_);
#endif
  state_msg.tick              = low_state_.tick;
  state_msg.imu.quaternion    = low_state_.imu.quaternion;
  state_msg.imu.gyroscope     = low_state_.imu.gyroscope;
  state_msg.imu.accelerometer = low_state_.imu.accelerometer;
  state_msg.imu.rpy           = low_state_.imu.rpy;

  for (std::size_t i{}; i < getDoF(); ++i) {
    state_msg.motor_state[i].q   = low_state_.motorState[i].q;
    state_msg.motor_state[i].dq  = low_state_.motorState[i].dq;
    state_msg.motor_state[i].tor = low_state_.motorState[i].tauEst;
  }
  for (std::size_t i{}; i < getNumLegs(); ++i) {
    state_msg.foot_force[i] = low_state_.footForce[i];
  }
}

void UnitreeApi::send() {
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  if (use_raw_udp_) {
    raw_udp_->Send();
    return;
  }
#endif
  if (udp_) udp_->Send();
}

void UnitreeApi::recv() {
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  if (use_raw_udp_) {
    raw_udp_->Recv();
    return;
  }
#endif
  if (udp_) udp_->Recv();
}

constexpr const char *UnitreeApi::getRobotName() {
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  return "aliengo";
#elif STEPIT_UNITREE_ROBOT == UNITREE_GO1
  return "go1";
#elif STEPIT_UNITREE_ROBOT == UNITREE_B1
  return "b1";
#else
  return "";
#endif
}

#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
STEPIT_REGISTER_ROBOTAPI(aliengo, kDefPriority, RobotApi::make<UnitreeApi>);
#elif STEPIT_UNITREE_ROBOT == UNITREE_GO1
STEPIT_REGISTER_ROBOTAPI(go1, kDefPriority, RobotApi::make<UnitreeApi>);
#elif STEPIT_UNITREE_ROBOT == UNITREE_B1
STEPIT_REGISTER_ROBOTAPI(b1, kDefPriority, RobotApi::make<UnitreeApi>);
#else
#error "Unsupported robot type."
#endif  // STEPIT_UNITREE_ROBOT
}  // namespace stepit
