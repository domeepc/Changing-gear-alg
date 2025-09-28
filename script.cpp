
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

namespace Config {
const double MAXIMUM_UPSHIFT_RPM = 13500.0;
const double SAMPLE_TIME = 0.5;
const double ALPHA_RPM = 0.2;
const double ALPHA_DOT = 0.2;
const double ALPHA_DDOT = 0.15;
const double PREDICTIVE_TIME = 0.08;

// CAN Configuration
const char* CAN_INTERFACE = "vcan0";  // Default CAN interface
const canid_t RPM_CAN_ID = 0x123;     // CAN ID for RPM data (adjust as needed)
const int RPM_BYTE_OFFSET = 0;        // Byte offset in CAN message for RPM data
const double RPM_SCALE_FACTOR = 1.0;  // Scale factor to convert raw data to RPM

// Debug and parsing options
const bool ENABLE_CAN_DEBUG = true;  // Enable detailed CAN message debugging
const bool USE_32BIT_RPM =
    false;  // Set to true if RPM is 32-bit instead of 16-bit

}  // namespace Config

class CAN {
 private:
  double currentRPM;
  int socketFd;
  struct sockaddr_can addr;
  struct ifreq ifr;
  bool isConnected;

 public:
  CAN();
  ~CAN();
  bool initSocketCAN(const char* interface = Config::CAN_INTERFACE);
  void closeSocketCAN();
  void PlayUpshiftSound();

  // Getters
  double getCurrentRPM();

  // Setters
  void setCurrentRPM(double rpm) { currentRPM = rpm; }

  // Utility
  bool isCANConnected() const { return isConnected; }
};

void CAN::PlayUpshiftSound() {
  std::cout << "Playing upshift sound!" << std::endl;
}

CAN::CAN() {
  currentRPM = 0.0;
  socketFd = -1;
  isConnected = false;
  memset(&addr, 0, sizeof(addr));
  memset(&ifr, 0, sizeof(ifr));
}

CAN::~CAN() { closeSocketCAN(); }

bool CAN::initSocketCAN(const char* interface) {
  // Create socket
  socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socketFd < 0) {
    std::cerr << "Error creating CAN socket" << std::endl;
    return false;
  }

  // Set up interface
  strcpy(ifr.ifr_name, interface);
  if (ioctl(socketFd, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error getting interface index for " << interface << std::endl;
    close(socketFd);
    socketFd = -1;
    return false;
  }

  // Set up address
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // Bind socket to CAN interface
  if (bind(socketFd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "Error binding CAN socket to " << interface << std::endl;
    close(socketFd);
    socketFd = -1;
    return false;
  }

  // Set non-blocking mode (optional)
  int flags = fcntl(socketFd, F_GETFL, 0);
  fcntl(socketFd, F_SETFL, flags | O_NONBLOCK);

  isConnected = true;
  std::cout << "SocketCAN initialized successfully on "
            << interface << std::endl;
  return true;
}

void CAN::closeSocketCAN() {
  if (socketFd >= 0) {
    close(socketFd);
    socketFd = -1;
  }
  isConnected = false;
}

double CAN::getCurrentRPM() {
  if (!isConnected) {
    return currentRPM;
  }

  struct can_frame frame;
  ssize_t nbytes = read(socketFd, &frame, sizeof(struct can_frame));

  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // No data available (non-blocking mode)
      return currentRPM;  // Return cached value
    } else {
      std::cerr << "Error reading from CAN socket: " << strerror(errno)
                << std::endl;
      return currentRPM;
    }
  }

  if (nbytes < sizeof(struct can_frame)) {
    std::cerr << "Incomplete CAN frame received" << std::endl;
    return currentRPM;
  }

  // Check if this is the RPM message we're looking for
  if (frame.can_id == Config::RPM_CAN_ID) {
    // Debug: Print raw CAN frame data
    if (Config::ENABLE_CAN_DEBUG) {
      std::cout << "CAN ID: 0x" << std::hex << frame.can_id << std::dec
                << ", DLC: " << (int)frame.can_dlc << ", Data: ";
      for (int i = 0; i < frame.can_dlc; i++) {
        printf("%02X ", frame.data[i]);
      }
      std::cout << std::endl;
    }

    // Extract RPM data based on configuration
    if (Config::USE_32BIT_RPM && frame.can_dlc >= Config::RPM_BYTE_OFFSET + 4) {
      // 32-bit RPM parsing (big-endian)
      uint32_t rawRPM32 = (frame.data[Config::RPM_BYTE_OFFSET] << 24) |
                          (frame.data[Config::RPM_BYTE_OFFSET + 1] << 16) |
                          (frame.data[Config::RPM_BYTE_OFFSET + 2] << 8) |
                          frame.data[Config::RPM_BYTE_OFFSET + 3];

      currentRPM = rawRPM32 * Config::RPM_SCALE_FACTOR;
      if (Config::ENABLE_CAN_DEBUG) {
        std::cout << "Raw RPM (32-bit): " << rawRPM32
                  << ", Scaled: " << currentRPM << std::endl;
      }
    } else if (frame.can_dlc >= Config::RPM_BYTE_OFFSET + 2) {
      // 16-bit RPM parsing (big-endian)
      uint16_t rawRPM16 = (frame.data[Config::RPM_BYTE_OFFSET] << 8) |
                          frame.data[Config::RPM_BYTE_OFFSET + 1];

      currentRPM = rawRPM16 * Config::RPM_SCALE_FACTOR;
      if (Config::ENABLE_CAN_DEBUG) {
        std::cout << currentRPM << std::endl;
      }
    }
  }

  return currentRPM;
}
class EMAFilter {
 private:
  double filteredRPM;
  double filteredDot;
  double filteredDDot;
  double previousRPM;
  double previousDot;
  double previousTime;

 public:
  EMAFilter();
  double PredictRPM(double dt);
  double getFilteredRPM() { return filteredRPM; }
  double getFilteredDot() { return filteredDot; }
  double getFilteredDDot() { return filteredDDot; }
  double getPreviousRPM() { return previousRPM; }
  double getPreviousDot() { return previousDot; }
  double getPreviousTime() { return previousTime; }
  void setFilteredRPM(double rpm) { filteredRPM = rpm; }
  void setFilteredDot(double dot) { filteredDot = dot; }
  void setFilteredDDot(double ddot) { filteredDDot = ddot; }
  void setPreviousRPM(double rpm) { previousRPM = rpm; }
  void setPreviousDot(double dot) { previousDot = dot; }
  void setPreviousTime(double time) { previousTime = time; }
  void filterValues(int currentRPM);
};

EMAFilter::EMAFilter() {
  filteredRPM = 0.0;
  filteredDDot = 0.0;
  filteredDot = 0.0;
  previousRPM = 0.0;
  previousDot = 0.0;
  previousTime = 0.0;
}

double EMAFilter::PredictRPM(double dt) {
  return filteredRPM + filteredDot * dt + 0.5 * filteredDDot * (dt * dt);
}

void EMAFilter::filterValues(int currentRPM) {
  using namespace Config;

  filteredRPM = ALPHA_RPM * currentRPM + (1 - ALPHA_RPM) * filteredRPM;

  double rpm_dot = (filteredRPM - previousRPM) / SAMPLE_TIME;

  filteredDot = ALPHA_DOT * rpm_dot + (1 - ALPHA_DOT) * filteredDot;

  double rpm_ddot = (filteredDot - previousDot) / SAMPLE_TIME;

  filteredDDot = ALPHA_DDOT * rpm_ddot + (1 - ALPHA_DDOT) * filteredDDot;

  setPreviousRPM(filteredRPM);
  setPreviousDot(filteredDot);
}

class CanInterface {
  CAN CanBUS;
  EMAFilter EMA;
  double predictedRPM;
  bool upshiftSoundPlayed;  // Flag to track if upshift sound has been played

 public:
  CanInterface();
  void run();
  double getPredictedRPM() { return predictedRPM; }
  void setPredictedRPM(double rpm) { predictedRPM = rpm; }
};

CanInterface::CanInterface() {
  predictedRPM = 0.0;
  upshiftSoundPlayed = false;  // Initialize the flag

  // Initialize CAN connection
  if (!CanBUS.initSocketCAN()) {
    std::cerr << "Failed to initialize SocketCAN." << std::endl;
  }
}

void CanInterface::run() {
  std::cout << "Starting CAN interface" << std::endl;

  while (true) {
    auto currentTime = std::chrono::steady_clock::now();
    auto timeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           currentTime.time_since_epoch())
                           .count();

    EMA.setPreviousTime(timeElapsed);

    if (timeElapsed - EMA.getPreviousTime() >= Config::SAMPLE_TIME * 1000) {
      // Read current RPM from CAN bus
      double currentRpmValue = CanBUS.getCurrentRPM();

      // Update the CAN object's RPM value
      CanBUS.setCurrentRPM(currentRpmValue);
    }

    EMA.filterValues(CanBUS.getCurrentRPM());
    setPredictedRPM(EMA.PredictRPM(Config::PREDICTIVE_TIME));

    std::cout << "Current RPM: " << CanBUS.getCurrentRPM()
              << ", Filtered RPM: " << EMA.getFilteredRPM()
              << ", Predicted RPM: " << getPredictedRPM() << std::endl;

    // Only print and play sound once when threshold is reached
    if (getPredictedRPM() >= Config::MAXIMUM_UPSHIFT_RPM &&
        !upshiftSoundPlayed) {
      CanBUS.PlayUpshiftSound();
      upshiftSoundPlayed = true;  // Set flag to prevent repeated output
    }

    // Reset flag when RPM drops below threshold (optional - for multiple
    // upshifts)
    if (getPredictedRPM() < Config::MAXIMUM_UPSHIFT_RPM) {
      upshiftSoundPlayed = false;
    }
  }
}

int main() {
  CanInterface canint;

  canint.run();

  return 0;
}