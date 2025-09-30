
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
#include <thread>

namespace Config {
const double MAXIMUM_UPSHIFT_RPM = 13500.0;
const double MINIMUM_DOWNSHIFT_RPM = 2000.0;
const double SAMPLE_TIME = 0.3;
const double ALPHA_RPM = 0.9;
const double ALPHA_DOT = 0.8;
const double ALPHA_DDOT = 0.6;
const double PREDICTIVE_TIME = 0.6;

// CAN Configuration
const char* CAN_INTERFACE = "vcan0";
const canid_t RPM_CAN_ID = 0x123;
const double RPM_SCALE_FACTOR = 1.0;

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
  void PlayDownshiftSound();

  // Getters
  double getCurrentRPM();

  // Setters
  void setCurrentRPM(double rpm) { currentRPM = rpm; }
};

void CAN::PlayUpshiftSound() {
  system(
      "(speaker-test -t sine -f 800) > /dev/null 2>&1  & pid=$!; sleep 1.0s; "
      "kill -9 $pid");
  std::cout << "Playing upshift sound!" << std::endl;
}

void CAN::PlayDownshiftSound() {
  system(
      "(speaker-test -t sine -f 500) > /dev/null 2>&1  & pid=$!; sleep 1.0s; "
      "kill -9 $pid");
  std::cout << "Playing downshift sound!" << std::endl;
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
    // Simple 16-bit RPM extraction from first two bytes (big-endian)
    if (frame.can_dlc >= 2) {
      uint16_t rawRPM = (frame.data[0] << 8) | frame.data[1];
      currentRPM = rawRPM * Config::RPM_SCALE_FACTOR;
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
  bool initialized;

 public:
  EMAFilter();
  double PredictRPM(double dt);
  double getFilteredRPM() { return filteredRPM; }
  void setPreviousRPM(double rpm) { previousRPM = rpm; }
  void setPreviousDot(double dot) { previousDot = dot; }
  double getFilteredDDot() { return filteredDDot; }
  void filterValues(int currentRPM);
};

EMAFilter::EMAFilter() {
  filteredRPM = 0.0;
  filteredDDot = 0.0;
  filteredDot = 0.0;
  previousRPM = 0.0;
  previousDot = 0.0;
  initialized = false;
}

double EMAFilter::PredictRPM(double dt) {
  if (!initialized) return 0.0;

  return filteredRPM + filteredDot * dt + 0.5 * filteredDDot * (dt * dt);
}

void EMAFilter::filterValues(int currentRPM) {
  using namespace Config;

  if (!initialized && currentRPM > 0) {
    filteredRPM = currentRPM;
    previousRPM = currentRPM;
    filteredDot = 0.0;
    previousDot = 0.0;
    filteredDDot = 0.0;
    initialized = true;
    return;
  }

  if (!initialized) {
    return;  // Skip filtering until we have a valid initial RPM
  }

  filteredRPM = ALPHA_RPM * currentRPM + (1 - ALPHA_RPM) * filteredRPM;

  double rpm_dot = (currentRPM - previousRPM) / SAMPLE_TIME;

  filteredDot = ALPHA_DOT * rpm_dot + (1 - ALPHA_DOT) * filteredDot;

  double rpm_ddot = (filteredDot - previousDot) / SAMPLE_TIME;

  filteredDDot = ALPHA_DDOT * rpm_ddot + (1 - ALPHA_DDOT) * filteredDDot;

  setPreviousRPM(currentRPM);
  setPreviousDot(filteredDot);
}

class CanInterface {
  CAN CanBUS;
  EMAFilter EMA;
  double predictedRPM;
  bool upshiftSoundPlayed;  // Flag to track if upshift sound has been played
  bool
      downshiftSoundPlayed;  // Flag to track if downshift sound has been played

 public:
  CanInterface();
  void run();
  double getPredictedRPM() { return predictedRPM; }
  void setPredictedRPM(double rpm) { predictedRPM = rpm; }
};

CanInterface::CanInterface() {
  upshiftSoundPlayed = false;    // Initialize the flag
  downshiftSoundPlayed = false;  // Initialize the flag

  // Initialize CAN connection
  if (!CanBUS.initSocketCAN()) {
    std::cerr << "Failed to initialize SocketCAN." << std::endl;
  }
}

void CanInterface::run() {
  std::cout << "Starting CAN interface" << std::endl;

  auto lastSampleTime = std::chrono::steady_clock::now();

  while (true) {
    auto currentTime = std::chrono::steady_clock::now();
    auto timeSinceLastSample =
        std::chrono::duration_cast<std::chrono::milliseconds>(currentTime -
                                                              lastSampleTime)
            .count();

    if (timeSinceLastSample >= Config::SAMPLE_TIME * 1000) {
      // Read current RPM from CAN bus and filter the values
      EMA.filterValues(CanBUS.getCurrentRPM());

      // Update last sample time
      lastSampleTime = currentTime;
    }

    setPredictedRPM(EMA.PredictRPM(Config::PREDICTIVE_TIME));

    std::cout << "Current RPM: " << CanBUS.getCurrentRPM()
              << " | Predicted RPM: " << getPredictedRPM() << std::endl;

    // Only print and play sound once when threshold is reached
    if (getPredictedRPM() >= Config::MAXIMUM_UPSHIFT_RPM &&
        !upshiftSoundPlayed && EMA.getFilteredDDot() > 0) {
      CanBUS.PlayUpshiftSound();
      upshiftSoundPlayed = true;  // Set flag to prevent repeated output
    } else if (getPredictedRPM() <= Config::MINIMUM_DOWNSHIFT_RPM &&
               !downshiftSoundPlayed && EMA.getFilteredDDot() < 0) {
      CanBUS.PlayDownshiftSound();
      downshiftSoundPlayed = true;  // Set flag to prevent repeated output
    }

    // Reset flag when RPM drops below threshold (optional - for multiple
    // upshifts)
    if (getPredictedRPM() < Config::MAXIMUM_UPSHIFT_RPM) {
      upshiftSoundPlayed = false;
    }

    if (getPredictedRPM() > Config::MINIMUM_DOWNSHIFT_RPM) {
      downshiftSoundPlayed = false;
    }

    // Sleep for a short duration to prevent busy-waiting
    std ::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int main() {
  CanInterface canint;

  canint.run();

  return 0;
}
