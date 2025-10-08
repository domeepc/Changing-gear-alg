
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

namespace Config {
const double MAXIMUM_UPSHIFT_RPM = 13500.0;
const double MINIMUM_DOWNSHIFT_RPM = 2000.0;
const double SAMPLE_TIME = 0.05;
const double ALPHA_RPM = 0.2;
const double ALPHA_DOT = 0.4;
const double ALPHA_DDOT = 0.3;
const double PREDICTIVE_TIME = 0.25;
const double MIN_OPERATING_RPM = 1000.0;
const double ACCELERATION_THRESHOLD = 50.0;
const double UPSHIFT_HYSTERESIS = 300.0;  // Make hysteresis configurable
const double DOWNSHIFT_HYSTERESIS = 300.0;
const double UPSHIFT_COOLDOWN_TIME = 0.5;

// CAN Configuration
const char* CAN_INTERFACE = "vcan0";
const canid_t RPM_CAN_ID = 0x405;
const double RPM_SCALE_FACTOR = 1.0;
const canid_t GEAR_CAN_ID = 0x402;
const double GEAR_SCALE_FACTOR = 1.0;

}  // namespace Config

class CAN {
 private:
  double currentRPM;
  int socketFd;
  int currentGear;
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
  void updateCanData();

  // Getters
  double getCurrentRPM() { return currentRPM; };
  int getCurrentGear() { return currentGear; };

  // Setters
  void setCurrentRPM(double rpm) { currentRPM = rpm; }
  void setCurrentGear(int gear) { currentGear = gear; }
};

void CAN::PlayUpshiftSound() {
  int result =
      system("timeout 1s speaker-test -t sine -f 800 > /dev/null 2>&1 &");
  if (result != 0) {
    std::cerr << "Error playing upshift sound" << std::endl;
  } else {
    std::cout << "🔴 UPSHIFT!" << std::endl;
  }
}

void CAN::PlayDownshiftSound() {
  int result =
      system("timeout 1s speaker-test -t sine -f 500 > /dev/null 2>&1 &");
  if (result != 0) {
    std::cerr << "Error playing downshift sound" << std::endl;
  } else {
    std::cout << "🔵 DOWNSHIFT!" << std::endl;
  }
}

CAN::CAN() {
  currentRPM = 0.0;
  socketFd = -1;
  currentGear = 0;
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

void CAN::updateCanData() {
  if (!isConnected) return;

  struct can_frame frame;
  ssize_t nbytes = read(socketFd, &frame, sizeof(struct can_frame));

  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return;
    else {
      std::cerr << "Error reading from CAN socket: " << strerror(errno)
                << std::endl;
      return;
    }
  }

  if (nbytes < sizeof(struct can_frame)) {
    std::cerr << "Incomplete CAN frame received" << std::endl;
    return;
  }
  else if (frame.can_id == Config::RPM_CAN_ID) {
    double newRPM = 0.0;
    // Simple 16-bit RPM extraction from first two bytes (big-endian)
    if (frame.can_dlc >= 2) {
      uint16_t rawRPM = (frame.data[0] << 8) | frame.data[1];
      newRPM = rawRPM * Config::RPM_SCALE_FACTOR;
    } else if (frame.can_dlc >= 1) {
      uint8_t rawRPM = frame.data[0];
      newRPM = rawRPM * Config::RPM_SCALE_FACTOR;
    }

    if (newRPM >= 0 && newRPM <= 21000) {
      setCurrentRPM(newRPM);
    }
  } else if (frame.can_id == Config::GEAR_CAN_ID) {
    int newGear = 0;  // Default to neutral or invalid gear
    if (frame.can_dlc >= 6) {
      uint16_t rawGear = (frame.data[4] << 8) | frame.data[5];
      newGear = static_cast<int>(rawGear * Config::GEAR_SCALE_FACTOR);
    }

    if (newGear >= -1 && newGear <= 5) {  // Assuming gears -1 (R) to 6
      setCurrentGear(newGear);
    } else
      std::cerr << "Received invalid gear value: " << newGear << std::endl;
  }
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
  double getFilteredDDot() { return filteredDDot; }
  void filterValues(double currentRPM);
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

void EMAFilter::filterValues(double currentRPM) {
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

  previousRPM = filteredRPM;
  previousDot = filteredDot;
}

class CanInterface {
  CAN CanBUS;
  EMAFilter EMA;
  double predictedRPM;
  bool upshiftSoundPlayed;    // Flag to track if upshift sound has been
                              // played
  bool downshiftSoundPlayed;  // Flag to track if downshift sound has been
                              // played
  bool engineRunning;
  std::chrono::steady_clock::time_point
      lastDownshiftTime;  // Time of last downshift
  int previousGear;
  std::chrono::steady_clock::time_point lastGearChangeTime;
  bool upshiftCooldownActive;

 public:
  CanInterface();
  void run();
  double getPredictedRPM() { return predictedRPM; }
  void setPredictedRPM(double rpm) { predictedRPM = rpm; }
};

CanInterface::CanInterface() {
  predictedRPM = 0.0;            // Initialize predicted RPM
  upshiftSoundPlayed = false;    // Initialize the flag
  downshiftSoundPlayed = false;  // Initialize the flag
  engineRunning = false;         // Initialize engine running state
  previousGear = 0;
  upshiftCooldownActive = false;
  lastGearChangeTime = std::chrono::steady_clock::now();

  // Initialize CAN connection
  if (!CanBUS.initSocketCAN()) {
    std::cerr << "Failed to initialize SocketCAN." << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void CanInterface::run() {
  std::cout << "Starting CAN interface" << std::endl;

  auto lastSampleTime = std::chrono::steady_clock::now();
  int sampleCount = 0;
  const int MIN_SAMPLES = 5;

  while (true) {
    auto currentTime = std::chrono::steady_clock::now();

    CanBUS.updateCanData();

    auto timeSinceLastSample =
        std::chrono::duration_cast<std::chrono::milliseconds>(currentTime -
                                                              lastSampleTime)
            .count();

    if (timeSinceLastSample >= Config::SAMPLE_TIME * 1000) {
      // Read current RPM from CAN bus and filter the values
      EMA.filterValues(CanBUS.getCurrentRPM());
      int currentGear = CanBUS.getCurrentGear();

      if (currentGear != previousGear && previousGear != 0) {
        if (currentGear < previousGear) {
          std::cout << "Gear: " << previousGear << " -> " << currentGear
                    << std::endl;
          upshiftCooldownActive = true;
          lastGearChangeTime = currentTime;
        } else if (currentGear > previousGear) {
          std::cout << "Gear: " << previousGear << " -> " << currentGear
                    << std::endl;
        }
        previousGear = currentGear;
      } else if (previousGear == 0 && currentGear > 0) {
        previousGear = currentGear;
      }

      sampleCount++;
      lastSampleTime = currentTime;
    }

    if (upshiftCooldownActive) {
      auto timeSinceGearChange =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              currentTime - lastGearChangeTime)
              .count();
      if (timeSinceGearChange >= Config::UPSHIFT_COOLDOWN_TIME * 1000) {
        upshiftCooldownActive = false;
      }
    }

    if (!engineRunning &&
        CanBUS.getCurrentRPM() > Config::MINIMUM_DOWNSHIFT_RPM) {
      engineRunning = true;
      std::cout << "Engine started" << std::endl;
    }

    if (sampleCount >= MIN_SAMPLES) {
      setPredictedRPM(EMA.PredictRPM(Config::PREDICTIVE_TIME));

      double accelerationMagnitude = std::abs(EMA.getFilteredDDot());

      if (engineRunning) {
        // Only print and play sound once when threshold is reached
        if (getPredictedRPM() >= Config::MAXIMUM_UPSHIFT_RPM &&
            EMA.getFilteredDDot() > 0 && !upshiftSoundPlayed &&
            !upshiftCooldownActive &&
            accelerationMagnitude > Config::ACCELERATION_THRESHOLD &&
            CanBUS.getCurrentRPM() > Config::MIN_OPERATING_RPM) {
          CanBUS.PlayUpshiftSound();
          upshiftSoundPlayed = true;  // Set flag to prevent repeated output

        } else if (getPredictedRPM() <= Config::MINIMUM_DOWNSHIFT_RPM &&
                   !downshiftSoundPlayed && EMA.getFilteredDDot() < 0 &&
                   accelerationMagnitude > Config::ACCELERATION_THRESHOLD &&
                   CanBUS.getCurrentRPM() > Config::MIN_OPERATING_RPM) {
          CanBUS.PlayDownshiftSound();
          downshiftSoundPlayed = true;  // Set flag to prevent repeated output
          lastDownshiftTime = std::chrono::steady_clock::now();
        }
      }
    } else {
      setPredictedRPM(CanBUS.getCurrentRPM());
    }

    std::cout << "Current RPM: " << CanBUS.getCurrentRPM()
              << " | Predicted RPM: " << getPredictedRPM()
              << " | Gear: " << CanBUS.getCurrentGear()
              << " | Cooldown Active: "
              << (upshiftCooldownActive ? "Yes" : "No") << std::endl;

    // Reset flag when RPM drops below threshold (optional - for multiple
    // upshifts)
    if (getPredictedRPM() <
        Config::MAXIMUM_UPSHIFT_RPM - Config::UPSHIFT_HYSTERESIS) {
      upshiftSoundPlayed = false;
    }

    if (getPredictedRPM() >
        Config::MINIMUM_DOWNSHIFT_RPM + Config::DOWNSHIFT_HYSTERESIS) {
      downshiftSoundPlayed = false;
    }

    // Sleep for a short duration to prevent busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  CanInterface canint;

  canint.run();

  return 0;
}
