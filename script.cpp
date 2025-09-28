#include <chrono>
#include <iostream>

namespace Config {
const double MAXIMUM_UPSHIFT_RPM = 13500.0;
const double SAMPLE_TIME = 0.5;
const double ALPHA_RPM = 0.2;
const double ALPHA_DOT = 0.2;
const double ALPHA_DDOT = 0.15;
}  // namespace Config

class CAN {
 private:
  double currentRPM;
  double predictiveTime;

 public:
  CAN();
  void PlayUpshiftSound();

  // Getters
  double getCurrentRPM() { return currentRPM; }
  double getPredictiveTime() { return predictiveTime; }

  // Setters
  void setCurrentRPM(double rpm) { currentRPM = rpm; }
  void setPredictiveTime(double time) { predictiveTime = time; }
};

CAN::CAN() {
  currentRPM = 0.0;
  predictiveTime = 0.0;
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
  int PredictRPM(int dt);
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

int EMAFilter::PredictRPM(int dt) {
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

 public:
  void run();
};

void CanInterface::run() {}

int main() {
  CanInterface canint;

  canint.run();

  return 0;
}