#include "app/index.hpp"

#include "tim.h"

#include "config.hpp"

#include "svpwm_test.hpp"

#include "logger.hpp"
LOGGER("app")

using namespace wibot;

class App {
public:
  void boot() {
    LOG_I("Starting SVPWM unit tests...");

    // Run SVPWM tests
    SvpwmTest::runTests();

    LOG_I("All SVPWM unit tests passed.");
  };
};

extern "C" void bootApp() { BootFrom<App>::boot("WibotFOC-G4", "0.0.1"); };