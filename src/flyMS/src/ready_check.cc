
#include "flyMS/ready_check.h"

#include <chrono>

#include "rc/dsm.h"
#include "rc/start_stop.h"
#include "rc/led.h"

// Default Constructor
ReadyCheck::ReadyCheck() {
  is_running_.store(true);
}

// Desctuctor
ReadyCheck::~ReadyCheck() {
  is_running_.store(false);
  rc_dsm_cleanup();
}


int ReadyCheck::WaitForStartSignal() {
  // Initialze the serial RC hardware
  if(rc_dsm_init()) {
    return -1;
  }

  //Toggle the kill switch to get going, to ensure controlled take-off
  //Keep kill switch down to remain operational
  int count = 1, toggle = 0, reset_toggle = 0;
  float switch_val[2] = {0.0f , 0.0f};
  bool first_run = true;
  printf("Toggle the kill swtich twice and leave up to initialize\n");
  while (count < 6 && rc_get_state() != EXITING && is_running_.load()) {
    // Blink the green LED light to signal that the program is ready
    reset_toggle++; // Only blink the led 1 in 20 times this loop runs
    if (toggle) {
      rc_led_set(RC_LED_GREEN, 0);
      if (reset_toggle == 20) {
        toggle = 0;
        reset_toggle = 0;
      }
    } else {
      rc_led_set(RC_LED_GREEN, 1);
      if (reset_toggle == 20) {
        toggle = 1;
        reset_toggle = 0;
      }
    }

    if (rc_dsm_is_new_data()) {
      //Skip the first run to let data history fill up
      if (first_run) {
        switch_val[1] = rc_dsm_ch_normalized(5);
        first_run = false;
        continue;
      }
      switch_val[1] = switch_val[0];
      switch_val[0] = rc_dsm_ch_normalized(5);

      if (switch_val[0] < 0.25 && switch_val[1] > 0.25)
        count++;
      if (switch_val[0] > 0.25 && switch_val[1] < 0.25)
        count++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Run at 100 Hz
  }

  //make sure the kill switch is in the position to fly before starting
  while (switch_val[0] < 0.25 && rc_get_state() != EXITING && is_running_.load()) {
    if (rc_dsm_is_new_data()) {
      switch_val[0] = rc_dsm_ch_normalized(5);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  printf("\nInitialized! Starting program\n");
  rc_led_set(RC_LED_GREEN, 1);
  return 0;
}
