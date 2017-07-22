#include <Arduino.h>
#include <unity.h>

#ifdef UNIT_TEST

void test_setup() {

}

void test_preconditions_success() {
    
}

void test_preconditions_fail() {

}

void test_phase_one_setup() {

}

void test_phase_one_setup_run_once() {

}

void test_phase_one_check_wait() {
    // Set the timer so that it is still going to return wait
}

void test_phase_one_check_success() {
    // Set the pulsecount so the flow rate is good.
    // Ensure RESULT_SUCCESS is returned.
}

void test_phase_one_check_fail() {
    // Reasons for failure:
    // - Low Flow Rate
    // - High Flow Rate
}


void setup() {
    // NOTE!!! Wait for > 2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();    // IMPORTANT LINE!
    // RUN_TEST(test_led_builtin_pin_number);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

}

#endif