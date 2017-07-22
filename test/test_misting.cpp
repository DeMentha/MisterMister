#include <Arduino.h>
#include <unity.h>

#ifdef UNIT_TEST

void test_setup() {

}

void test_preconditions_success() {
    TEST_ASSERT_EQUAL(13, 13);
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
    RUN_TEST(test_preconditions_success);

    pinMode(LED_BUILTIN, OUTPUT);
    UNITY_END();
}

void loop() {

}

#endif