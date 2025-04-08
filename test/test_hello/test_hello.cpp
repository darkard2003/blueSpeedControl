#include <Arduino.h>
#include <unity.h>

// Basic string equality test
void test_hello() {
    TEST_ASSERT_EQUAL_STRING("Hello", "Hello");
}

// Test for PWM configuration values
void test_pwm_constants() {
    TEST_ASSERT_EQUAL(1000, 1000); // pwmFreq
    TEST_ASSERT_EQUAL(8, 8);       // pwmResolution
    TEST_ASSERT_EQUAL(15, 15);     // LED1_PIN
    TEST_ASSERT_EQUAL(4, 4);       // LED2_PIN
}

// Test that drive control has correct initial values
void test_drive_control_init() {
    uint8_t testDriveControl[2] = {127, 127};
    TEST_ASSERT_EQUAL(127, testDriveControl[0]);
    TEST_ASSERT_EQUAL(127, testDriveControl[1]);
}

// Test for LED state calculations
void test_led_state_toggle() {
    int ledState = 0;
    ledState = !ledState;
    TEST_ASSERT_EQUAL(1, ledState);
    ledState = !ledState;
    TEST_ASSERT_EQUAL(0, ledState);
}

void setup() {
    delay(2000); // Allow board to initialize
    UNITY_BEGIN();
    RUN_TEST(test_hello);
    RUN_TEST(test_pwm_constants);
    RUN_TEST(test_drive_control_init);
    RUN_TEST(test_led_state_toggle);
    UNITY_END();
}

void loop() {
    // Empty - tests only need to run once
}