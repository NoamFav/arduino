/**
 * PID Motor Position Control for Arduino Feather M0
 * Controls motor position using potentiometer feedback with enhanced output formatting
 */

// Pin Definitions
#define MOTOR_PWM_PIN 9      // PWM output (0-255)
#define MOTOR_DIR_PIN 20     // Direction control (SDA as GPIO)
#define POTENTIOMETER_PIN A0 // Position feedback

// Timing Configuration
static const float SAMPLE_TIME_S = 0.010f; // 10ms sample time
static const uint32_t SAMPLE_TIME_MS = (uint32_t)(SAMPLE_TIME_S * 1000);

// Motor Control Limits
static const int MOTOR_OUTPUT_MIN = -255;
static const int MOTOR_OUTPUT_MAX = 255;
static const int MIN_PWM_KICK = 10;          // Minimum PWM to overcome static friction
static const int MOTOR_DEADBAND = 2;         // Below this value, stop motor
static const float INTEGRAL_DEADBAND = 2.0f; // Don't integrate small errors
static const float SMOOTHING_ALPHA = 0.20f;  // EMA filter coefficient for pot reading

// Potentiometer Calibration (adjust these values based on your hardware)
static const float POT_RAW_MIN = 200.0f; // Minimum raw ADC value
static const float POT_RAW_MAX = 800.0f; // Maximum raw ADC value
static const float POT_RANGE = 1023.0f;

// Output formatting options
#define OUTPUT_MODE_PLOTTER 0 // Simple CSV for Arduino Serial Plotter
#define OUTPUT_MODE_VERBOSE 1 // Detailed human-readable output
#define OUTPUT_MODE_JSON 2    // JSON format for advanced monitoring

static const uint8_t OUTPUT_FORMAT = 0; // Change this to switch formats

// Performance tracking
struct PerformanceMetrics {
    uint32_t loop_count = 0;
    uint32_t last_report_time = 0;
    float max_error = 0.0f;
    float avg_error = 0.0f;
    float error_sum = 0.0f;
    static const uint32_t REPORT_INTERVAL_MS = 5000; // Report every 5 seconds
};

// PID Controller Structure
struct PIDController {
    // Tuning parameters
    float kp = 0.8f;  // Proportional gain
    float ki = 0.25f; // Integral gain
    float kd = 0.02f; // Derivative gain

    // Controller state
    float sample_time = SAMPLE_TIME_S;
    float integral_sum = 0.0f;
    float previous_error = 0.0f;

    // Output limits
    float output_min = MOTOR_OUTPUT_MIN;
    float output_max = MOTOR_OUTPUT_MAX;
};

// Global Variables
PIDController pid_controller;
float target_position = 512.0f; // Target position (0-1023)
char serial_buffer[64];
size_t serial_buffer_length = 0;
PerformanceMetrics metrics;

/**
 * Map raw ADC reading to 0-1023 range with calibration
 */
float mapPotentiometerValue(int raw) {
    long v = (long)(raw - POT_RAW_MIN) * 1023L / (POT_RAW_MAX - POT_RAW_MIN);
    if (v < 0)
        v = 0;
    if (v > 1023)
        v = 1023;
    return (float)v;
}

/**
 * Read potentiometer position with exponential moving average filtering
 */
float readPotentiometerPosition() {
    static float filtered_value = mapPotentiometerValue(analogRead(POTENTIOMETER_PIN));
    int raw_reading = analogRead(POTENTIOMETER_PIN);

    float mapped = mapPotentiometerValue(raw_reading);
    filtered_value += SMOOTHING_ALPHA * (mapped - filtered_value);

    return filtered_value; // always in 0–1023 space
}

/**
 * Set motor speed and direction
 * @param output: Motor command (-255 to +255)
 */
void setMotorOutput(float output) {
    bool direction_forward = (output >= 0.0f);
    int pwm_value = (int)fabsf(output);

    // Apply deadband and minimum kick
    if (pwm_value < MOTOR_DEADBAND) {
        pwm_value = 0; // Stop motor for small commands
    } else if (pwm_value < MIN_PWM_KICK) {
        pwm_value = MIN_PWM_KICK; // Overcome static friction
    }

    // Clamp to PWM range
    pwm_value = constrain(pwm_value, 0, 255);

    // Set direction and PWM
    digitalWrite(MOTOR_DIR_PIN, direction_forward ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN, pwm_value);
}

/**
 * Send formatted status update based on selected output mode
 */
void sendStatusUpdate(float position, float error, float output) {
    static uint32_t update_counter = 0;
    update_counter++;

    switch (OUTPUT_FORMAT) {
    case OUTPUT_MODE_PLOTTER:
        // Simple CSV format for Arduino Serial Plotter
        Serial.print(target_position, 2);
        Serial.print(',');
        Serial.print(position, 2);
        Serial.print(',');
        Serial.println(output, 1);
        break;

    case OUTPUT_MODE_VERBOSE:
        // Human-readable format with labels
        Serial.print("Target: ");
        Serial.print(target_position, 1);
        Serial.print(" | Position: ");
        Serial.print(position, 1);
        Serial.print(" | Error: ");
        Serial.print(error, 1);
        Serial.print(" | Output: ");
        Serial.print(output, 1);
        Serial.print(" | PID(");
        Serial.print(pid_controller.kp, 3);
        Serial.print(",");
        Serial.print(pid_controller.ki, 3);
        Serial.print(",");
        Serial.print(pid_controller.kd, 3);
        Serial.println(")");
        break;

    case OUTPUT_MODE_JSON:
        // JSON format for advanced monitoring tools
        Serial.print("{\"update\":");
        Serial.print(update_counter);
        Serial.print(",\"time\":");
        Serial.print(millis());
        Serial.print(",\"target\":");
        Serial.print(target_position, 2);
        Serial.print(",\"position\":");
        Serial.print(position, 2);
        Serial.print(",\"error\":");
        Serial.print(error, 2);
        Serial.print(",\"output\":");
        Serial.print(output, 1);
        Serial.print(",\"pid\":{\"kp\":");
        Serial.print(pid_controller.kp, 3);
        Serial.print(",\"ki\":");
        Serial.print(pid_controller.ki, 3);
        Serial.print(",\"kd\":");
        Serial.print(pid_controller.kd, 3);
        Serial.print(",\"integral\":");
        Serial.print(pid_controller.integral_sum, 2);
        Serial.println("}}");
        break;
    }
}

/**
 * Update and display performance metrics
 */
void updatePerformanceMetrics(float error) {
    metrics.loop_count++;
    metrics.error_sum += fabsf(error);
    metrics.avg_error = metrics.error_sum / metrics.loop_count;

    if (fabsf(error) > metrics.max_error) {
        metrics.max_error = fabsf(error);
    }

    uint32_t current_time = millis();
    if (current_time - metrics.last_report_time >= PerformanceMetrics::REPORT_INTERVAL_MS) {
        if (OUTPUT_FORMAT == OUTPUT_MODE_VERBOSE) {
            Serial.println("=== Performance Report ===");
            Serial.print("Updates: ");
            Serial.print(metrics.loop_count);
            Serial.print(" | Avg Error: ");
            Serial.print(metrics.avg_error, 2);
            Serial.print(" | Max Error: ");
            Serial.print(metrics.max_error, 2);
            Serial.print(" | Loop Rate: ");
            Serial.print(metrics.loop_count * 1000.0f / PerformanceMetrics::REPORT_INTERVAL_MS, 1);
            Serial.println(" Hz");
            Serial.println("========================");
        }

        metrics.last_report_time = current_time;
        // Reset some metrics for next period
        metrics.max_error = 0.0f;
        metrics.error_sum = 0.0f;
        metrics.loop_count = 0;
    }
}

/**
 * Display help information
 */
void printHelp() {
    Serial.println("=== PID Motor Controller Help ===");
    Serial.println("Commands:");
    Serial.println("  123          - Set target position");
    Serial.println("  sp=123       - Set target position");
    Serial.println("  kp=1.0       - Set proportional gain");
    Serial.println("  ki=0.25      - Set integral gain");
    Serial.println("  kd=0.02      - Set derivative gain");
    Serial.println("  600,1.0,0.25,0.02 - Set target,kp,ki,kd");
    Serial.println("  help         - Show this help");
    Serial.println("  status       - Show current settings");
    Serial.println("Range: Position 0-1023");
    Serial.println("================================");
}

/**
 * Execute one PID control step with anti-windup and integral deadband
 * @param controller: PID controller instance
 * @param error: Current position error
 * @return: Control output
 */
float executePIDStep(PIDController &controller, float error) {
    // Calculate PID terms
    float proportional_term = controller.kp * error;
    float derivative_term =
        controller.kd * (error - controller.previous_error) / controller.sample_time;

    // Calculate proposed integral with deadband
    float proposed_integral = controller.integral_sum;
    if (fabsf(error) > INTEGRAL_DEADBAND) {
        proposed_integral += error;
    }

    // Test if integral update would cause saturation
    float tentative_output = proportional_term +
                             controller.ki * controller.sample_time * proposed_integral +
                             derivative_term;

    // Anti-windup: only update integral if it doesn't worsen saturation
    bool saturated_high = (tentative_output > controller.output_max) && (error > 0);
    bool saturated_low = (tentative_output < controller.output_min) && (error < 0);

    if (!saturated_high && !saturated_low) {
        controller.integral_sum = proposed_integral;
    }

    // Calculate final output
    float output = proportional_term +
                   controller.ki * controller.sample_time * controller.integral_sum +
                   derivative_term;

    // Apply output limits
    output = constrain(output, controller.output_min, controller.output_max);

    // Update state
    controller.previous_error = error;

    return output;
}

static inline bool startsWith(const char *s, const char *p) {
    while (*p) {
        if (*s++ != *p++)
            return false;
    }
    return true;
}

static inline const char *skipWS(const char *s) {
    while (*s == ' ' || *s == '\t')
        ++s;
    return s;
}

// Fast float parser: [+/-]digits[.digits]; no exponent, no locale.
static float parseFloat(const char *s, const char **endptr = nullptr) {
    s = skipWS(s);
    bool neg = false;
    if (*s == '+' || *s == '-') {
        neg = (*s == '-');
        ++s;
    }
    unsigned long ip = 0;
    while (*s >= '0' && *s <= '9') {
        ip = ip * 10 + (*s - '0');
        ++s;
    }
    unsigned long fp = 0, denom = 1;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            fp = fp * 10 + (*s - '0');
            denom *= 10;
            ++s;
        }
    }
    float v = (float)ip + (denom > 1 ? (float)fp / (float)denom : 0.0f);
    if (endptr)
        *endptr = s;
    return neg ? -v : v;
}

// Split by ',', ';', or space into up to 4 tokens (in-place)
static int split4(char *s, char *out[4]) {
    int n = 0;
    s = (char *)skipWS(s);
    while (*s && n < 4) {
        out[n++] = s;
        while (*s && *s != ',' && *s != ';' && *s != ' ')
            ++s;
        if (*s) {
            *s++ = 0;
            s = (char *)skipWS(s);
        }
    }
    return n;
}

/**
 * Parse and execute serial commands
 * Supported formats:
 * - "512" -> set target position
 * - "600,0.9,0.25,0.02" -> set target,kp,ki,kd
 * - "kp=1.0", "ki=0.25", "kd=0.02", "sp=512" -> set individual parameters
 */
void processSerialCommand(const char *command_in) {
    // work on a mutable copy to allow split in-place
    static char buf[64];
    size_t L = 0;
    for (; L < sizeof(buf) - 1 && command_in[L]; ++L)
        buf[L] = command_in[L];
    buf[L] = '\0';

    char *cmd = buf;
    cmd = (char *)skipWS(cmd);
    if (!*cmd)
        return;

    // help / status (string matches)
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
        return;
    }
    if (strcmp(cmd, "status") == 0) {
        Serial.println("=== Current Settings ===");
        Serial.print("Target: ");
        Serial.println(target_position, 1);
        Serial.print("PID Gains - Kp: ");
        Serial.print(pid_controller.kp, 3);
        Serial.print(", Ki: ");
        Serial.print(pid_controller.ki, 3);
        Serial.print(", Kd: ");
        Serial.println(pid_controller.kd, 3);
        Serial.println("======================");
        return;
    }

    // kp=..., ki=..., kd=..., sp=...
    if (startsWith(cmd, "kp=")) {
        pid_controller.kp = parseFloat(cmd + 3);
        Serial.print("ACK kp=");
        Serial.println(pid_controller.kp, 3);
        return;
    }
    if (startsWith(cmd, "ki=")) {
        pid_controller.ki = parseFloat(cmd + 3);
        Serial.print("ACK ki=");
        Serial.println(pid_controller.ki, 3);
        return;
    }
    if (startsWith(cmd, "kd=")) {
        pid_controller.kd = parseFloat(cmd + 3);
        Serial.print("ACK kd=");
        Serial.println(pid_controller.kd, 3);
        return;
    }
    if (startsWith(cmd, "sp=")) {
        target_position = constrain(parseFloat(cmd + 3), 0.0f, POT_RANGE);
        pid_controller.integral_sum = 0.0f;
        Serial.print("ACK sp=");
        Serial.println(target_position, 1);
        return;
    }

    // also accept "sp 800"
    if (startsWith(cmd, "sp ")) {
        target_position = constrain(parseFloat(cmd + 3), 0.0f, POT_RANGE);
        pid_controller.integral_sum = 0.0f;
        Serial.print("ACK sp=");
        Serial.println(target_position, 1);
        return;
    }

    // batch: "target,kp,ki,kd" or "target;kp;ki;kd" or spaced
    char *tok[4];
    int n = split4(cmd, tok);
    if (n == 4) {
        float a = parseFloat(tok[0]);
        float b = parseFloat(tok[1]);
        float c = parseFloat(tok[2]);
        float d = parseFloat(tok[3]);
        target_position = constrain(a, 0.0f, POT_RANGE);
        pid_controller.kp = b;
        pid_controller.ki = c;
        pid_controller.kd = d;
        pid_controller.integral_sum = 0.0f;
        Serial.println("ACK batch");
        return;
    }

    // plain number -> setpoint
    if ((*cmd == '+' || *cmd == '-' || (*cmd >= '0' && *cmd <= '9'))) {
        target_position = constrain(parseFloat(cmd), 0.0f, POT_RANGE);
        pid_controller.integral_sum = 0.0f;
        Serial.print("ACK sp=");
        Serial.println(target_position, 1);
        return;
    }

    Serial.println("Unknown command. Type 'help' for available commands.");
}

/**
 * Non-blocking serial input processing
 */
void processSerialInput() {
    while (Serial.available()) {
        char received_char = (char)Serial.read();

        // Skip carriage return
        if (received_char == '\r') {
            continue;
        }

        // Process complete line
        if (received_char == '\n') {
            serial_buffer[min(serial_buffer_length, sizeof(serial_buffer) - 1)] = '\0';
            processSerialCommand(serial_buffer);
            serial_buffer_length = 0;
        }
        // Add character to buffer if there's space
        else if (serial_buffer_length < sizeof(serial_buffer) - 1) {
            serial_buffer[serial_buffer_length++] = received_char;
        }
    }
}

void setup() {
    // Initialize pins
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);

    // Initialize serial communication
    Serial.begin(115200);

    // Ensure motor is stopped initially
    setMotorOutput(0);

    // Display startup message
    Serial.println("PID Motor Controller Ready");
    Serial.println("Type 'help' for commands");
    printHelp();
}

void loop() {
    static uint32_t last_control_update = 0;
    uint32_t current_time = millis();

    // Execute control loop at fixed sample rate
    if (current_time - last_control_update >= SAMPLE_TIME_MS) {
        last_control_update += SAMPLE_TIME_MS; // Maintain consistent timing

        // Read current position
        float current_position = readPotentiometerPosition();

        // Calculate position error
        float position_error = target_position - current_position;

        // Execute PID control
        float motor_output = executePIDStep(pid_controller, position_error);

        // Apply motor command
        setMotorOutput(motor_output);

        // Update performance tracking
        updatePerformanceMetrics(position_error);

        // Send formatted data for monitoring
        // sendStatusUpdate(current_position, position_error, motor_output);
    }

    // Handle serial communication outside of timed control loop
    processSerialInput();
}
