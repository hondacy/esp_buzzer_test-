// Standalone ESP-IDF app for a passive buzzer driven through a DRV8833.
// DRV8833 IN1 -> ESP GPIO2, DRV8833 IN2 -> ESP GPIO3.
// The web page accepts notes such as "C4 D4 E4" or timed steps such as
// "880,120 0,80 A4,250".
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

extern "C" void app_main(void);

namespace {

constexpr const char *TAG = "esp_buzzer";

// DRV8833 input pins. GPIO2 is a boot strapping pin on some ESP32 boards,
// so keep the DRV8833 input from externally pulling it during reset.
constexpr gpio_num_t BUZZER_IN1_GPIO = GPIO_NUM_2;
constexpr gpio_num_t BUZZER_IN2_GPIO = GPIO_NUM_3;

// Three logic outputs for an external 3-phase HDD spindle motor driver.
// Do not connect a 3-wire HDD motor directly to ESP GPIOs.
constexpr gpio_num_t MOTOR_PHASE_U_GPIO = GPIO_NUM_4;
constexpr gpio_num_t MOTOR_PHASE_V_GPIO = GPIO_NUM_5;
constexpr gpio_num_t MOTOR_PHASE_W_GPIO = GPIO_NUM_6;

constexpr const char *HOSTNAME = "esp-buzzer";
constexpr const char *AP_SSID = "esp-buzzer-setup";
constexpr const char *AP_PASSWORD = ""; // Empty password means open AP.
constexpr uint8_t AP_CHANNEL = 6;
constexpr uint8_t AP_MAX_CLIENTS = 4;
constexpr int WIFI_CONNECTED_BIT = BIT0;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;

constexpr ledc_mode_t BUZZER_LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t BUZZER_LEDC_TIMER = LEDC_TIMER_0;
constexpr ledc_channel_t BUZZER_LEDC_CH_A = LEDC_CHANNEL_0;
constexpr ledc_channel_t BUZZER_LEDC_CH_B = LEDC_CHANNEL_1;
constexpr ledc_timer_bit_t BUZZER_DUTY_RES = LEDC_TIMER_10_BIT;
constexpr uint32_t BUZZER_DUTY = 512; // 50% of 10-bit range.

constexpr uint32_t MIN_FREQ_HZ = 30;
constexpr uint32_t MAX_FREQ_HZ = 12000;
constexpr uint32_t MIN_STEP_MS = 1;
constexpr uint32_t MAX_STEP_MS = 10000;
constexpr uint32_t DEFAULT_NOTE_MS = 250;
constexpr size_t MAX_STEPS = 128;
constexpr size_t MAX_SEQUENCE_CHARS = 2048;
constexpr uint32_t COMMAND_POLL_MS = 20;
constexpr uint32_t INTER_STEP_GAP_MS = 8;
constexpr size_t MAX_WIFI_SSID_CHARS = 32;
constexpr size_t MAX_WIFI_PASSWORD_CHARS = 64;
constexpr uint32_t MOTOR_MIN_STEP_HZ = 5;
constexpr uint32_t MOTOR_MAX_STEP_HZ = 1200;
constexpr uint32_t MOTOR_RAMP_MS = 700;
constexpr uint32_t MOTOR_BUSY_DELAY_MAX_US = 5000;

constexpr const char *DEFAULT_SEQUENCE = "C5,120 rest,80 D5,120 rest,80 E5,180 rest,120 D5,140 C5,220";

struct Step {
    uint32_t freq_hz = 0;     // 0 means rest.
    uint32_t duration_ms = 0;
};

enum class PlayerCommandType : uint8_t {
    Play,
    Stop,
};

struct PlayerCommand {
    PlayerCommandType type;
};

enum class OutputTarget : uint8_t {
    Buzzer,
    Motor,
};

enum class PhaseDrive : uint8_t {
    Low,
    High,
    Floating,
};

SemaphoreHandle_t g_state_mutex = nullptr;
SemaphoreHandle_t g_wifi_mutex = nullptr;
EventGroupHandle_t g_wifi_events = nullptr;
QueueHandle_t g_player_queue = nullptr;
httpd_handle_t g_httpd = nullptr;

std::string g_sequence_text = DEFAULT_SEQUENCE;
std::vector<Step> g_sequence_steps;
OutputTarget g_output_target = OutputTarget::Buzzer;
std::string g_saved_ssid;
std::string g_saved_password;
std::string g_current_ssid;
bool g_playing = false;
bool g_ledc_configured = false;
bool g_setup_ap_active = false;
bool g_sta_connected = false;

class SemaphoreGuard {
public:
    explicit SemaphoreGuard(SemaphoreHandle_t semaphore)
        : semaphore_(semaphore), locked_(!semaphore || xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {
    }

    ~SemaphoreGuard()
    {
        if (semaphore_ && locked_) xSemaphoreGive(semaphore_);
    }

    SemaphoreGuard(const SemaphoreGuard &) = delete;
    SemaphoreGuard &operator=(const SemaphoreGuard &) = delete;

    bool ok() const { return locked_; }

private:
    SemaphoreHandle_t semaphore_;
    bool locked_;
};

std::string trim_copy(const std::string &s)
{
    size_t begin = 0;
    while (begin < s.size() && std::isspace(static_cast<unsigned char>(s[begin]))) ++begin;

    size_t end = s.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;

    return s.substr(begin, end - begin);
}

std::string lower_copy(std::string s)
{
    for (char &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool parse_u32(const std::string &s, uint32_t *value)
{
    if (s.empty()) return false;
    for (char c : s) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }

    char *end = nullptr;
    unsigned long parsed = std::strtoul(s.c_str(), &end, 10);
    if (!end || *end != '\0' || parsed > UINT32_MAX) return false;

    *value = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_note_freq(const std::string &s, uint32_t *freq_hz)
{
    if (s.size() < 2) return false;

    int semitone = 0;
    switch (std::toupper(static_cast<unsigned char>(s[0]))) {
    case 'C':
        semitone = 0;
        break;
    case 'D':
        semitone = 2;
        break;
    case 'E':
        semitone = 4;
        break;
    case 'F':
        semitone = 5;
        break;
    case 'G':
        semitone = 7;
        break;
    case 'A':
        semitone = 9;
        break;
    case 'B':
        semitone = 11;
        break;
    default:
        return false;
    }

    size_t pos = 1;
    if (pos < s.size() && (s[pos] == '#' || s[pos] == 'b' || s[pos] == 'B')) {
        semitone += (s[pos] == '#') ? 1 : -1;
        ++pos;
    }

    if (pos >= s.size()) return false;

    uint32_t octave = 0;
    if (!parse_u32(s.substr(pos), &octave) || octave > 8) return false;

    if (semitone < 0) {
        semitone += 12;
        if (octave == 0) return false;
        --octave;
    } else if (semitone > 11) {
        semitone -= 12;
        ++octave;
    }

    int midi_note = static_cast<int>((octave + 1) * 12 + semitone);
    double freq = 440.0 * std::pow(2.0, (midi_note - 69) / 12.0);
    *freq_hz = static_cast<uint32_t>(std::lround(freq));
    return true;
}

bool parse_pitch(const std::string &text, uint32_t *freq_hz, std::string *error)
{
    std::string pitch = lower_copy(trim_copy(text));

    if (pitch == "r" || pitch == "rest" || pitch == "pause" || pitch == "0") {
        *freq_hz = 0;
        return true;
    }

    if (parse_u32(pitch, freq_hz)) return true;
    if (parse_note_freq(pitch, freq_hz)) return true;

    *error = "bad note or frequency '" + text + "'";
    return false;
}

bool parse_step_token(const std::string &token, Step *step, std::string *error)
{
    size_t sep = token.find_first_of(",:/");
    uint32_t freq = 0;
    uint32_t duration = DEFAULT_NOTE_MS;

    if (sep == std::string::npos) {
        if (!parse_pitch(token, &freq, error)) return false;
    } else {
        std::string pitch_text = trim_copy(token.substr(0, sep));
        std::string duration_text = trim_copy(token.substr(sep + 1));

        if (pitch_text.empty() || duration_text.empty()) {
            *error = "bad step '" + token + "'";
            return false;
        }

        if (!parse_pitch(pitch_text, &freq, error)) return false;
        if (!parse_u32(duration_text, &duration)) {
            *error = "bad duration '" + duration_text + "'";
            return false;
        }
    }

    if (freq != 0 && (freq < MIN_FREQ_HZ || freq > MAX_FREQ_HZ)) {
        char msg[96];
        std::snprintf(msg, sizeof(msg), "frequency must be 0 or %" PRIu32 "..%" PRIu32 " Hz",
                      MIN_FREQ_HZ, MAX_FREQ_HZ);
        *error = msg;
        return false;
    }

    if (duration < MIN_STEP_MS || duration > MAX_STEP_MS) {
        char msg[96];
        std::snprintf(msg, sizeof(msg), "duration must be %" PRIu32 "..%" PRIu32 " ms",
                      MIN_STEP_MS, MAX_STEP_MS);
        *error = msg;
        return false;
    }

    step->freq_hz = freq;
    step->duration_ms = duration;
    return true;
}

bool parse_sequence(const std::string &text, std::vector<Step> *steps, std::string *error)
{
    if (text.size() > MAX_SEQUENCE_CHARS) {
        *error = "sequence is too long";
        return false;
    }

    steps->clear();
    std::string token;

    for (size_t i = 0; i <= text.size(); ++i) {
        char c = (i < text.size()) ? text[i] : ' ';
        bool boundary = std::isspace(static_cast<unsigned char>(c)) || c == ';';

        if (!boundary) {
            token += c;
            continue;
        }

        if (token.empty()) continue;

        if (steps->size() >= MAX_STEPS) {
            *error = "too many steps";
            return false;
        }

        Step step;
        if (!parse_step_token(token, &step, error)) return false;
        steps->push_back(step);
        token.clear();
    }

    if (steps->empty()) {
        *error = "empty sequence";
        return false;
    }

    return true;
}

std::string json_escape(const std::string &s)
{
    std::string out;
    out.reserve(s.size() + 8);

    for (char c : s) {
        switch (c) {
        case '"':
        case '\\':
            out += '\\';
            out += c;
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) >= 0x20) out += c;
            break;
        }
    }

    return out;
}

bool wait_for_player_command(uint32_t duration_ms, PlayerCommand *command);

bool nvs_get_string(nvs_handle_t nvs, const char *key, std::string *value, size_t max_chars)
{
    size_t len = 0;
    esp_err_t err = nvs_get_str(nvs, key, nullptr, &len);
    if (err != ESP_OK || len == 0 || len > max_chars + 1) return false;

    std::vector<char> buffer(len);
    err = nvs_get_str(nvs, key, buffer.data(), &len);
    if (err != ESP_OK) return false;

    *value = buffer.data();
    return true;
}

bool load_wifi_credentials()
{
    nvs_handle_t nvs = 0;
    if (nvs_open("buzzer", NVS_READONLY, &nvs) != ESP_OK) return false;

    std::string ssid;
    std::string password;
    bool found = nvs_get_string(nvs, "ssid", &ssid, MAX_WIFI_SSID_CHARS);
    nvs_get_string(nvs, "password", &password, MAX_WIFI_PASSWORD_CHARS);
    nvs_close(nvs);

    if (!found || ssid.empty()) return false;

    g_saved_ssid = ssid;
    g_saved_password = password;
    return true;
}

bool save_wifi_credentials(const std::string &ssid, const std::string &password)
{
    if (ssid.empty() || ssid.size() > MAX_WIFI_SSID_CHARS || password.size() > MAX_WIFI_PASSWORD_CHARS) return false;

    nvs_handle_t nvs = 0;
    if (nvs_open("buzzer", NVS_READWRITE, &nvs) != ESP_OK) return false;

    esp_err_t err = nvs_set_str(nvs, "ssid", ssid.c_str());
    if (err == ESP_OK) err = nvs_set_str(nvs, "password", password.c_str());
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err != ESP_OK) return false;

    g_saved_ssid = ssid;
    g_saved_password = password;
    return true;
}

void buzzer_force_idle_gpio()
{
    gpio_reset_pin(BUZZER_IN1_GPIO);
    gpio_reset_pin(BUZZER_IN2_GPIO);
    gpio_set_direction(BUZZER_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_IN1_GPIO, 0);
    gpio_set_level(BUZZER_IN2_GPIO, 0);
}

void buzzer_stop()
{
    if (g_ledc_configured) {
        ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CH_A, 0);
        ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CH_B, 0);
    }
    buzzer_force_idle_gpio();
}

bool buzzer_start_tone(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        buzzer_stop();
        return true;
    }

    ledc_timer_config_t timer = {};
    timer.speed_mode = BUZZER_LEDC_MODE;
    timer.timer_num = BUZZER_LEDC_TIMER;
    timer.duty_resolution = BUZZER_DUTY_RES;
    timer.freq_hz = freq_hz;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed for %" PRIu32 " Hz: %s", freq_hz, esp_err_to_name(err));
        buzzer_stop();
        return false;
    }

    ledc_channel_config_t ch_a = {};
    ch_a.gpio_num = BUZZER_IN1_GPIO;
    ch_a.speed_mode = BUZZER_LEDC_MODE;
    ch_a.channel = BUZZER_LEDC_CH_A;
    ch_a.timer_sel = BUZZER_LEDC_TIMER;
    ch_a.duty = BUZZER_DUTY;
    ch_a.hpoint = 0;

    ledc_channel_config_t ch_b = ch_a;
    ch_b.gpio_num = BUZZER_IN2_GPIO;
    ch_b.channel = BUZZER_LEDC_CH_B;
    ch_b.flags.output_invert = 1;

    err = ledc_channel_config(&ch_a);
    if (err == ESP_OK) err = ledc_channel_config(&ch_b);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
        buzzer_stop();
        return false;
    }

    g_ledc_configured = true;
    return true;
}

void motor_set_phase(gpio_num_t gpio, PhaseDrive drive)
{
    if (drive == PhaseDrive::Floating) {
        gpio_set_direction(gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(gpio, GPIO_FLOATING);
        return;
    }

    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, drive == PhaseDrive::High ? 1 : 0);
}

void motor_coast()
{
    motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::Floating);
    motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::Floating);
    motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::Floating);
}

void motor_apply_state(uint32_t state)
{
    switch (state % 6) {
    case 0:
        motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::High);
        motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::Low);
        motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::Floating);
        break;
    case 1:
        motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::High);
        motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::Floating);
        motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::Low);
        break;
    case 2:
        motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::Floating);
        motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::High);
        motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::Low);
        break;
    case 3:
        motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::Low);
        motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::High);
        motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::Floating);
        break;
    case 4:
        motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::Low);
        motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::Floating);
        motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::High);
        break;
    default:
        motor_set_phase(MOTOR_PHASE_U_GPIO, PhaseDrive::Floating);
        motor_set_phase(MOTOR_PHASE_V_GPIO, PhaseDrive::Low);
        motor_set_phase(MOTOR_PHASE_W_GPIO, PhaseDrive::High);
        break;
    }
}

void motor_delay_us(uint32_t delay_us)
{
    if (delay_us > MOTOR_BUSY_DELAY_MAX_US) {
        vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(1, delay_us / 1000)));
    } else {
        esp_rom_delay_us(delay_us);
    }
}

bool motor_run_step_rate(uint32_t step_hz, uint32_t duration_ms, PlayerCommand *command)
{
    if (step_hz == 0) {
        motor_coast();
        return wait_for_player_command(duration_ms, command);
    }

    step_hz = std::min(std::max(step_hz, MOTOR_MIN_STEP_HZ), MOTOR_MAX_STEP_HZ);

    int64_t start_us = esp_timer_get_time();
    int64_t duration_us = static_cast<int64_t>(duration_ms) * 1000;
    int64_t ramp_us = std::min<int64_t>(duration_us, static_cast<int64_t>(MOTOR_RAMP_MS) * 1000);
    uint32_t state = 0;

    while (esp_timer_get_time() - start_us < duration_us) {
        if (xQueueReceive(g_player_queue, command, 0) == pdTRUE) {
            motor_coast();
            return true;
        }

        int64_t elapsed_us = esp_timer_get_time() - start_us;
        uint32_t drive_hz = step_hz;
        if (ramp_us > 0 && step_hz > MOTOR_MIN_STEP_HZ && elapsed_us < ramp_us) {
            drive_hz = MOTOR_MIN_STEP_HZ +
                       static_cast<uint32_t>((static_cast<uint64_t>(step_hz - MOTOR_MIN_STEP_HZ) * elapsed_us) /
                                             ramp_us);
            drive_hz = std::max<uint32_t>(MOTOR_MIN_STEP_HZ, drive_hz);
        }

        motor_apply_state(state++);

        uint32_t period_us = std::max<uint32_t>(500, 1000000UL / drive_hz);
        int64_t remaining_us = duration_us - (esp_timer_get_time() - start_us);
        if (remaining_us <= 0) break;
        motor_delay_us(static_cast<uint32_t>(std::min<int64_t>(period_us, remaining_us)));
    }

    motor_coast();
    return false;
}

void set_playing(bool playing)
{
    SemaphoreGuard lock(g_state_mutex);
    g_playing = playing;
}

bool wait_for_player_command(uint32_t duration_ms, PlayerCommand *command)
{
    uint32_t remaining_ms = duration_ms;
    while (remaining_ms > 0) {
        uint32_t slice_ms = std::min<uint32_t>(remaining_ms, COMMAND_POLL_MS);
        if (xQueueReceive(g_player_queue, command, pdMS_TO_TICKS(slice_ms)) == pdTRUE) return true;
        remaining_ms -= slice_ms;
    }

    return false;
}

void player_task(void *)
{
    PlayerCommand command;

    while (true) {
        if (xQueueReceive(g_player_queue, &command, portMAX_DELAY) != pdTRUE) continue;

        while (true) {
            if (command.type == PlayerCommandType::Stop) {
                buzzer_stop();
                motor_coast();
                set_playing(false);
                break;
            }

            std::vector<Step> steps;
            OutputTarget target = OutputTarget::Buzzer;
            {
                SemaphoreGuard lock(g_state_mutex);
                steps = g_sequence_steps;
                target = g_output_target;
                g_playing = true;
            }

            bool interrupted = false;
            PlayerCommand next_command = {};

            for (size_t i = 0; i < steps.size(); ++i) {
                if (target == OutputTarget::Motor) {
                    buzzer_stop();
                    if (motor_run_step_rate(steps[i].freq_hz, steps[i].duration_ms, &next_command)) {
                        interrupted = true;
                        break;
                    }
                } else {
                    motor_coast();
                    if (steps[i].freq_hz == 0) {
                        buzzer_stop();
                    } else {
                        buzzer_start_tone(steps[i].freq_hz);
                    }

                    if (wait_for_player_command(steps[i].duration_ms, &next_command)) {
                        interrupted = true;
                        break;
                    }

                    buzzer_stop();
                }

                if (i + 1 < steps.size() && wait_for_player_command(INTER_STEP_GAP_MS, &next_command)) {
                    interrupted = true;
                    break;
                }
            }

            buzzer_stop();
            motor_coast();
            set_playing(false);

            if (!interrupted) break;
            command = next_command;
        }
    }
}

std::string read_req_body(httpd_req_t *req)
{
    std::string body;
    if (req->content_len > MAX_SEQUENCE_CHARS + 128) return body;

    body.resize(req->content_len);
    size_t received = 0;

    while (received < req->content_len) {
        int r = httpd_req_recv(req, body.data() + received, req->content_len - received);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) break;
        received += r;
    }

    body.resize(received);
    return body;
}

std::string url_decode(const std::string &in)
{
    std::string out;
    out.reserve(in.size());

    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '%' && i + 2 < in.size()) {
            char hex[3] = {in[i + 1], in[i + 2], 0};
            out += static_cast<char>(std::strtol(hex, nullptr, 16));
            i += 2;
        } else if (in[i] == '+') {
            out += ' ';
        } else {
            out += in[i];
        }
    }

    return out;
}

std::string form_value(const std::string &body, const char *key)
{
    std::string prefix = std::string(key) + "=";
    size_t pos = body.find(prefix);
    if (pos == std::string::npos) return "";

    pos += prefix.size();
    size_t end = body.find('&', pos);
    return url_decode(body.substr(pos, end == std::string::npos ? end : end - pos));
}

void send_response(httpd_req_t *req, const char *type, const std::string &body)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, body.c_str(), body.size());
}

void wifi_event_handler(void *, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        {
            SemaphoreGuard lock(g_state_mutex);
            g_sta_connected = false;
            g_current_ssid.clear();
        }
        if (g_wifi_events) xEventGroupClearBits(g_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "WiFi station disconnected");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = static_cast<ip_event_got_ip_t *>(event_data);
        std::string ssid;
        wifi_ap_record_t ap = {};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            char ssid_buf[33] = {};
            std::memcpy(ssid_buf, ap.ssid, sizeof(ssid_buf) - 1);
            ssid = ssid_buf;
        }

        {
            SemaphoreGuard lock(g_state_mutex);
            g_sta_connected = true;
            g_current_ssid = ssid;
        }
        if (g_wifi_events) xEventGroupSetBits(g_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi connected: %s, IP " IPSTR, ssid.c_str(), IP2STR(&event->ip_info.ip));
    }
}

void start_mdns()
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(err));
        return;
    }

    mdns_hostname_set(HOSTNAME);
    mdns_instance_name_set("ESP Buzzer");
    mdns_service_add("ESP Buzzer Web", "_http", "_tcp", 80, nullptr, 0);
    ESP_LOGI(TAG, "mDNS ready: http://%s.local", HOSTNAME);
}

void init_wifi()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    if (sta_netif) esp_netif_set_hostname(sta_netif, HOSTNAME);

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

bool start_setup_ap()
{
    SemaphoreGuard wifi_lock(g_wifi_mutex);

    wifi_config_t ap_config = {};
    std::strncpy(reinterpret_cast<char *>(ap_config.ap.ssid), AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = std::strlen(AP_SSID);
    ap_config.ap.channel = AP_CHANNEL;
    ap_config.ap.max_connection = AP_MAX_CLIENTS;

    if (AP_PASSWORD[0] == '\0') {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        std::strncpy(reinterpret_cast<char *>(ap_config.ap.password), AP_PASSWORD,
                     sizeof(ap_config.ap.password) - 1);
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Setup AP start failed: %s", esp_err_to_name(err));
        return false;
    }

    {
        SemaphoreGuard state_lock(g_state_mutex);
        g_setup_ap_active = true;
    }
    ESP_LOGI(TAG, "Setup AP active: SSID '%s', IP 192.168.4.1", AP_SSID);
    return true;
}

bool connect_wifi(const std::string &ssid, const std::string &password)
{
    SemaphoreGuard wifi_lock(g_wifi_mutex);
    if (ssid.empty() || ssid.size() > MAX_WIFI_SSID_CHARS || password.size() > MAX_WIFI_PASSWORD_CHARS) return false;

    bool keep_ap = false;
    {
        SemaphoreGuard state_lock(g_state_mutex);
        keep_ap = g_setup_ap_active;
        g_sta_connected = false;
        g_current_ssid.clear();
    }

    if (g_wifi_events) xEventGroupClearBits(g_wifi_events, WIFI_CONNECTED_BIT);

    wifi_mode_t mode = keep_ap ? WIFI_MODE_APSTA : WIFI_MODE_STA;
    esp_err_t err = esp_wifi_set_mode(mode);
    if (err == ESP_OK) esp_wifi_disconnect();

    wifi_config_t sta_config = {};
    std::strncpy(reinterpret_cast<char *>(sta_config.sta.ssid), ssid.c_str(), sizeof(sta_config.sta.ssid) - 1);
    std::strncpy(reinterpret_cast<char *>(sta_config.sta.password), password.c_str(),
                 sizeof(sta_config.sta.password) - 1);
    sta_config.sta.threshold.authmode = password.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_PSK;

    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (err == ESP_OK) err = esp_wifi_connect();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect setup failed for '%s': %s", ssid.c_str(), esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Connecting to WiFi SSID '%s'", ssid.c_str());
    EventBits_t bits = xEventGroupWaitBits(
        g_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) return true;

    esp_wifi_disconnect();
    ESP_LOGW(TAG, "WiFi connection timed out for '%s'", ssid.c_str());
    return false;
}

void disable_setup_ap_later_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    bool should_disable = false;
    {
        SemaphoreGuard state_lock(g_state_mutex);
        should_disable = g_sta_connected && g_setup_ap_active;
    }

    if (should_disable) {
        SemaphoreGuard wifi_lock(g_wifi_mutex);
        esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err == ESP_OK) {
            SemaphoreGuard state_lock(g_state_mutex);
            g_setup_ap_active = false;
            ESP_LOGI(TAG, "Setup AP disabled; use http://%s.local on the WiFi network", HOSTNAME);
        } else {
            ESP_LOGW(TAG, "Failed to disable setup AP: %s", esp_err_to_name(err));
        }
    }

    vTaskDelete(nullptr);
}

esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    SemaphoreGuard wifi_lock(g_wifi_mutex);

    wifi_scan_config_t scan_config = {};
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    std::vector<wifi_ap_record_t> records(count);
    if (count > 0) esp_wifi_scan_get_ap_records(&count, records.data());

    std::string body = "{\"networks\":[";
    for (uint16_t i = 0; i < count; ++i) {
        if (records[i].ssid[0] == '\0') continue;
        if (body.back() != '[') body += ",";
        body += "{\"ssid\":\"";
        body += json_escape(reinterpret_cast<const char *>(records[i].ssid));
        body += "\",\"rssi\":";
        body += std::to_string(records[i].rssi);
        body += "}";
    }
    body += "]}";

    send_response(req, "application/json", body);
    return ESP_OK;
}

esp_err_t wifi_connect_handler(httpd_req_t *req)
{
    std::string body = read_req_body(req);
    std::string ssid = trim_copy(form_value(body, "ssid"));
    std::string password = form_value(body, "password");

    if (ssid.empty()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing ssid");
        return ESP_OK;
    }

    if (ssid.size() > MAX_WIFI_SSID_CHARS || password.size() > MAX_WIFI_PASSWORD_CHARS) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "wifi credentials are too long");
        return ESP_OK;
    }

    if (!connect_wifi(ssid, password)) {
        start_setup_ap();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "wifi connection failed");
        return ESP_OK;
    }

    if (!save_wifi_credentials(ssid, password)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "connected, but failed to save credentials");
        return ESP_OK;
    }

    xTaskCreate(disable_setup_ap_later_task, "disable_setup_ap", 3072, nullptr, 4, nullptr);
    std::string response = "{\"ok\":true,\"url\":\"http://";
    response += HOSTNAME;
    response += ".local\"}";
    send_response(req, "application/json", response);
    return ESP_OK;
}

esp_err_t index_handler(httpd_req_t *req)
{
    static const char *page = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP Buzzer</title>
<style>
:root{color-scheme:light;--bg:#f4f6f5;--panel:#fff;--ink:#17201b;--muted:#66736d;--line:#d8e0dc;--go:#126a4a;--stop:#a43d35;--blue:#245ea8}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:Arial,sans-serif}main{max-width:720px;margin:auto;padding:16px}
h1{font-size:1.35rem;margin:0 0 14px}h2{font-size:1rem;margin:0 0 12px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;margin-bottom:12px}
label{display:grid;gap:8px;font-weight:700;margin-top:10px}textarea,input,select{width:100%;border:1px solid #b8c4bf;border-radius:6px;padding:10px;font:15px Arial,sans-serif;color:var(--ink);min-height:42px}textarea{min-height:180px;resize:vertical;font-family:ui-monospace,Menlo,monospace}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}button{border:0;border-radius:6px;min-height:42px;padding:10px 16px;font:inherit;color:white;background:var(--go);cursor:pointer}.secondary{background:var(--blue)}.danger{background:var(--stop)}
.status{min-height:24px;margin-top:12px;color:var(--muted)}.hint{color:var(--muted);font-size:.9rem;margin:8px 0 0}@media(max-width:560px){main{padding:10px}button{flex:1 1 120px}}
</style>
</head>
<body>
<main>
<h1>ESP Buzzer</h1>
<section class="panel">
<h2>WiFi</h2>
<div class="status" id="wifiStatus"></div>
<label>Network
<select id="ssidList" onchange="document.getElementById('ssid').value=this.value"></select>
</label>
<label>SSID
<input id="ssid" autocomplete="off">
</label>
<label>Password
<input id="password" type="password" autocomplete="current-password">
</label>
<div class="row">
<button class="secondary" onclick="scanWifi()">Scan</button>
<button onclick="connectWifi()">Connect</button>
</div>
<p class="hint" id="wifiHint"></p>
</section>
<section class="panel">
<h2>Sound</h2>
<label>Output
<select id="target">
<option value="buzzer">Buzzer / DRV8833</option>
<option value="motor">HDD motor / 3 phase</option>
</select>
</label>
<label>Sequence
<textarea id="sequence" spellcheck="false" placeholder="880,120 0,80 988,120"></textarea>
</label>
<div class="row">
<button onclick="play()">Play</button>
<button class="danger" onclick="stopTone()">Stop</button>
<button class="secondary" onclick="refresh()">Reload</button>
</div>
<div class="status" id="status"></div>
</section>
</main>
<script>
const $=id=>document.getElementById(id);
async function api(path,options){
  const res=await fetch(path,options);
  if(!res.ok)throw new Error(await res.text());
  return res.headers.get('content-type')?.includes('json')?res.json():res.text();
}
async function refresh(){
  const state=await api('/api/state');
  $('sequence').value=state.sequence;
  $('target').value=state.output_target;
  $('status').textContent=state.playing?'Playing':'Ready';
  $('wifiStatus').textContent=state.wifi_connected?'Connected to '+state.current_ssid:'Setup AP active: '+state.ap_ssid;
  $('wifiHint').textContent=state.wifi_connected?'Open http://'+state.hostname+'.local on this WiFi network':'Join '+state.ap_ssid+' and open http://192.168.4.1 or http://'+state.hostname+'.local';
}
async function play(){
  $('status').textContent='Playing';
  const body=new URLSearchParams({target:$('target').value,sequence:$('sequence').value});
  try{await api('/api/play',{method:'POST',body});await refresh();}
  catch(e){$('status').textContent=e.message;}
}
async function stopTone(){
  try{await api('/api/stop',{method:'POST'});await refresh();}
  catch(e){$('status').textContent=e.message;}
}
async function scanWifi(){
  const list=$('ssidList');
  list.innerHTML='';
  try{
    const data=await api('/api/wifi/scan');
    data.networks.forEach(n=>{
      const opt=document.createElement('option');
      opt.value=n.ssid;
      opt.textContent=n.ssid+' ('+n.rssi+' dBm)';
      list.appendChild(opt);
    });
    if(data.networks[0])$('ssid').value=data.networks[0].ssid;
  }catch(e){$('wifiStatus').textContent='Scan failed: '+e.message;}
}
async function connectWifi(){
  $('wifiStatus').textContent='Connecting...';
  const body=new URLSearchParams({ssid:$('ssid').value,password:$('password').value});
  try{
    const result=await api('/api/wifi/connect',{method:'POST',body});
    $('wifiStatus').textContent='Connected. Use '+result.url;
    await refresh();
  }catch(e){$('wifiStatus').textContent='Connection failed: '+e.message;}
}
refresh();scanWifi();
</script>
</body>
</html>)HTML";

    send_response(req, "text/html; charset=utf-8", page);
    return ESP_OK;
}

esp_err_t state_handler(httpd_req_t *req)
{
    std::string sequence;
    std::string current_ssid;
    OutputTarget output_target = OutputTarget::Buzzer;
    bool playing = false;
    bool sta_connected = false;
    bool setup_ap_active = false;
    {
        SemaphoreGuard lock(g_state_mutex);
        sequence = g_sequence_text;
        output_target = g_output_target;
        playing = g_playing;
        current_ssid = g_current_ssid;
        sta_connected = g_sta_connected;
        setup_ap_active = g_setup_ap_active;
    }

    std::string body = "{\"playing\":";
    body += playing ? "true" : "false";
    body += ",\"sequence\":\"";
    body += json_escape(sequence);
    body += "\",\"output_target\":\"";
    body += output_target == OutputTarget::Motor ? "motor" : "buzzer";
    body += "\",\"wifi_connected\":";
    body += sta_connected ? "true" : "false";
    body += ",\"setup_ap_active\":";
    body += setup_ap_active ? "true" : "false";
    body += ",\"current_ssid\":\"";
    body += json_escape(current_ssid);
    body += "\",\"hostname\":\"";
    body += HOSTNAME;
    body += "\",\"ap_ssid\":\"";
    body += AP_SSID;
    body += "\"}";

    send_response(req, "application/json", body);
    return ESP_OK;
}

esp_err_t play_handler(httpd_req_t *req)
{
    std::string body = read_req_body(req);
    std::string sequence = form_value(body, "sequence");
    std::string target_text = lower_copy(trim_copy(form_value(body, "target")));
    if (sequence.empty() && body.find('=') == std::string::npos) sequence = url_decode(body);

    std::vector<Step> steps;
    std::string error;
    if (!parse_sequence(sequence, &steps, &error)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, error.c_str());
        return ESP_OK;
    }

    {
        SemaphoreGuard lock(g_state_mutex);
        g_sequence_text = sequence;
        g_sequence_steps = steps;
        g_output_target = target_text == "motor" ? OutputTarget::Motor : OutputTarget::Buzzer;
    }

    PlayerCommand command = {PlayerCommandType::Play};
    xQueueOverwrite(g_player_queue, &command);
    ESP_LOGI(TAG, "Playing %u steps", static_cast<unsigned>(steps.size()));
    send_response(req, "application/json", "{\"ok\":true}");
    return ESP_OK;
}

esp_err_t stop_handler(httpd_req_t *req)
{
    PlayerCommand command = {PlayerCommandType::Stop};
    xQueueOverwrite(g_player_queue, &command);
    send_response(req, "application/json", "{\"ok\":true}");
    return ESP_OK;
}

void start_http_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 6;

    ESP_ERROR_CHECK(httpd_start(&g_httpd, &config));

    httpd_uri_t routes[] = {
        {"/", HTTP_GET, index_handler, nullptr},
        {"/api/state", HTTP_GET, state_handler, nullptr},
        {"/api/play", HTTP_POST, play_handler, nullptr},
        {"/api/stop", HTTP_POST, stop_handler, nullptr},
        {"/api/wifi/scan", HTTP_GET, wifi_scan_handler, nullptr},
        {"/api/wifi/connect", HTTP_POST, wifi_connect_handler, nullptr},
    };

    for (auto &route : routes) ESP_ERROR_CHECK(httpd_register_uri_handler(g_httpd, &route));
    ESP_LOGI(TAG, "Web server ready: http://192.168.4.1 on setup AP or http://%s.local on WiFi", HOSTNAME);
}

void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }
}

} // namespace

void app_main(void)
{
    init_nvs();

    g_state_mutex = xSemaphoreCreateMutex();
    g_wifi_mutex = xSemaphoreCreateMutex();
    g_wifi_events = xEventGroupCreate();
    g_player_queue = xQueueCreate(1, sizeof(PlayerCommand));
    configASSERT(g_state_mutex);
    configASSERT(g_wifi_mutex);
    configASSERT(g_wifi_events);
    configASSERT(g_player_queue);

    std::string error;
    if (!parse_sequence(g_sequence_text, &g_sequence_steps, &error)) {
        ESP_LOGE(TAG, "Default sequence parse failed: %s", error.c_str());
        g_sequence_text = "1000,200";
        parse_sequence(g_sequence_text, &g_sequence_steps, &error);
    }

    buzzer_force_idle_gpio();
    motor_coast();
    xTaskCreate(player_task, "buzzer_player", 4096, nullptr, 5, nullptr);

    bool has_saved_wifi = load_wifi_credentials();
    init_wifi();
    start_mdns();

    if (has_saved_wifi) {
        ESP_LOGI(TAG, "Saved WiFi credentials found for '%s'", g_saved_ssid.c_str());
        if (!connect_wifi(g_saved_ssid, g_saved_password)) {
            ESP_LOGW(TAG, "Saved WiFi connection failed; starting setup AP");
            start_setup_ap();
        }
    } else {
        ESP_LOGI(TAG, "No saved WiFi credentials; starting setup AP");
        start_setup_ap();
    }

    start_http_server();
}
