#include "pca9555.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pca9555 {

const uint8_t INPUT_REG = 0;
const uint8_t OUTPUT_REG = 2;
const uint8_t INVERT_REG = 4;
const uint8_t CONFIG_REG = 6;
const uint8_t REG_SHIFT_THRESHOLD = 8;

namespace
{
void clear_array(uint8_t (&array)[2], uint8_t val)
{
  for (auto& elem : array)
    elem = val;
}
}
static const char *const TAG = "pca9555";

void PCA9555Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PCA9555/PCA9555A...");
  // Test to see if device exists
  if (!this->read_inputs_()) {
    ESP_LOGE(TAG, "PCA9555 not available under 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  uint8_t empty_invert[2] = {0, 0};
  // No polarity inversion
  this->write_register_(INVERT_REG, empty_invert);

  // All inputs at initialization
  clear_array(this->config_mask_, 0xFF);
  // Invert mask as the part sees a 1 as an input
  this->write_register_(CONFIG_REG, this->config_mask_);

  // All ouputs low
  clear_array(this->output_mask_, 0);
  this->write_register_(OUTPUT_REG, this->output_mask_);
  // Read the inputs
  this->read_inputs_();
  ESP_LOGD(TAG, "Initialization complete. Warning: %d, Error: %d", this->status_has_warning(),
           this->status_has_error());
}
void PCA9555Component::dump_config() {
  ESP_LOGCONFIG(TAG, "PCA9555:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with PCA9555 (0x%02X) failed!", this->address_);
  }
}

bool PCA9555Component::digital_read(uint8_t pin) {
  uint8_t shift = pin/REG_SHIFT_THRESHOLD;
  pin = pin%REG_SHIFT_THRESHOLD;
  this->read_inputs_();
  return this->input_mask_[shift] & (1 << pin);
}

void PCA9555Component::digital_write(uint8_t pin, bool value) {
  uint8_t shift = pin/REG_SHIFT_THRESHOLD;
  pin = pin%REG_SHIFT_THRESHOLD;
  if (value) {
    this->output_mask_[shift] |= (1 << pin);
  } else {
    this->output_mask_[shift] &= ~(1 << pin);
  }
  this->write_register_(OUTPUT_REG, this->output_mask_);
}

void PCA9555Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t shift = pin/REG_SHIFT_THRESHOLD;
  pin = pin%REG_SHIFT_THRESHOLD;
  if (flags == gpio::FLAG_INPUT) {
    // Clear mode mask bit
    this->config_mask_[shift] |= 1 << pin;
  } else if (flags == gpio::FLAG_OUTPUT) {
    // Set mode mask bit
    this->config_mask_[shift] &= ~(1 << pin);
  }
  this->write_register_(CONFIG_REG, this->config_mask_);
}

bool PCA9555Component::read_inputs_() {
  uint8_t inputs[2];

  if (this->is_failed()) {
//    ESP_LOGD(TAG, "Device marked failed");
    return false;
  }

//  for (auto& part_value : inputs) {
    if ((this->last_error_ = this->read_register(INPUT_REG, &inputs[0], 1, true)) != esphome::i2c::ERROR_OK) {
      this->status_set_warning();
      ESP_LOGE(TAG, "read_register_(): I2C I/O error: %d", (int) this->last_error_);
      return false;
    }
    if ((this->last_error_ = this->read_register(INPUT_REG+1, &inputs[1], 1, true)) != esphome::i2c::ERROR_OK) {
      this->status_set_warning();
      ESP_LOGE(TAG, "read_register_(): I2C I/O error: %d", (int) this->last_error_);
      return false;
    }
//  }
  this->status_clear_warning();
  this->input_mask_[0] = inputs[0]; this->input_mask_[1] = inputs[1];
  return true;
}

bool PCA9555Component::write_register_(uint8_t reg, uint8_t value) {
  if ((this->last_error_ = this->write_register(reg, &value, 1, true)) != esphome::i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "write_register_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }

  this->status_clear_warning();
  return true;
}

bool PCA9555Component::write_register_(uint8_t reg, uint8_t (&value)[2]) {
  for (auto part_value : value) {
    if ((this->last_error_ = this->write_register(reg++, &part_value, 1, true)) != esphome::i2c::ERROR_OK) {
      this->status_set_warning();
      ESP_LOGE(TAG, "write_register_(): I2C I/O error: %d", (int) this->last_error_);
      return false;
    }
  }

  this->status_clear_warning();
  return true;
}

float PCA9555Component::get_setup_priority() const { return setup_priority::IO; }

void PCA9555GPIOPin::setup() { pin_mode(flags_); }
void PCA9555GPIOPin::pin_mode(gpio::Flags flags) { this->parent_->pin_mode(this->pin_, flags); }
bool PCA9555GPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void PCA9555GPIOPin::digital_write(bool value) { this->parent_->digital_write(this->pin_, value != this->inverted_); }
std::string PCA9555GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via PCA9555", pin_);
  return buffer;
}

}  // namespace pca9555
}  // namespace esphome
