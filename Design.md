# HumidityControl

* Wlan manager for ip config. Async possible?
 * See https://github.com/tzapu/WiFiManager/issues/723
* Measure humidity
 * if no humidity device, scan i2c for bme280 or bmp280
 * if device, read temperature, humidity, pressure if available
* Control fan via mqtt:
  * Always on
  * Always off
  * Automatic
    * Switch on fan if humidity is above maximum
    * Switch off fan if humidity is below minimum
    * In between set fan speed according to humidity
* If new temperature/hum/press/fan status, then publish to mqtt
* Device configuration via mqtt
 * Set device name (replace {nx} or {nd} with n hex or dec digits of device id)
 * Set mqtt topic template (replace {n} with device name and {t} for subtopic)
  * Fallback humi/{6x}/{t} is always valid
 * Reset wlan config and reboot to activate wlan manager
 * Set maximum humidity (turns on fan)
 * Set minimum humidity (turns off fan)
 * Set fan speed percent if always on
 * Set led brightness percent
* Device query via mqtt
 * ip
 * name
 * version
 * topic template
 * uptime
 * temperature/humidity/pressure
 * fan mode
 * fan power status
 * humidity maximum
 * humidity minimum
 * percent fan usage in automatic mode
 * led brightness percent
 * fan rpm
 * all
* Button toggles on/off/auto, multiple: Set wlan manager status, long: reboot
* ws2812 or rgb led yellow = below minimum humidity ... blue = above maximum humidity
* ws2812 or rgb led red = always off, white = always on
