# Why esp8266Drone-RTOS-SDK?
1. Run multiple tasks independently
2. Faster
3. Learn more about SoCs technology

# The Components
1. The PID component from https://github.com/ussserrr/pid-controller-server.git
2. The MPU6050 component from http://www.i2cdevlib.com
3. The tcp_ip adapter  component from the ESP8266 SDK (included by default in the project Makefile). We are goint to set a http server (or use mqtt/modbus, still looking for the best solution)

# Environment Setup
1. Follow the https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/ programming guide.
2. Clone the this repository into ../esp directory
3. On the terminal(esp path)type make menuconfig (You can also change/set the toolchain path here, but it is recommended to do this on the bashrc profile)
4. On the same terminal: type make flash monitor
