# ESP32 MQTT bridge for EasyDrv8833

This is an Arduino PlatformIO project for ESP32 boards. The ESP32 is connected to a MQTT broker over 
your local WiFi on one hand and to an EasyDrv8833 via I2C on the other. It subscribes to certain MQTT 
topics and reacts to messages on these topics as if a command was sent to the ESP32. It then forwards
these commands to the EasyDrv8833 over I2C. This makes it possible to control the driver
by just sending messages to the MQTT broker and has been my principal testing platform
when developing the EasyDrv8833.

Currently the following commands are implemented:

* easydrv/power [off/on]
* easydrv/state nstate (1 uint8 value)
* easydrv/duty12 nduty1 nduty2 (2 unit8 values)
* easydrv/pidpars Kp Ki Kd  (3 float values)
* easydrv/setmode nset1 nset2 nmode (3 integer values, nmode is unit8)
* easydrv/resetpos npos1 npos2 (2 integer values)

The program also continuously reads the encoder state of the driver over I2C and publishes
the received data to the topic `easydrv/encall` on the MQTT broker. 

The web interface plots these values as a time series using [uplot](https://github.com/leeoniya/uPlot).

## MQTT broker

You'll need access to a MQTT broker somewhere, I use a local install of `mosquitto`. If you
you want to use the web interface, the broker must have websockets enabled, you can use
a configuration file similar to the following example for the current version of `mosquitto`

    pid_file /var/run/mosquitto.pid
    port 1883
    protocol mqtt
    listener 9001
    protocol websockets
    persistence true
    persistence_location /var/lib/mosquitto/
    log_dest file /var/log/mosquitto/mosquitto.log
    include_dir /etc/mosquitto/conf.d

For the time being no authentication is used.

## Web Interface

The `web` directory contains a simple web interface to the MQTT broker. Edit `index.html` and set 
the variable `MQTTbroker` if your MQTT broker is not running on `localhost`. The open `index.html`
in your browser. You can send commands to the EasyDrv8833 by entering the desired parameter 
values in the form and then pressing the desired button. Encoder values are plotted as a 
time series.

## ESp32 boards

I am using a EzSBC board, but the code should be easy to adapt to other boards. The EzSBC board
has a RGB LED connected to pins 16,17 and 18. You have to change functions `led_on`, `led_off` and
`setup` (defines the pins) in this case.

## Environment variables

When compiling the code you have to make sure VS Code can see the following environment variables,
these are injected dynamically into the code.

    export WIFI_SSID=YourWiFiSSID
    export WIFI_PASSWORD=YourWifiPassword
    export MQTT_BROKER=MqttBrokerIP

Don't forget to adjust port settings in `platformio.ini`.

## License

This program is BSD-3 licensed, see source code.
