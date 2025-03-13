# Arduino Projects Repository

This repository contains various Arduino projects developed by [Amarsalim30](https://github.com/Amarsalim30).

## Table of Contents
- [Projects](#projects)
  - [1st Blinking LED](#1st-blinking-led)
  - [DevilTwin NodeMCU](#deviltwin-nodemcu)
  - [ESP8266 PhiSiFi](#esp8266-phisifi)
  - [Line Follow Car Project](#line-follow-car-project)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact Information](#contact-information)

## Projects

### 1st Blinking LED
Path: `1st_blinkingLED/1st_blinkingLED.ino`

This project demonstrates the basic functionality of blinking multiple LEDs using Arduino.

```c++
int const LED1 = 5;
int const LED2 = 6;
int const LED3 = 7;

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
}

void loop() {
  digitalWrite(LED1, HIGH);
  delay(1000);
  digitalWrite(LED1, LOW);
  delay(1000);
  digitalWrite(LED2, HIGH);
  delay(1000);
  digitalWrite(LED2, LOW);
  delay(1000);
  digitalWrite(LED3, HIGH);
  delay(1000);
  digitalWrite(LED3, LOW);
  delay(1000);
}
```

### DevilTwin NodeMCU
Path: `DevilTwin-NodeMCU/DevilTwin-NodeMCU.ino`

This project creates a Wi-Fi access point and performs network scanning using an ESP8266 NodeMCU.

```c++
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiSTA.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <DNSServer.h>
```

### ESP8266 PhiSiFi
Path: `ESP8266_PhiSiFi/ESP8266_PhiSiFi.ino`

This project sets up an ESP8266 as a Wi-Fi access point and performs HTTP requests.

```c++
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
```

### Line Follow Car Project
Path: `Line Follow Car Project/DIYLinefollow/DIYLinefollow.ino`

This project is used to make an Arduino line follower robot car.

```c++
#define in1 8
#define in2 7
#define in3 6
#define in4 5
#define enA 9
#define enB 10
#define trigger A2
#define echo A3
```

## Installation

To clone and run this repository, you'll need [Git](https://git-scm.com) installed on your computer. From your command line:

```bash
# Clone this repository
git clone https://github.com/Amarsalim30/arduino.git

# Go into the repository
cd arduino
```

## Usage

Each project has its own folder. Navigate to the respective folder and open the `.ino` file in the Arduino IDE to upload it to your Arduino board.

## Contributing

Contributions are welcome! Please follow these steps to contribute:

1. Fork the repository
2. Create a new branch (`git checkout -b feature-branch`)
3. Commit your changes (`git commit -m 'Add some feature'`)
4. Push to the branch (`git push origin feature-branch`)
5. Open a pull request

## License

This project is licensed under the MIT License.

## Contact Information

For any questions or feedback, please contact [Amarsalim30](https://github.com/Amarsalim30).
```

Feel free to customize this README further based on the specific details and requirements of your projects.
