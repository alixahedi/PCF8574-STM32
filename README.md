# PCF8574 STM32 HAL Library

This library allows you to control the PCF8574 I/O expander using the STM32 HAL library. It supports setting and reading individual pins and controlling multiple PCF8574 modules on a single I2C bus.

## Features
- Control individual pins on the PCF8574.
- Support for multiple PCF8574 modules on the same I2C bus.
- Easily toggle, set, or read the state of each pin.
- Compatible with STM32 HAL library.

## Requirements
- STM32 microcontroller
- STM32 HAL library
- I2C communication bus

## Getting Started

### Hardware Setup
Connect the PCF8574 to the I2C bus of the STM32 microcontroller. You can connect up to 8 PCF8574 modules on the same bus by configuring the A0, A1, and A2 address pins.

### Software Setup

1. Include the `pcf8574.h` and `pcf8574.c` files in your project.
2. Make sure that the I2C peripheral is initialized in your STM32 project.

### Example Usage

Here is an example of how to initialize and use the library:

```c
#include "pcf8574.h"

// Create a handle for PCF8574
PCF8574_HandleTypeDef pcf8574;

int main(void) {
    // HAL initialization
    HAL_Init();
    
    // I2C handle (example, customize as needed)
    I2C_HandleTypeDef hi2c1;
    // Initialize your I2C peripheral (ensure this is done based on your project setup)
    // HAL_I2C_Init(&hi2c1);

    // Initialize the PCF8574 with the desired address (A0 = 0, A1 = 0, A2 = 0)
    PCF8574_Init(&pcf8574, &hi2c1, 0, 0, 0);

    // Set Pin 0 to HIGH (1)
    PCF8574_WritePin(&pcf8574, 0, GPIO_PIN_SET);

    // Read the state of Pin 0
    GPIO_PinState state = PCF8574_ReadPin(&pcf8574, 0);

    while (1) {
        // Toggle Pin 0
        PCF8574_TogglePin(&pcf8574, 0);

        // Delay for 500 ms
        HAL_Delay(500);
    }
}
```
## 📞 Contact
For questions or suggestions, feel free to reach out via email:
📧 Alixahedi@gmail.com

<br/>
<a href="https://twitter.com/alixahedi">
<img align="left" alt="Alix | Twitter" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/Twitter.png" />
</a>
<a href="https://www.linkedin.com/in/ali-zahedi-b5a360158//">
<img align="left" alt="Ali's LinkedIN" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/Linkedin.png" />
</a>
<a href="https://www.facebook.com/Alixahedi/">
<img align="left" alt="Ali's FaceBook" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/fb.png" />
</a>
<a href="https://www.instagram.com/Alixahedi">
<img align="left" alt="Ali's Instagram" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/insta.png" />
</a>
<br/>
