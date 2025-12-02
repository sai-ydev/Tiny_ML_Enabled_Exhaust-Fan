**Description**

This project demonstrates a smart exhaust fan prototyped using the [FRDM-MCXN236](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXN236) development board. It demonstrates the prototype of a low-power exhaust fan that controlled by the MCXN236 MCU based on air quality and commands received via Wi-Fi.

A detailed write-up is available from [here](https://community.element14.com/challenges-projects/design-challenges/smart-spaces-design-challenge/a/projects/PR135/tinyml-enabled-low-power-exhaust-fans-for-smart-buildings).
The main application for this prototype is

- 3D Printing Farms where operations can result in release of volatile organic compounds (VOCs)
- Bars/Meeting Rooms to eliminate odors
- Protecting vulnerable population from allergies etc.

The project consists of the following components:

- FRDM-MCXN236 MCU - The MCX N series MCU at the heart of the application
- [EMC2101 Fan Controller](https://www.adafruit.com/product/4808)
- [BME688 breakout](https://www.adafruit.com/product/5046)
- [WizFi 360 board](https://wizfi.github.io/Document/docs/wizfi360_shield)
- 5V DC Motor Fan

The system overview is shown in the image below:

<img width="736" height="468" alt="image" src="https://github.com/user-attachments/assets/0ba2a84b-a2ee-435e-9cea-d642db22cfd5" />

The image below shows a picture of the setup:

  ![Setup Picture](img/setup.jpg)
