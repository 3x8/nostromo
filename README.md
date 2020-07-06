# nostromo
![quad](icon.png)

***BLDC ESC firmware (GPLv3.0)***

**features:**
  * PROSHOT1000 input protocol *(PWM for thrust tests only)*
  * DSHOT commands support
  * High input linearity:  *(1000 steps)*
  * Complementary PWM  *(regenerative braking)*
  * Configurable motor Brake
  * KISS telemetry protocol
  * Maximal Current,Temperature protection
  * 3D mode
  * kalman filter *(Voltage, Current)*

**new:**
  * bootloader enabled (4way interface uploader)
  * 500K ERPM
  * 48KHz, 24KHz
  * median filter *(ERPM)*

**supported hardware:**
  * WRAITH32V2  *(official)*
  * TYPHOON32V2  *(official)*
  * WRAITH32
  * WRAITH32MINI
  * SUCCEX50AV2
  * DYS35ARIA
  * KISS24A  *(testing)*
  * FURLING45MINI  *(hardware modification)*

forked from: <br/>
<https://github.com/conuthead/f051bldc> <br/>
<https://github.com/betaflight/betaflight-esc> <br/>
