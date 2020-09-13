# nostromo
![quad](icon.png)

***BLDC ESC firmware (GPLv3.0)***

**features:**
  * PROSHOT1000 input protocol *(PWM for thrust tests only)*
  * DSHOT commands support
  * high input linearity:  *(1000 steps)*
  * complementary PWM  *(regenerative braking)*
  * configurable motor Brake
  * KISS telemetry protocol
  * max current, max temperature *(protection)*
  * 3D mode
  * kalman filter *(voltage, current)*

**new:**
  * bootloader *(betaflight 4way interface uploader)*
  * 600K ERpm
  * 48KHz, 24KHz
  * median filter *(ERpm)*

**supported hardware:**
  * WRAITH32V2  *(official)*
  * TYPHOON32V2  *(official)*
  * WRAITH32
  * WRAITH32MINI
  * SUCCEX50AV2
  * SUCCEXMINI40A
  * DYS35ARIA
  * KISS24A  *(testing)*
  * FURLING45MINI  *(hardware modification)*

forked from: <br/>
<https://github.com/conuthead/f051bldc> <br/>
<https://github.com/betaflight/betaflight-esc> <br/>
