# nostromo
![quad](icon.png)

***BLDC ESC firmware (GPLv3.0)***

**features:**
  * bootloader *(betaflight 4way interface uploader)*
  * PROSHOT1000
  * DSHOT600 *(new)*
  * DSHOT300 *(new)*
  * PWM *(debug only)*
  * DSHOT commands support
  * high input linearity:  *(1000 steps)*
  * complementary PWM  *(regenerative braking)*
  * autotiming  *(new)* *(speed based, on timer IRQ)*
  * 48KHz, 24KHz *(motor PWM)*
  * configurable motor Brake
  * 3D mode
  * 500K ERpm
  * KISS telemetry protocol
  * max current, max temperature *(protection)*
  * median filter *(ERpm)*
  * kalman filter *(voltage, current)*

**supported hardware:**
  * RAZOR32V2  *(new)*
  * WRAITH32V2  *(official)*
  * TYPHOON32V2  *(official)*
  * WRAITH32
  * WRAITH32MINI
  * SUCCEX50AV2
  * SUCCEXMINI40A *(new)*
  * DYS35ARIA
  * KISS24A  *(testing)*
  * FURLING45MINI  *(hardware modification)*

forked from: <br/>
<https://github.com/conuthead/f051bldc> <br/>
<https://github.com/betaflight/betaflight-esc> <br/>
