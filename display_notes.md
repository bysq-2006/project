## 显示屏问题
想让显示屏显示内容时，普通 OLED/IPS200 例程没有反应，最后确认这个屏幕只能用 `E5_05_ips200pro_page` 这类 IPS200PRO 例程。

## IMU sensor note

- Actual sensor module: IMU660RB
- Verified on board by module silkscreen: "IMU660RB"
- Matching demo path:
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\Example\Motherboard_Demo\E4_imu\E4_03_imu660rb_demo`
- Matching driver files:
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\SeekFree_RT1064_Opensource_Library\libraries\zf_device\zf_device_imu660rb.c`
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\SeekFree_RT1064_Opensource_Library\libraries\zf_device\zf_device_imu660rb.h`
- Default SPI pins from driver:
  SCK/SPC = C23, MOSI/SDI = C22, MISO/SDO = C21, CS = C20
- Do not use IMU660RC for this board unless the hardware module is changed.
