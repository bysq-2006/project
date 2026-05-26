## 显示屏记录

- 想让显示屏显示内容时，普通 OLED/IPS200 例程没有反应。
- 最后确认这个屏幕需要使用 `E5_05_ips200pro_page` 这类 IPS200PRO 例程。

## IMU 传感器记录

- 实际传感器模块：IMU660RB。
- 已通过板子丝印确认：`IMU660RB`。
- 匹配的例程路径：
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\Example\Motherboard_Demo\E4_imu\E4_03_imu660rb_demo`
- 匹配的驱动文件：
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\SeekFree_RT1064_Opensource_Library\libraries\zf_device\zf_device_imu660rb.c`
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\SeekFree_RT1064_Opensource_Library\libraries\zf_device\zf_device_imu660rb.h`
- 驱动默认 SPI 引脚：
  SCK/SPC = C23，MOSI/SDI = C22，MISO/SDO = C21，CS = C20。
- 除非硬件模块更换，否则不要使用 IMU660RC 的例程或驱动。

## OpenART 摄像头记录

- 已验证可稳定工作的 OpenMV 风格摄像头初始化：
  `sensor.set_pixformat(sensor.RGB565)`
  `sensor.set_framesize(sensor.QVGA)`
  `sensor.skip_frames(time=2000)`
- 实际图像格式：RGB565，QVGA 320x240。
- 单帧原始数据大小：
  320 * 240 * 2 = 153600 字节。
- 实测耗时：
  `sensor.snapshot()` 第一帧之后约 19 ms。
  `img.bytearray()` 约 5-6 ms。
  连接 IDE 并打印调试信息时，FPS 约 22-28。
- 之前测试过 `sensor.GRAYSCALE` + `sensor.QQQVGA` + `skip_frames(time=1000)`，在当前板子/固件组合上会卡在 `sensor.skip_frames()`。
- 当前 OpenART 预览/测试脚本路径：
  `D:\bysq_D\RT1064_Library-master\RT1064_Library-master\SeekFree_RT1064_Opensource_Library\project\openart_preview\main.py`

## OpenART 文件路径与 TFLite 记录

- 在 OpenART Plus mini-MIMXRT1170 当前固件里，不能只依赖 `os.listdir()` 判断 `/sd` 是否可用。实测目录枚举可能失败，但直接按完整路径读取文件可以成功。
- 已验证可用的模型加载方式：
  `MODEL = "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite"`
  `net = tf.load(MODEL, load_to_fb=True)`
- 已验证可用的图片资源路径：
  `WALL = "/sd/img/WallBlock.bmp"`
  `img = image.Image(WALL)`
- 不要在 OpenART 脚本中使用电脑路径，例如：
  `E:\...`
  `D:\...`
  板子无法直接读取 Windows 路径。
- 不要使用以下路径，除非已经通过 `open()` 或 `tf.load()` 直接验证成功：
  `model.tflite`
  `/model.tflite`
  `/flash/model.tflite`
  `/sdcard/model.tflite`
- 使用仓库自带 YOLO/TFLite 模型的 benchmark 结果：
  静态 `WallBlock.bmp`：推理耗时约 68-69 ms，约 11 FPS。
  实时摄像头输入：采图耗时约 10-39 ms，推理耗时约 69-70 ms，在 IDE 中约 9 FPS。
- 仓库自带 YOLO 模型不是为比赛中的围墙/色块目标训练的，所以在 `WallBlock.bmp` 或实时地图画面上输出 `found: 0` 是正常现象。

## 识别方案记录

- 纯色色块可以先用 OpenMV/OpenART 的 `img.find_blobs()` 做传统颜色阈值识别。
- `find_blobs()` 会把相邻且满足同一颜色阈值的像素组成一个 blob。即使 `merge=False`，真正连在一起的同色区域仍然会被当成一个目标。
- 对相邻同色块，不能只依赖 blob 个数。需要结合尺寸拆分、网格识别，或后续训练专用 AI 模型。
- 如果训练 AI 模型，训练数据应优先使用 OpenART 摄像头实际拍摄到的画面截图，而不是只使用素材图片。实际画面包含手机屏幕亮度、反光、透视、摩尔纹、模糊和背景干扰，更接近比赛环境。
