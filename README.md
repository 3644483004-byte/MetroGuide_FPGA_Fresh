# 一程安心 FPGA 项目

## 项目与交付版本

- 目标器件：Anlogic EG4S20BG256。
- 工程：`MetroGuide_FPGA_Fresh.al`。
- 开发工具：Anlogic TD 6.2.1 Engineer 6.2.168116。
- 当前交付 BIT：`release\MetroGuide_BoardClockTimestamp_20261029_1330.bit`。
- 当前 TF 卡素材包：`assets\card_files_10fps`。

本版本已完成 TD 综合、布局布线和 BitGen。最终 setup WNS 为 +116 ps，hold WNS 为 +3 ps，TNS 均为 0；引脚约束未修改。构建通过不等于已完成本次交付 BIT 的实体板复测，现场仍应检查 HDMI、音频及按键响应。

SHA-256：`D707886F08146300B48B4794C5661E0C924B7C13DC475EDAB8FF03406C2DC840`

## 下载到 FPGA

在 BitWriter 中选择上述 `.bit` 文件，器件选择 EG4S20BG256，使用 `PROGRAM_SRAM` 下载。这个文件负责 FPGA 配置；视频和音频素材还要另存到 TF 卡。

## TF 卡素材

TF 卡格式为 FAT32。将 `assets\card_files_10fps` **里面的文件直接复制到 TF 卡根目录**，不要把整个文件夹套在 TF 卡的子目录中。应有：

- `1.wav`、`2.wav`、`3.wav`、`4.wav`
- `V1.BIN`、`V2.BIN`、`V3.BIN`、`V4.BIN`
- `VIDEO.JSON`（素材清单，便于核对）

当前视频数据为 640×480、RGB565 little-endian、10 帧/秒；四段帧数分别为 523、376、523、503。音频为 48 kHz、双声道、16 位 PCM WAV。`assets\frames` 中的 BMP 是源帧，FPGA 从卡上读取转换后的 `V1.BIN`～`V4.BIN`；不要把 BMP 单张复制到卡上代替 BIN。

复制完成后安全弹出 TF 卡，再插入开发板。

## 按键与拨码开关

| 控件 | 功能 |
|---|---|
| KEY1 | 开/关自动连播；开启后当前段结束播放下一段，第 4 段后回到第 1 段 |
| KEY2 | 立即切到下一段并从头播放 |
| KEY3 | 立即切到上一段并从头播放 |
| KEY4 | 播放/暂停；暂停后从当前位置继续，水印随暂停冻结 |
| SW1～SW4 | 直接从头播放第 1～4 段 |

KEY 为低电平触发。SW1～SW4 按约束的有效电平触发；重复触发同一 SW 时，先拨回再拨到触发位置。

## 画面叠加与当前限制

画面显示当前视频、连播状态、播放进度、时间和动态音量条。赛事水印大小为 144×50 像素，按约 8 像素/秒水平、4 像素/秒垂直移动；KEY4 暂停时冻结，继续播放后从原位置恢复。暂停状态还显示播放/暂停图标。

日期与北京时间时间戳由板载 50 MHz 时钟输入（R7）和 25 MHz 像素时钟计数产生。源码启动初值为 2026-10-29 13:30:00；断电后会回到该初值，不保留走时。需要校准时修改 rtl/top.v 中 START_YEAR、START_MONTH、START_DAY、START_HOUR、START_MINUTE、START_SECOND 并重新编译。

## 工程目录

- `rtl`：当前工程 RTL 源码。
- `ip`：HDMI、PLL 等工程 IP 源码。
- `constraints`：引脚约束 `pin.adc` 与时序约束 `timing.sdc`。
- `assets\card_files_10fps`：当前 TF 卡交付素材包。
- `assets\audio_source`、`assets\audio_tf`：音频源文件和规范化音频。
- `assets\frames`：原始 BMP 帧，供后续重新生成 BIN。
- `sim`：仿真相关文件；`tools`：音频处理、10 fps 视频素材生成和运行工程辅助脚本。
- `docs\reference\HDMI_I2S_example.al`：HDMI/I²S 参考工程文件，不属于本工程的活动编译源；`src` 为随项目保留的参考/辅助源文件。
- `release`：当前下载文件由 `CURRENT_DOWNLOAD.txt` 指定；旧版 BIT 与说明保留，避免覆盖回退版本。

## 引脚约束摘要

最终引脚以 `constraints\pin.adc` 为准：50 MHz 时钟 R7；KEY1～KEY4 为 A2/B2/B1/C1；SW1～SW4 为 C8/C7/C6/C5；HDMI DDC 为 P2/R2，HDMI 时钟及数据为 C3/G5/F1/E1；TF 卡 CLK/CMD/DAT0～DAT3 为 A14/A13/B14/B15/A11/A12。

请勿仅凭本摘要修改管脚；需要更改时以开发板资料和约束文件为准。

