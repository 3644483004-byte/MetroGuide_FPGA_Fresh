# MetroGuide_FPGA_Fresh

这是 Anlogic EG4S20BG256 地铁导览音画播放项目的代码仓库。

## 文件

- `MetroGuide_FPGA_Fresh_Source.zip`：包含完整工程目录结构的源代码包，含 TD 工程、RTL、约束、IP、仿真文件、工具脚本、文档和 release 文件。下载后解压即可查看各目录。
- `rtl/`：仓库内单独提供的当前播放控制、SD 读取、音频和视频帧相关 RTL 文件。
- `assets/audio_tf/1.wav` 至 `4.wav`：四段对应音频。
- `MetroGuide_FPGA_Fresh.al`：工程入口文件副本。

## 当前上传范围

本仓库没有包含 BMP 视频帧和 `V1.BIN` 至 `V4.BIN` 视频数据文件。完整音视频播放需要另外将四组生成的视频 BIN 和四段 WAV 放到 FAT32 TF 卡根目录；当前仓库提供的 WAV 可用于核对音频素材。TD 自动生成的 Runs/cache 目录未打包。

## 硬件和开发环境

- FPGA：Anlogic EG4S20BG256
- 工具：Anlogic TD 6.2.1 Engineer 6.2.168116

代码包对应 `MetroGuide_FPGA_Fresh`，不是 `FPGA大赛` 文件夹中的其他项目。
