18:28 2021/1/24

## 说明

### 一、环境配置

#### 1. 配置环境变量

设置系统环境变量，此设置为永久有效  

|变量名|变量值|
|:-:|:-|
|`BSP_ROOT`|`.`|

每次打开 env 后还会设置临时的环境变量，只对当前的 env 内部使用  

#### 2. 添加编译工具路径

**目前已经兼容 rt-thread 和 rt-smart**  

1. 配置 rt-smart 环境  

*如果仓库自带的 rt-smart 内核代码，则无需修改*  

修改 `.\rt-smart\kernel\bsp\imx6ull\proj_smart\envconfig.bat`  

  - `RTT_EXEC_PATH`，设置成编译工具所在的路径  
  - `RTT_PROJ`，设置为 `rt-smart`  

```
@set RTT_ROOT=xxx\rt-thread :: xxx 为 rt-thread 内核代码的路径，同时需要切换到 rt-smart 分支
@set BSP_ROOT=%cd%\..

@set RTT_TOOL_PATH=%cd%\..\scripts

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-smart

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-linux-musleabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\musleabi\bin
@set PATH=%RTT_EXEC_PATH%;%RTT_TOOL_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.
```

2. 配置rt-thread 环境  

修改 `.\rt-smart\kernel\bsp\imx6ull\proj_rtt\envconfig.bat`  

  - `RTT_EXEC_PATH`，设置成编译工具所在的路径  
  - `RTT_PROJ`，设置为 `rt-thread`  
  - `RTT_ROOT`，设置为 rt-thread 内核路径（可直接使用 git 上的内核版本）  

```
@set RTT_ROOT=xxx\rt-thread :: xxx 为 rt-thread 内核代码的路径
@set BSP_ROOT=%cd%\..

@set RTT_TOOL_PATH=%cd%\..\scripts

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-thread

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-none-eabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin
@set PATH=%RTT_EXEC_PATH%;%RTT_TOOL_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.
```

### 二、配置说明

#### 1. RT-Smart

可从以下方式加入 rt-smart 内核  

1. 使用 git 上的 *最新* 版本（单独 clone rt-thread 并切换分支到 `rt-smart`）  
进入 `proj_smart_git` 工程下编译  

2. 使用本仓库内的版本（内核版本为 `rt-smart-20201125`）  
进入 `proj_smart` 工程下编译  

#### 2. RT-Thread

可从以下方式加入 rt-thread 内核  

1. 使用 git 上的 *最新* 版本（单独 clone rt-thread）  
进入 `proj_rtt` 工程下编译  

### 二、支持功能

|功能项|是否支持|备注|
|:-:|:-:|:-|
|**下载方式**||
|USB 方式下载|√|使用 100ask.org 的下载软件|
||√|使用 uuu 工具下载（推荐）|
|SD 方式下载|√|需要修改正点原子的 imxdownload 工具|
|**驱动**|||
|UART|√||
|I2C|√||
|SPI|√||
|LCD|√||
|ENET|√||
|**设备**|||
|ICM20608|√||
|PCF8574x|√||
|GT9147|√||
|LAN8720|√||
|**第三方库**|||
|LittlevGL|√|Ver 7.9.1|
|LwIP|√|Ver 2.0.2|

### 三、测试

1. 在 `.\rt-smart\kernel\bsp\imx6ull\proj_xxx\` 目录下打开 env 工具  

2. 输入 `./envconfig.bat`，回车  

3. 输入 `scons` 进行编译，`scons -j8` 可以提高编译速度  
*使用中发现，极少数情况下 `scons -j8` 会出现链接失败的情况，此时可先清除工程然后使用 `scons` 命令重试一次*  

4. 如果使用 USB 方式下载，修改启动方式拨码开关到 USB 启动方式（根据板卡要求设置）  
目前提供以下两种方式  

  - 打开 100ask.org 提供的下载软件，切换到专业版界面进行下载  
  - 使用 uuu 工具下载（**推荐**）  

5. 如果使用 SD 方式下载，需要修改正点原子的 imxdownload 工具  

打开路径 `.\rt-smart\kernel\bsp\imx6ull\scripts\` 下 `imxdownload.c` 文件  
修改以下两个宏  

```
#define ADDR_ENTRY              (0x80010000)    //start address, refer to link.lds! using phys address
#define IMAGE_SIZE              (2*1024*1024)   //image size
```

编译生成可执行文件，烧写需要在 Linux 环境中进行（参考正点原子教程）  

### 四、测试用例

### 1. 配置环境

每次打开 env 软件，执行一次即可  

```
> .\envconfig.bat
config finished.
```

### 2. 编译链接

```
> scons -j8
scons: Reading SConscript files ...
scons: done reading SConscript files.

...
...

LINK rtsmart.elf
arm-linux-musleabi-objcopy -O binary rtsmart.elf output.bin
arm-linux-musleabi-size rtsmart.elf
   text    data     bss     dec     hex filename
 777848   40656  106564  925068   e1d8c rtsmart.elf
mkimage.exe -n ../../../../tools/imx/imximage.cfg.cfgtmp -T imximage -e 0x80010000 -d output.bin output.imx
Image Type:   Freescale IMX Boot Image
Image Ver:    2 (i.MX53/6/7 compatible)
Mode:         DCD
Data Size:    831488 Bytes = 812.00 KiB = 0.79 MiB
Load Address: 8000f420
Entry Point:  80010000
scons: done building targets.
```

### 3. 下载

```
> uuu.exe download.clst
uuu (Universal Update Utility) for nxp imx chips -- libuuu_1.3.136-0-g1ecc47f

Success 1    Failure 0


1:11     2/ 2 [Done                                  ] SDP: done
```

### 4. 清除工程

分别使用 `scons --clean` 和 `make clean`  

如果只想重新编译工程，仅使用 `scons --clean` 即可  
`make clean` 用于清除生成的镜像文件，如：`.bin .imx .DEBUG`  

```
> scons --clean
scons: Reading SConscript files ...
scons: done reading SConscript files.
scons: Cleaning targets ...

...
...

scons: done cleaning targets.

> make clean
clear ...
clear all processing files success.
```

### 五、注意事项

1. 链接地址为 0x80010000 ，如若需要修改，需要同时修改 imxdownload 工具  
**当前 rt-smart 和 rt-thread 的启动地址均为 0x80010000**  

2. 平台程序可不经就该完全兼容 rt-smart 和 rt-thread  

3. 平台程序使用 fsl 的 sdk 进行开发，代码较为臃肿，但目前暂无整合计划  
rt-thread 的 bsp 提供 imx6ull 的寄存器文件，如不习惯 sdk ，可简单根据 sdk 整合为寄存器操作  

4. **建议不要同时打开多个 env 软件**，保证同一时间只打开一个 env 软件，否则会产生意想不到的问题  
