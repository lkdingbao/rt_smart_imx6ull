18:28 2021/1/24

## 说明

### 一、环境配置

#### 1. 配置环境变量

|变量名|变量值|
|:-:|:-|
|`BSP_ROOT`|`.`|

#### 2. 添加编译工具路径

修改 `.\rt-smart\kernel\bsp\imx6ull\envconfig.bat` 文件中的 `RTT_EXEC_PATH`  
值设置成编译工具所在的路径  

同时设置 `RTT_PROJ` 的值为 `rt-smart`  

```
@set RTT_ROOT=%cd%\..\..\..\.\kernel
@set RTT_TOOL_PATH=%cd%\..\..\..\.\tools\imx

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-smart

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-linux-musleabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\musleabi\bin
@set PATH=%RTT_EXEC_PATH%;%RTT_TOOL_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.
```

### 二、源码修改

源码基于官网上的 `rt-smart-20201125` 版本  
对以下两处进行修改  

1. `.\rt-smart\kernel\libcpu\arm\cortex-a\mmu.h` 文件第 45 行添加  

```c
#define NORMAL_WT_MEM  (SHARED|AP_RW|DOMAIN0|MEMWT|DESC_SEC)
```

2. `.\rt-smart\kernel\include\rthw.h` 文件第 152 行添加  

```c
void rt_hw_ms_delay(rt_uint32_t ms);
```

3. `.\rt-smart\kernel\components\drivers\spi\spi_core.c` 文件第 56 行添加  

```c
device->bus->owner = device;
```

4. `.\rt-smart\kernel\components\drivers\include\drivers\i2c.h`  文件第 26 行添加  

```c
#define RT_I2C_REG_ADDR_8BIT    (0u << 8)
#define RT_I2C_REG_ADDR_16BIT   (1u << 8)
```

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

1. 在 `.\rt-smart\kernel\bsp\imx6ull\` 目录下打开 env 工具  

2. 输入 `./envconfig.bat`，回车  

3. 输入 `scons` 进行编译，`scons -j8` 可以提高编译速度  

4. 如果使用 USB 方式下载，修改启动方式拨码开关到 USB 启动方式（根据板卡要求设置）  
目前提供以下两种方式  

  - 打开 100ask.org 提供的下载软件，切换到专业版界面进行下载  
  - 使用 uuu 工具下载（推荐）  

5. 如果使用 SD 方式下载，需要修改正点原子的 imxdownload 工具  

打开路径 `.\rt-smart\tools\imx\` 下 `imxdownload.c` 文件  
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
mkimage.exe -n ../../../tools/imx/imximage.cfg.cfgtmp -T imximage -e 0x80010000 -d output.bin output.imx
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

2. 程序可不经修改完全兼容 RT-Thread ，需要修改链接选项  
**修改 `envconfig.bat 文件即可`**  
