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

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-smart

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-linux-musleabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\musleabi\bin
@set PATH=%RTT_EXEC_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

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

```
LINK rtthread.elf
arm-linux-musleabi-objcopy -O binary rtthread.elf rtthread.bin
arm-linux-musleabi-size rtthread.elf
   text    data     bss     dec     hex filename
2139256   40656  106064 2285976  22e198 rtthread.elf
../../../tools/imx/mkimage.exe -n ../../../tools/imx/imximage.cfg.cfgtmp -T imximage -e 0x80010000 -d rtthread.bin rtthread.imx
Image Type:   Freescale IMX Boot Image
Image Ver:    2 (i.MX53/6/7 compatible)
Mode:         DCD
Data Size:    2191360 Bytes = 2140.00 KiB = 2.09 MiB
Load Address: 8000f420
Entry Point:  80010000
scons: done building targets.
```

4. 如果使用 USB 方式下载，修改启动方式拨码开关到 USB 启动方式（根据板卡要求设置）  
打开 100ask.org 提供的下载软件，切换到专业版界面进行下载  

5. 如果使用 SD 方式下载，需要修改正点原子的 imxdownload 工具  

打开路径 `.\rt-smart\tools\imx\` 下 `imxdownload.c` 文件  
修改以下两个宏  

```
#define ADDR_ENTRY              (0x80010000)    //start address, refer to link.lds! using phys address
#define IMAGE_SIZE              (2*1024*1024)   //image size
```

编译生成可执行文件，烧写需要在 Linux 环境中进行（参考正点原子教程）  

### 四、注意事项

1. 链接地址为 0x80010000 ，如若需要修改，需要同时修改 imxdownload 工具  

2. 程序可不经修改完全兼容 RT-Thread ，需要修改链接选项  
**修改 `envconfig.bat 文件即可`**  
