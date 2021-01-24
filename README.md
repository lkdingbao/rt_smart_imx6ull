18:28 2021/1/24

## 说明

### 一、环境配置

#### 1. 配置环境变量

|变量名|变量值|
|:-:|:-|
|`BSP_ROOT`|`.`|
|`RTT_ROOT`|`盘符:\...\rt-smart\kernel`|

#### 2. 添加编译工具路径

修改`.\rt-smart\kernel\bsp\imx6ull\smart-env.bat`文件中的`RTT_EXEC_PATH`  
值设置成编译工具所在的路径  

```
@set RTT_CC=gcc
@set RTT_EXEC_PATH=D:\gnu_gcc\install_arm-linux-musleabi_for_i686-w64-mingw32\bin
@set RTT_CC_PREFIX=arm-linux-musleabi-
@set PATH=%RTT_EXEC_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%
```

### 二、源码修改

源码基于官网上的`rt-smart-20201125`版本  
对以下两处进行修改  

1. `.\rt-smart\kernel\libcpu\arm\cortex-a\mmu.h`文件第45行添加  

```c
#define NORMAL_WT_MEM  (SHARED|AP_RW|DOMAIN0|MEMWT|DESC_SEC)
```

2. `.\rt-smart\kernel\include\rthw.h`文件第152行添加  

```c
void rt_hw_ms_delay(rt_uint32_t ms);
```

3. `.\rt-smart\kernel\components\drivers\spi\spi_core.c`文件第56行添加  

```
device->bus->owner = device;
```

### 二、支持功能

|功能项|是否支持|备注|
|:-:|:-:|:-|
|**下载方式**||
|USB方式下载|√|使用100ask.org的下载软件|
|SD方式下载|√|需要修改正点原子的imxdownload工具|
|**驱动**|||
|UART|√||
|I2C|√||
|SPI|√||
|LCD|√||
|**设备**|||
|ICM20608|√||
|PCF8574x|√||

### 三、测试

1. 在`.\rt-smart\kernel\bsp\imx6ull\`目录下打开env工具  

2. 输入`smart-env.bat`，回车，配置临时环境变量  

3. 输入`scons`进行编译，`scons -j 8`可以提高编译速度  

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

4. 如果使用USB方式下载，修改启动方式拨码开关到USB启动方式（根据板卡要求设置）  
打开100ask.org提供的下载软件，切换到专业版界面进行下载  

5. 如果使用SD方式下载，需要修改正点原子的imxdownload工具  

打开路径`.\rt-smart\tools\imx\`下`imxdownload.c`文件  
修改以下两个宏  

```
#define ADDR_ENTRY              (0x80010000)    //start address, refer to link.lds! using phys address
#define IMAGE_SIZE              (2*1024*1024)   //image size
```

编译生成可执行文件，烧写需要在Linux环境中进行（参考正点原子教程）  
