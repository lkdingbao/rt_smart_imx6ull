12:06 2021/1/24

## README.md

|FileName|Note|Others|
|:-|:-|:-|
|`mkimage.exe`|create image file|using in Windows|
|`mkimage`|create image file|using in Linux|
|`imximage.cfg.cfgtmp`|DCD data|used by `mkimage.exe` or `mkimage`|
|`msys-2.0.dll`|dll file||
|`imxdownload.c`|downloader source code||

### 1. How to use 'mkimage.exe'

1. Move this Folder to `../tools/`  
2. Edit `rtconfig.py` and add following contents at file end(`POST_ACTION` need modified)  

```
MKIMAGE_PATH = "../../../tools/imx/"
MKIMAGE = MKIMAGE_PATH + 'mkimage.exe'
MKIMAGE_CFG_FILE = MKIMAGE_PATH + 'imximage.cfg.cfgtmp'

POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin \n' + \
              SIZE + ' $TARGET \n' + \
              MKIMAGE + " -n " + MKIMAGE_CFG_FILE + ' -T imximage -e 0x80010000 -d rtthread.bin rtthread.imx \n'
```

### 2. How to use 'imxdownload.c'

If you use ALIENTEK NAND board and 100ask downloader, you can use this file to download to SD card  

1. Open `imxdownload.c` to modify params  
2. Use `gcc imxdownload.c -o imxdownload` in Linux environment  
4. Use `./imxdownload <source_bin> <sd_device> [-256m or -512m]` to download  

### 3. File 'msys-2.0.dll'

This file is not necessary  
But if you get following error, using this file to repair it  

```
---------------------------
mkimage.exe - 系统错误
---------------------------
由于找不到 msys-2.0.dll，无法继续执行代码。重新安装程序可能会解决此问题。 
---------------------------
确定   
---------------------------
```
