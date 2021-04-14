/*
 * IMX6ULL
 *   imx6ull download tool(refer to from ALIENTEK)
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 * 2021-01-28     Lyons        add notes throw if sd not exited
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*
 * Instructions:
 *
 * 1. edit macro ADDR_ENTRY to modify the load address(must be physical address)
 * 2. edit macro IMAGE_SIZE to tell the BootROM the size of image
 * 3. edit macro IMX_FILE_NAME to modify the created image file
 *    please not modified because downloader uses it defaultly
 * 4. command example: ./imdownload <source.bin> /dev/sdb [-256m or -512m]
 */

#define PARAM_DDR_SIZE_256M     (0)
#define PARAM_DDR_SIZE_512M     (1)

/* source bin data offset in imx file */
#define IMX_FILE_BIN_OFFSET     (3072)

#define IMX_FILE_NAME           "load.imx"

#define ADDR_ENTRY              (0x80010000)
#define ADDR_SELF               (ADDR_ENTRY - 3*1024)
#define ADDR_START              (ADDR_ENTRY - 4*1024)
#define ADDR_DCD                (ADDR_SELF + 0x2C)
#define ADDR_BOOT               (ADDR_SELF + 0x20)
#define IMAGE_SIZE              (3*1024*1024)

#define LOG_D(...)      \
do \
{ \
    printf(__VA_ARGS__); \
    printf("\r\n"); \
} while(0);

const int _k_imx6_256mb_ivt_dcd_tbl[256];
const int _k_imx6_512mb_ivt_dcd_tbl[256];

int main( int argc, char *argv[] )
{
    FILE *fp = NULL;
    unsigned char *dummy_buf = NULL;
    unsigned char cmd_buf[1024];
    int file_len;
    int ddr_size;

    LOG_D("--------------------------");
    LOG_D("imx6ul image download tool");
    LOG_D("build %s %s", __DATE__, __TIME__);
    LOG_D("--------------------------");

    switch (argc)
    {
        case 3:
            ddr_size = PARAM_DDR_SIZE_256M;
            break;

        case 4:
            if (0 == strcmp(argv[3], "-256m")) {
                ddr_size = PARAM_DDR_SIZE_256M;
                break;
            }
            if (0 == strcmp(argv[3], "-512m")) {
                ddr_size = PARAM_DDR_SIZE_512M;
                break;
            }
            break;

        default:
            LOG_D("Help Menu:");
            LOG_D("<toolname> <source_bin> <sd_device> [<-512m or -256m>]");
            return -1;
    }

    char *source_bin_name = argv[1];
    char *sd_device_name = argv[2];

    if (0 != access(sd_device_name, F_OK))
    {
        LOG_D("can not open sd device %s!", sd_device_name);
        return -1;
    }

    fp = fopen(source_bin_name, "rb");
    if (NULL == fp)
    {
        LOG_D("can not open file %s!", source_bin_name);
        return -1;
    }

    fseek(fp, 0L, SEEK_END);
    file_len = ftell(fp);
    fclose(fp);
    LOG_D("file %s size = %dBytes", source_bin_name, file_len);

    dummy_buf = calloc(1, file_len + IMX_FILE_BIN_OFFSET);
    if (NULL == dummy_buf)
    {
        LOG_D("memory malloc failed!");
        goto _main_exit;
    }

    fp = fopen(source_bin_name, "rb");
    fread(&dummy_buf[IMX_FILE_BIN_OFFSET], 1, file_len, fp);
    fclose(fp);

    /* add DCD data to head of imx file */
    switch (ddr_size)
    {
        case PARAM_DDR_SIZE_256M:
            LOG_D("DDR size: 256MB");
            memcpy(dummy_buf, _k_imx6_256mb_ivt_dcd_tbl, sizeof(_k_imx6_256mb_ivt_dcd_tbl));
            break;

        case PARAM_DDR_SIZE_512M:
            LOG_D("DDR size: 512MB");
            memcpy(dummy_buf, _k_imx6_512mb_ivt_dcd_tbl, sizeof(_k_imx6_512mb_ivt_dcd_tbl));
            break;

        default:
            break;
    }

    /* remove old imx file and create a new one */
    LOG_D("delete old %s ...", IMX_FILE_NAME);
    sprintf(cmd_buf, "rm -rf %s", IMX_FILE_NAME);
    system(cmd_buf);

    LOG_D("create new %s ...", IMX_FILE_NAME);
    sprintf(cmd_buf, "touch %s", IMX_FILE_NAME);
    system(cmd_buf);

    fp = fopen(IMX_FILE_NAME, "wb");
    if (NULL == fp)
    {
        LOG_D("can not open file %s!", IMX_FILE_NAME);
        goto _main_exit;
    }

    file_len += IMX_FILE_BIN_OFFSET;
    if (file_len != fwrite(dummy_buf, 1, file_len, fp))
    {
        LOG_D("file write failed!");
        fclose(fp);
        goto _main_exit;
    }

    fclose(fp); //save and start download!

    sprintf(cmd_buf, "sudo dd iflag=dsync oflag=dsync if=%s of=%s bs=512 seek=2", IMX_FILE_NAME, sd_device_name);
    LOG_D("download %s to %s ...", IMX_FILE_NAME, sd_device_name);
    system(cmd_buf);

_main_exit:
    if (dummy_buf)
        free(dummy_buf);

    return 0;
}

const int _k_imx6_512mb_ivt_dcd_tbl[256] =
{
    0X402000D1,ADDR_ENTRY,0X00000000,ADDR_DCD,  ADDR_BOOT, ADDR_SELF, 0X00000000,0X00000000,
    ADDR_START,IMAGE_SIZE,0X00000000,0X40E801D2,0X04E401CC,0X68400C02,0XFFFFFFFF,0X6C400C02,
    0XFFFFFFFF,0X70400C02,0XFFFFFFFF,0X74400C02,0XFFFFFFFF,0X78400C02,0XFFFFFFFF,0X7C400C02,
    0XFFFFFFFF,0X80400C02,0XFFFFFFFF,0XB4040E02,0X00000C00,0XAC040E02,0X00000000,0X7C020E02,
    0X30000000,0X50020E02,0X30000000,0X4C020E02,0X30000000,0X90040E02,0X30000000,0X88020E02,
    0X30000C00,0X70020E02,0X00000000,0X60020E02,0X30000000,0X64020E02,0X30000000,0XA0040E02,
    0X30000000,0X94040E02,0X00000200,0X80020E02,0X30000000,0X84020E02,0X30000000,0XB0040E02,
    0X00000200,0X98040E02,0X30000000,0XA4040E02,0X30000000,0X44020E02,0X30000000,0X48020E02,
    0X30000000,0X1C001B02,0X00800000,0X00081B02,0X030039A1,0X0C081B02,0X0B000300,0X3C081B02,
    0X44014801,0X48081B02,0X302C4040,0X50081B02,0X343E4040,0X1C081B02,0X33333333,0X20081B02,
    0X33333333,0X2C081B02,0X333333F3,0X30081B02,0X333333F3,0XC0081B02,0X09409400,0XB8081B02,
    0X00080000,0X04001B02,0X2D000200,0X08001B02,0X3030331B,0X0C001B02,0XF3526B67,0X10001B02,
    0X630B6DB6,0X14001B02,0XDB00FF01,0X18001B02,0X40172000,0X1C001B02,0X00800000,0X2C001B02,
    0XD2260000,0X30001B02,0X23106B00,0X40001B02,0X4F000000,0X00001B02,0X00001884,0X90081B02,
    0X00004000,0X1C001B02,0X32800002,0X1C001B02,0X33800000,0X1C001B02,0X31800400,0X1C001B02,
    0X30802015,0X1C001B02,0X40800004,0X20001B02,0X00080000,0X18081B02,0X27020000,0X04001B02,
    0X2D550200,0X04041B02,0X06100100,0X1C001B02,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000
};

const int _k_imx6_256mb_ivt_dcd_tbl[256] =
{
    0X402000D1,ADDR_ENTRY,0X00000000,ADDR_DCD,  ADDR_BOOT, ADDR_SELF, 0X00000000,0X00000000,
    ADDR_START,IMAGE_SIZE,0X00000000,0X40E801D2,0X04E401CC,0X68400C02,0XFFFFFFFF,0X6C400C02,
    0XFFFFFFFF,0X70400C02,0XFFFFFFFF,0X74400C02,0XFFFFFFFF,0X78400C02,0XFFFFFFFF,0X7C400C02,
    0XFFFFFFFF,0X80400C02,0XFFFFFFFF,0XB4040E02,0X00000C00,0XAC040E02,0X00000000,0X7C020E02,
    0X30000000,0X50020E02,0X30000000,0X4C020E02,0X30000000,0X90040E02,0X30000000,0X88020E02,
    0X30000C00,0X70020E02,0X00000000,0X60020E02,0X30000000,0X64020E02,0X30000000,0XA0040E02,
    0X30000000,0X94040E02,0X00000200,0X80020E02,0X30000000,0X84020E02,0X30000000,0XB0040E02,
    0X00000200,0X98040E02,0X30000000,0XA4040E02,0X30000000,0X44020E02,0X30000000,0X48020E02,
    0X30000000,0X1C001B02,0X00800000,0X00081B02,0X030039A1,0X0C081B02,0X04000000,0X3C081B02,
    0X3C013C01,0X48081B02,0X38324040,0X50081B02,0X28304040,0X1C081B02,0X33333333,0X20081B02,
    0X33333333,0X2C081B02,0X333333F3,0X30081B02,0X333333F3,0XC0081B02,0X09409400,0XB8081B02,
    0X00080000,0X04001B02,0X2D000200,0X08001B02,0X3030331B,0X0C001B02,0XF352433F,0X10001B02,
    0X630B6DB6,0X14001B02,0XDB00FF01,0X18001B02,0X40172000,0X1C001B02,0X00800000,0X2C001B02,
    0XD2260000,0X30001B02,0X23104300,0X40001B02,0X47000000,0X00001B02,0X00001883,0X90081B02,
    0X00004000,0X1C001B02,0X32800002,0X1C001B02,0X33800000,0X1C001B02,0X31800400,0X1C001B02,
    0X30802015,0X1C001B02,0X40800004,0X20001B02,0X00080000,0X18081B02,0X27020000,0X04001B02,
    0X2D550200,0X04041B02,0X06100100,0X1C001B02,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
    0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,0X00000000,
};