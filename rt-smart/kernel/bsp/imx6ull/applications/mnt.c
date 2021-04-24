#include <rtthread.h>

#include <dfs_fs.h>

int mnt_init(void)
{
    if (dfs_mount("sd0", "/", "elm", 0, NULL) != 0)
    {
        rt_kprintf("dir %s mount failed!\n", "/");
        return -1;
    }

    rt_kprintf("file system initialization done!\n");
    return 0;
}
INIT_ENV_EXPORT(mnt_init);
