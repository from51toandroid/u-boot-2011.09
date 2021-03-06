//Copyright 2007-2013
//Allwinner Technology Co., Ltd. <www.allwinnertech.com>
//Jerry Wang <wangflord@allwinnertech.com>
 
#include <config.h>
#include <common.h>
#include <private_boot0.h>
#include <private_uboot.h>
#include <sunxi_mbr.h>
#include "sprite_verify.h"
#include "sprite_card.h"
#include <sunxi_nand.h>






int sunxi_sprite_download_mbr(void *buffer, uint buffer_size)
{
	int ret;

	if(buffer_size != (SUNXI_MBR_SIZE * SUNXI_MBR_COPY_NUM))
	{
		printf("the mbr size is bad\n");

		return -1;
	}
	if(sunxi_sprite_init(0))
	{
		printf("sunxi sprite init fail when downlaod mbr\n");

		return -1;
	}
	if(sunxi_sprite_write(0, buffer_size/512, buffer) == (buffer_size/512))
	{
		debug("mbr write ok\n");

		ret = 0;
	}
	else
	{
		debug("mbr write fail\n");

		ret = -1;
	}
	if( uboot_spare_head.boot_data.storage_type == 2 ){
		//printf("begin to write standard mbr\n");
		if( card_download_standard_mbr(buffer) ){
			printf("write standard mbr err\n");
			return -1;
		}
		//printf("successed to write standard mbr\n");
	}

	if( sunxi_sprite_exit(0) ){
		printf("sunxi sprite exit fail when downlaod mbr\n");
		return -1;
	}
	return ret;
}




//烧录uboot  
int sunxi_sprite_download_uboot(void *buffer, int production_media, int mode)
{    
    struct spare_boot_head_t *uboot = (struct spare_boot_head_t *)buffer;
    printf( "start to shao uboot!!!\n" );
    //校验特征字符是否正确
    printf("magic = %s\n", uboot->boot_head.magic);
    if( strncmp((const char *)uboot->boot_head.magic, UBOOT_MAGIC, MAGIC_SIZE) ){
        printf("sunxi sprite: uboot magic is error\n");
        return -1;
    }
    //校验数据是否正确
    if( !mode ){
        if( sunxi_sprite_verify_checksum(buffer, uboot->boot_head.length, uboot->boot_head.check_sum) ){
            printf("sunxi sprite: uboot checksum is error\n");
            return -1;
        }
        //读出dram参数
        //填充FLASH信息
        if( !production_media ){
            nand_uboot_get_flash_info((void *)uboot->boot_data.nand_spare_data, STORAGE_BUFFER_SIZE );
        }
    }
    //regenerate check sum
    uboot->boot_head.check_sum = sunxi_sprite_generate_checksum(buffer, uboot->boot_head.length, uboot->boot_head.check_sum);
    //校验数据是否正确
    if( sunxi_sprite_verify_checksum(buffer, uboot->boot_head.length, uboot->boot_head.check_sum) ){
        printf("sunxi sprite: uboot checksum is error\n");
        return -1;
    }
    printf("uboot size = 0x%x\n", uboot->boot_head.length);
    printf("storage type = %d\n", production_media);
    if( !production_media ){
        debug("nand down uboot\n");
        if( uboot_spare_head.boot_data.work_mode == WORK_MODE_BOOT ){
            printf("work_mode_boot \n");
            return nand_force_download_uboot(uboot->boot_head.length,buffer);
        }else{
            return nand_download_uboot(uboot->boot_head.length, buffer);
        }
    }else{
        printf("mmc down uboot\n");
        return card_download_uboot(uboot->boot_head.length, buffer);
    }
}









int sunxi_sprite_download_boot0(void *buffer, int production_media)
{
    boot0_file_head_t    *boot0  = (boot0_file_head_t *)buffer;

	//校验特征字符是否正确
	debug("112233 %s\n", boot0->boot_head.magic);
	if(strncmp((const char *)boot0->boot_head.magic, BOOT0_MAGIC, MAGIC_SIZE)){
		printf("sunxi sprite: boot0 magic is error\n");
		return -1;
	}
	//校验数据是否正确
	if(sunxi_sprite_verify_checksum(buffer, boot0->boot_head.length, boot0->boot_head.check_sum)){
		printf("sunxi sprite: boot0 checksum is error\n");
		return -1;
	}
	//读出dram参数
	//填充FLASH信息
	if( !production_media ){
		nand_uboot_get_flash_info((void *)boot0->prvt_head.storage_data, STORAGE_BUFFER_SIZE);
	}else{
		extern int mmc_write_info(int dev_num,void *buffer,u32 buffer_size);
		mmc_write_info(2,(void *)boot0->prvt_head.storage_data,STORAGE_BUFFER_SIZE);
	}


	#if 0
	{
		int i;
		uint *addr = (uint *)DRAM_PARA_STORE_ADDR;

		for( i=0; i<32; i++ ){
			 //printf("dram para[%d] = %x\n", i, addr[i]);
		}
	}
	#endif
	
	memcpy((void *)&boot0->prvt_head.dram_para, (void *)DRAM_PARA_STORE_ADDR, 32 * 4);
	/* regenerate check sum */
	boot0->boot_head.check_sum = sunxi_sprite_generate_checksum(buffer, boot0->boot_head.length, boot0->boot_head.check_sum);
	//校验数据是否正确
	if( sunxi_sprite_verify_checksum(buffer, boot0->boot_head.length, boot0->boot_head.check_sum) ){
		printf("sunxi sprite: boot0 checksum is error\n");
		return -1;
	}
	//printf("production_media123 = %d\n", production_media);
	if( !production_media ){
		return nand_download_boot0(boot0->boot_head.length, buffer);
	}else{
		return card_download_boot0(boot0->boot_head.length, buffer);
	}
}



