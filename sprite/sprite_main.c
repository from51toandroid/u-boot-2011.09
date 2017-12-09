/*
 * (C) Copyright 2007-2013
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Jerry Wang <wangflord@allwinnertech.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <config.h>
#include <common.h>
#include <sunxi_mbr.h>
#include <sys_config.h>
#include "sprite_card.h"
#include "sprite_download.h"
#include "sprite_erase.h"



#if 0
static void __dump_dlmap(sunxi_download_info *dl_info)
{
	dl_one_part_info		*part_info;
	u32 i;
	char buffer[32];

	printf("*************DOWNLOAD MAP DUMP************\n");
	printf("total download part %d\n", dl_info->download_count);
	printf("\n");
	for(part_info = dl_info->one_part_info, i=0;i<dl_info->download_count;i++, part_info++)
	{
		memset(buffer, 0, 32);
		memcpy(buffer, part_info->name, 16);
		printf("download part[%d] name          :%s\n", i, buffer);
		memset(buffer, 0, 32);
		memcpy(buffer, part_info->dl_filename, 16);
		printf("download part[%d] download file :%s\n", i, buffer);
		memset(buffer, 0, 32);
		memcpy(buffer, part_info->vf_filename, 16);
		printf("download part[%d] verify file   :%s\n", i, buffer);
		printf("download part[%d] lenlo         :0x%x\n", i, part_info->lenlo);
		printf("download part[%d] addrlo        :0x%x\n", i, part_info->addrlo);
		printf("download part[%d] encrypt       :0x%x\n", i, part_info->encrypt);
		printf("download part[%d] verify        :0x%x\n", i, part_info->verify);
		printf("\n");
	}
}
#endif



#if 0
static void __dump_mbr(sunxi_mbr_t *mbr_info)
{
	sunxi_partition  		*part_info;
	u32 i;
	char buffer[32];
    
	printf("*************MBR DUMP***************\n");
	printf("total mbr part %d\n", mbr_info->PartCount);
	printf("\n");
	for(part_info = mbr_info->array, i=0;i<mbr_info->PartCount;i++, part_info++)
	{
		memset(buffer, 0, 32);
		memcpy(buffer, part_info->name, 16);
		printf("part[%d] name      :%s\n", i, buffer);
		memset(buffer, 0, 32);
		memcpy(buffer, part_info->classname, 16);
		printf("part[%d] classname :%s\n", i, buffer);
		printf("part[%d] addrlo    :0x%x\n", i, part_info->addrlo);
		printf("part[%d] lenlo     :0x%x\n", i, part_info->lenlo);
		printf("part[%d] user_type :0x%x\n", i, part_info->user_type);
		printf("part[%d] keydata   :0x%x\n", i, part_info->keydata);
		printf("part[%d] ro        :0x%x\n", i, part_info->ro);
		printf("\n");
	}
}
#endif

/*
************************************************************************************************************
*
*                                             function
*
*    name          :
*
*    parmeters     :	workmode: 升级模式选择：0，卡上固件形式升级；1，文件形式升级
*
*						name    : 文件升级时的名词
*
*    return        :
*
*    note          :
*
*
************************************************************************************************************
*/


DECLARE_GLOBAL_DATA_PTR;



extern void UART_open( __s32 uart_port, void  *uart_ctrl, __u32 apb_freq );
extern void UART_printf2( const char * str, ...);

#define my_msg(fmt,args...)				UART_printf2(fmt,##args)










#define Ser_Printf   printf
#define macdbg_prser printf

int macdbg_dmphex(const char* buff, int len)
{
    int retval = 0; 
    int x, y, tot, lineoff;
    const char* curr;
    
    Ser_Printf("buff = 0x%p.\r\n", buff );
    lineoff = 0;
    curr = buff;
    tot = 0;
    for( x = 0; x+16 < len; ){   
         Ser_Printf("%x\t", lineoff);
         for( y = 0; y < 16; y++ ){
              macdbg_prser("%02x ", (unsigned char)*(curr + y));
         }
         macdbg_prser("  ");
         for( y = 0; y < 16; y++ ){
              char c;
              c = *(curr + y);
              if( c > 31 && c < 127 ){
                  macdbg_prser("%c", c);
              }else{
                  macdbg_prser("%c", '.');
              }
              tot++;
         }
         curr += 16;
         x += 16;
         lineoff+=16;
         macdbg_prser("\r\n");
    }
    
    //do last line

	//Ser_Printf("tot %d.\r\n", tot );
	//Ser_Printf("len %d.\r\n", len );
    if( tot < len ){
        curr = (buff + tot);
        macdbg_prser("%x\t", lineoff);
        for( y = 0; y < (len - tot); y++ ){
             macdbg_prser("%02x ", (unsigned char)*(curr + y));
        }
        //padding with spaces
        //Ser_Printf("(len - tot) %d.\r\n", (len - tot) );
        if( (len - tot) < 16 ){
            for( y = 0; y < (32 - ((len - tot)*2)); y++ ){
                 macdbg_prser(" ");
            }
        }
        for( y = 0; y < 16-(len - tot); y++ ){
             macdbg_prser(" ");
        }

	   
        macdbg_prser("  "); 
	   //Ser_Printf("(len - tot) %d.\r\n", (len - tot) );
        for( y = 0; y < (len - tot); y++ ){
            char c;
            c = *(curr + y);
            if( c >31 && c < 127 ){
                macdbg_prser("%c", c);
            }else{
                macdbg_prser("%c", '.');
			  //macdbg_prser("%c", c);
            }
        }
    }
    macdbg_prser("\r\n");	
    return retval;
}






int sunxi_card_sprite_main(int workmode, char *name)
{     
    int production_media;					
    //升级介质
    uchar img_mbr[1024 * 1024]; 
    //mbr
    sunxi_download_info dl_map;			
    //dlinfo
    int sprite_next_work;
                     	
    printf("\n\n\nsunxi sprite begin\n");  
        
        
          
       
    production_media = uboot_spare_head.boot_data.storage_type;
    //获取当前的量产介质是nand或者卡
    printf( "production_media = %d\n", production_media );  
    sprite_cartoon_create();
    //启动动画显示
    sprite_uichar_printf( "please waiting...\n" );
    if( sprite_card_firmware_probe(name) ){
        //检查固件合法性

        sprite_uichar_printf( "burn firmware failed!!!\n" );
        printf("sunxi sprite firmware probe fail\n");
        return -1;
    }
    
    //sprite_uichar_printf( "production_media = %x.\n", production_media );
    //sprite_uichar_printf( "uboot_spare_head.boot_data.work_mode = %x.\n", uboot_spare_head.boot_data.work_mode );
    //sprite_uichar_printf( "uboot_spare_head.boot_data.uart_port = %x.\n", uboot_spare_head.boot_data.uart_port );
    #if 0
    sprite_uichar_printf( "1port 0-7 = %x %x %x %x %x %x %x %x.\n",
    uboot_spare_head.boot_data.uart_gpio[0].port,
    uboot_spare_head.boot_data.uart_gpio[0].port_num,
    uboot_spare_head.boot_data.uart_gpio[0].mul_sel,
    uboot_spare_head.boot_data.uart_gpio[0].pull,
    uboot_spare_head.boot_data.uart_gpio[0].drv_level,
    uboot_spare_head.boot_data.uart_gpio[0].data,
    uboot_spare_head.boot_data.uart_gpio[0].reserved[0],
    uboot_spare_head.boot_data.uart_gpio[0].reserved[1] );
    sprite_uichar_printf( "2port 0-7 = %x %x %x %x %x %x %x %x.\n",
    uboot_spare_head.boot_data.uart_gpio[1].port,
    uboot_spare_head.boot_data.uart_gpio[1].port_num,
    uboot_spare_head.boot_data.uart_gpio[1].mul_sel,
    uboot_spare_head.boot_data.uart_gpio[1].pull,
    uboot_spare_head.boot_data.uart_gpio[1].drv_level,
    uboot_spare_head.boot_data.uart_gpio[1].data,
    uboot_spare_head.boot_data.uart_gpio[1].reserved[0],
    uboot_spare_head.boot_data.uart_gpio[1].reserved[1] );
    uboot_spare_head.boot_data.uart_port = 2;
    uboot_spare_head.boot_data.uart_gpio[0].port = 2;
    uboot_spare_head.boot_data.uart_gpio[0].port_num = 0;
    uboot_spare_head.boot_data.uart_gpio[0].mul_sel = 2;                       
    uboot_spare_head.boot_data.uart_gpio[1].port = 2;
    uboot_spare_head.boot_data.uart_gpio[1].port_num = 1; 
    uboot_spare_head.boot_data.uart_gpio[1].mul_sel = 2;  	
    UART_open( uboot_spare_head.boot_data.uart_port, (void *)&uboot_spare_head.boot_data.uart_gpio[0], 24*1000*1000 );      	
    sprite_uichar_printf( "production_media = %x.\n", production_media );
    sprite_uichar_printf( "production_media = %x.\n", production_media );
    my_msg("hello uart2\r\n");
    #endif
     
    sprite_cartoon_upgrade(5);
    //显示进度                                      
    //tick_printf( "firmware probe ok\n" );           
    //获取dl_map文件,用于指引下载的数据
    //tick_printf( "fetch download map\n" );
    
    if( sprite_card_fetch_download_map(&dl_map) ){
    	printf("sunxi sprite error : fetch download map error\n");
    	return -1;
    }
                                                       
	//__dump_dlmap(&dl_map);                    
    //获取mbr                                     
    //tick_printf("fetch mbr\n");                      
	if( sprite_card_fetch_mbr(&img_mbr) ){                     
    	printf("sunxi sprite error : fetch mbr error\n");             
    	return -1;
    }
	//__dump_mbr((sunxi_mbr_t *)img_mbr);
    //根据mbr，决定擦除时候是否要保留数据
    //tick_printf("begin to erase flash\n");
    nand_get_mbr((char *)img_mbr, 16 * 1024);
    
	if( sunxi_sprite_erase_flash(img_mbr) ){
		printf("sunxi sprite error: erase flash err\n");
		return -1;
	}
	//tick_printf("successed in erasing flash\n");

	
	if( sunxi_sprite_download_mbr(img_mbr, sizeof(sunxi_mbr_t) * SUNXI_MBR_COPY_NUM) ){
		printf("sunxi sprite error: download mbr err\n");
		return -1;
	}
    sprite_cartoon_upgrade(10);
    //tick_printf("begin to download part\n");
    //开始烧写分区
    if( sunxi_sprite_deal_part(&dl_map) ){
    	printf("sunxi sprite error : download part error\n");
    	return -1;
    }
    //tick_printf("successed in downloading part\n");
	sprite_cartoon_upgrade(80);
	sunxi_sprite_exit(1);

    if( sunxi_sprite_deal_uboot(production_media) ){
    	printf("sunxi sprite error : download uboot error\n");
    	return -1;
    }
    //tick_printf("successed in downloading uboot\n");
    sprite_cartoon_upgrade(90);
    if( sunxi_sprite_deal_boot0(production_media) ){
    	printf("sunxi sprite error : download boot0 error\n");
    	return -1;
    }
    //tick_printf("successed in downloading boot0\n");
    sprite_cartoon_upgrade(100);
		
    sprite_uichar_printf("CARD OK!!!\n");
	//烧写结束

	while(1);
    
    
	
	__msdelay(3000);
	//处理烧写完成后的动作
	if( script_parser_fetch("card_boot", "next_work", &sprite_next_work, 1) ){
		sprite_next_work = SUNXI_UPDATE_NEXT_ACTION_SHUTDOWN;
	}
	sunxi_update_subsequent_processing(sprite_next_work);

	return 0;
}

