/*
 * Copyright 2000-2009
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>



int do_dp_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{


	asm volatile ("nop");

    printf("test.0\n");
	printf("test.1\n");
	printf("test.2\n");
	printf("test.3\n");
	

    return 0;
}

U_BOOT_CMD(
	k518test0,	CONFIG_SYS_MAXARGS,	1,	do_dp_test,
	"do nothing123, successfully",
	NULL
);
