/*
 * (C) Copyright 2000
 * Rob Taylor, Flying Pig Systems. robt@flyingpig.com.
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
#include <linux/compiler.h>
#ifdef CONFIG_ALLWINNER
#include <asm/arch/cpu.h>
#include <boot_type.h>
#endif
#include <ns16550.h>
#ifdef CONFIG_NS87308
#include <ns87308.h>
#endif
#ifdef CONFIG_KIRKWOOD
#include <asm/arch/kirkwood.h>
#elif defined(CONFIG_ORION5X)
#include <asm/arch/orion5x.h>
#elif defined(CONFIG_ARMADA100)
#include <asm/arch/armada100.h>
#elif defined(CONFIG_PANTHEON)
#include <asm/arch/pantheon.h>
#endif

#if defined (CONFIG_SERIAL_MULTI)
#include <serial.h>
#endif

#include <sys_config.h>

DECLARE_GLOBAL_DATA_PTR;

#if !defined(CONFIG_CONS_INDEX)
#if defined (CONFIG_SERIAL_MULTI)
/*   with CONFIG_SERIAL_MULTI we might have no console
 *  on these devices
 */
#else
#error	"No console index specified."
#endif /* CONFIG_SERIAL_MULTI */
#elif (CONFIG_CONS_INDEX < 1) || (CONFIG_CONS_INDEX > 4)
#error	"Invalid console index value."
#endif

#if CONFIG_CONS_INDEX == 1 && !defined(CONFIG_SYS_NS16550_COM1)
#error	"Console port 1 defined but not configured."
#elif CONFIG_CONS_INDEX == 2 && !defined(CONFIG_SYS_NS16550_COM2)
#error	"Console port 2 defined but not configured."
#elif CONFIG_CONS_INDEX == 3 && !defined(CONFIG_SYS_NS16550_COM3)
#error	"Console port 3 defined but not configured."
#elif CONFIG_CONS_INDEX == 4 && !defined(CONFIG_SYS_NS16550_COM4)
#error	"Console port 4 defined but not configured."
#endif

/* Note: The port number specified in the functions is 1 based.
 *	 the array is 0 based.
 */
static NS16550_t serial_ports[4] = {
#ifdef CONFIG_SYS_NS16550_COM1
	(NS16550_t)CONFIG_SYS_NS16550_COM1,
#else
	NULL,
#endif
#ifdef CONFIG_SYS_NS16550_COM2
	(NS16550_t)CONFIG_SYS_NS16550_COM2,
#else
	NULL,
#endif
#ifdef CONFIG_SYS_NS16550_COM3
	(NS16550_t)CONFIG_SYS_NS16550_COM3,
#else
	NULL,
#endif
#ifdef CONFIG_SYS_NS16550_COM4
	(NS16550_t)CONFIG_SYS_NS16550_COM4
#else
	NULL
#endif
};

#define PORT	serial_ports[port-1]
#if defined(CONFIG_CONS_INDEX)
#define CONSOLE	(serial_ports[CONFIG_CONS_INDEX-1])
#endif

#if defined(CONFIG_SERIAL_MULTI)

/* Multi serial device functions */
#define DECLARE_ESERIAL_FUNCTIONS(port) \
    int  eserial##port##_init (void) {\
	int clock_divisor; \
	clock_divisor = calc_divisor(serial_ports[port-1]); \
	NS16550_init(serial_ports[port-1], clock_divisor); \
	return(0);}\
    void eserial##port##_setbrg (void) {\
	serial_setbrg_dev(port);}\
    int  eserial##port##_getc (void) {\
	return serial_getc_dev(port);}\
    int  eserial##port##_tstc (void) {\
	return serial_tstc_dev(port);}\
    void eserial##port##_putc (const char c) {\
	serial_putc_dev(port, c);}\
    void eserial##port##_puts (const char *s) {\
	serial_puts_dev(port, s);}

/* Serial device descriptor */
#define INIT_ESERIAL_STRUCTURE(port, name) {\
	name,\
	eserial##port##_init,\
	NULL,\
	eserial##port##_setbrg,\
	eserial##port##_getc,\
	eserial##port##_tstc,\
	eserial##port##_putc,\
	eserial##port##_puts, }

#endif /* CONFIG_SERIAL_MULTI */

static int calc_divisor (NS16550_t port)
{
#ifdef CONFIG_OMAP1510
	/* If can't cleanly clock 115200 set div to 1 */
	if ((CONFIG_SYS_NS16550_CLK == 12000000) && (gd->baudrate == 115200)) {
		port->osc_12m_sel = OSC_12M_SEL;	/* enable 6.5 * divisor */
		return (1);				/* return 1 for base divisor */
	}
	port->osc_12m_sel = 0;			/* clear if previsouly set */
#endif
#ifdef CONFIG_OMAP1610
	/* If can't cleanly clock 115200 set div to 1 */
	if ((CONFIG_SYS_NS16550_CLK == 48000000) && (gd->baudrate == 115200)) {
		return (26);		/* return 26 for base divisor */
	}
#endif

#ifdef CONFIG_APTIX
#define MODE_X_DIV 13
#else
#define MODE_X_DIV 16
#endif

	/* Compute divisor value. Normally, we should simply return:
	 *   CONFIG_SYS_NS16550_CLK) / MODE_X_DIV / gd->baudrate
	 * but we need to round that value by adding 0.5.
	 * Rounding is especially important at high baud rates.
	 */
	return (CONFIG_SYS_NS16550_CLK + (gd->baudrate * (MODE_X_DIV / 2))) /
		(MODE_X_DIV * gd->baudrate);
}




#if !defined(CONFIG_SERIAL_MULTI)
int serial_init (void)
{
    int clock_divisor;
    int uart_console;
    #ifdef CONFIG_NS87308
    initialise_ns87308();
    #endif
     
    #if 0
    #ifdef CONFIG_SYS_NS16550_COM1
    clock_divisor = calc_divisor(serial_ports[0]);
    NS16550_init(serial_ports[0], clock_divisor);
    #endif
    #ifdef CONFIG_SYS_NS16550_COM2
    clock_divisor = calc_divisor(serial_ports[1]);
    NS16550_init(serial_ports[1], clock_divisor);
    #endif
    #ifdef CONFIG_SYS_NS16550_COM3
    clock_divisor = calc_divisor(serial_ports[2]);
    NS16550_init(serial_ports[2], clock_divisor);
    #endif
    #ifdef CONFIG_SYS_NS16550_COM4
    clock_divisor = calc_divisor(serial_ports[3]);
    NS16550_init(serial_ports[3], clock_divisor);
    #endif
    #else
    printf( "uart_console = %d\n", uart_console );
    uart_console = uboot_spare_head.boot_data.uart_port;
    if( (uart_console < 0) || (uart_console > 4) ){
        uart_console = 0;
    }
    //uart_console = 2;
    //gpio_request((void *)uboot_spare_head.boot_data.uart_gpio, 2);
    clock_divisor = calc_divisor(serial_ports[uart_console]);
    NS16550_init(serial_ports[uart_console], clock_divisor);
    gd->uart_console = uart_console;
    #endif
    return 0;
}
#endif




void _serial_putc( const char c,const int port )
{
	if (c == '\n')
		NS16550_putc(PORT, '\r');
	//serial_ports[port-1]

	NS16550_putc(PORT, c);
}

void
_serial_putc_raw(const char c,const int port)
{
	NS16550_putc(PORT, c);
}

void
_serial_puts (const char *s,const int port)
{
	while (*s) {
		_serial_putc (*s++,port);
	}
}


int
_serial_getc(const int port)
{
	return NS16550_getc(PORT);
}

int
_serial_tstc(const int port)
{
	return NS16550_tstc(PORT);
}

void
_serial_setbrg (const int port)
{
	int clock_divisor;

	clock_divisor = calc_divisor(PORT);
	NS16550_reinit(PORT, clock_divisor);
}

#if defined(CONFIG_SERIAL_MULTI)
static inline void
serial_putc_dev(unsigned int dev_index,const char c)
{
	_serial_putc(c,dev_index);
}
#else
void
serial_putc(const char c)
{
#ifdef CONFIG_ALLWINNER
	_serial_putc(c,gd->uart_console+1);
#else
	_serial_putc(c,CONFIG_CONS_INDEX);
#endif
}
#endif

#if defined(CONFIG_SERIAL_MULTI)
static inline void
serial_putc_raw_dev(unsigned int dev_index,const char c)
{
	_serial_putc_raw(c,dev_index);
}
#else
void
serial_putc_raw(const char c)
{
#ifdef CONFIG_ALLWINNER
	_serial_putc_raw(c,gd->uart_console+1);
#else
	_serial_putc_raw(c,CONFIG_CONS_INDEX);
#endif
}
#endif







#if defined(CONFIG_SERIAL_MULTI)
static inline void
serial_puts_dev(unsigned int dev_index,const char *s)
{
	_serial_puts(s,dev_index);
}
//error
#else
void
serial_puts(const char *s)
{
    #ifdef CONFIG_ALLWINNER
    error
    _serial_puts(s,gd->uart_console+1);
    #else
    error
	_serial_puts(s,CONFIG_CONS_INDEX);
    #endif
}
#endif










#if defined(CONFIG_SERIAL_MULTI)
static inline int serial_getc_dev(unsigned int dev_index)
{
	return _serial_getc(dev_index);
}
#else
int
serial_getc(void)
{

xcvcxvcvx
#ifdef CONFIG_ALLWINNER
//cccc
	return _serial_getc(gd->uart_console+1);
#else
	return _serial_getc(CONFIG_CONS_INDEX);
#endif
}
#endif

#if defined(CONFIG_SERIAL_MULTI)
static inline int
serial_tstc_dev(unsigned int dev_index)
{
	return _serial_tstc(dev_index);
}
#else
int
serial_tstc(void)
{
#ifdef CONFIG_ALLWINNER
	return _serial_tstc(gd->uart_console+1);
#else
	return _serial_tstc(CONFIG_CONS_INDEX);
#endif
}
#endif

#if defined(CONFIG_SERIAL_MULTI)
static inline void
serial_setbrg_dev(unsigned int dev_index)
{
	_serial_setbrg(dev_index);
}
#else
void
serial_setbrg(void)
{
#ifdef CONFIG_ALLWINNER
	_serial_setbrg(gd->uart_console+1);
#else
	_serial_setbrg(CONFIG_CONS_INDEX);
#endif
}
#endif






#if defined(CONFIG_SERIAL_MULTI)

int eserial1_init(void) 
{
    int clock_divisor; 
    clock_divisor = calc_divisor(serial_ports[1-1]); 
    NS16550_init(serial_ports[1-1], clock_divisor); 
    return(0);
}
void eserial1_setbrg (void) 
{
    serial_setbrg_dev(1);
}	
int eserial1_getc(void) 
{
    return serial_getc_dev(1);
}	
int eserial1_tstc(void)
{
    return serial_tstc_dev(1);
}
void eserial1_putc(const char c) 
{
    serial_putc_dev(1, c);
}
void eserial1_puts(const char *s) 
{
    serial_puts_dev(1, s);
}

struct serial_device eserial1_device =
{
    "eserial0",
    eserial1_init,
    NULL,
    eserial1_setbrg,
    eserial1_getc,
    eserial1_tstc,
    eserial1_putc,
    eserial1_puts, 
};


int eserial2_init(void) 
{
    int clock_divisor; 
    clock_divisor = calc_divisor(serial_ports[2-1]); 
    NS16550_init(serial_ports[2-1], clock_divisor); 
    return(0);
}
void eserial2_setbrg (void) 
{
    serial_setbrg_dev(2);
}	
int eserial2_getc(void) 
{
    return serial_getc_dev(2);
}	
int eserial2_tstc(void)
{
    return serial_tstc_dev(2);
}
void eserial2_putc(const char c) 
{
    serial_putc_dev(2, c);
}
void eserial2_puts(const char *s) 
{
    serial_puts_dev(2, s);
}

struct serial_device eserial2_device =
{
    "eserial1",
    eserial2_init,
    NULL,
    eserial2_setbrg,
    eserial2_getc,
    eserial2_tstc,
    eserial2_putc,
    eserial2_puts, 
};





//DECLARE_ESERIAL_FUNCTIONS(2);
//struct serial_device eserial2_device =
//	INIT_ESERIAL_STRUCTURE(2, "eserial1");


//DECLARE_ESERIAL_FUNCTIONS(3);
//struct serial_device eserial3_device =
//	INIT_ESERIAL_STRUCTURE(3, "eserial2");



int eserial3_init(void) 
{
    int clock_divisor; 
    clock_divisor = calc_divisor(serial_ports[3-1]); 
    NS16550_init(serial_ports[3-1], clock_divisor); 
    return(0);
}
void eserial3_setbrg (void) 
{
    serial_setbrg_dev(3);
}	
int eserial3_getc(void) 
{
    return serial_getc_dev(3);
}	
int eserial3_tstc(void)
{
    return serial_tstc_dev(3);
}
void eserial3_putc(const char c) 
{
    serial_putc_dev(3, c);
}
void eserial3_puts(const char *s) 
{
    serial_puts_dev(3, s);
}

struct serial_device eserial3_device =
{
    "eserial2",
    eserial3_init,
    NULL,
    eserial3_setbrg,
    eserial3_getc,
    eserial3_tstc,
    eserial3_putc,
    eserial3_puts, 
};









//DECLARE_ESERIAL_FUNCTIONS(4);
//struct serial_device eserial4_device =
//	INIT_ESERIAL_STRUCTURE(4, "eserial3");





int eserial4_init(void) 
{
    int clock_divisor; 
    clock_divisor = calc_divisor(serial_ports[4-1]); 
    NS16550_init(serial_ports[4-1], clock_divisor); 
    return(0);
}
void eserial4_setbrg(void) 
{
    serial_setbrg_dev(4);
}	
int eserial4_getc(void) 
{
    return serial_getc_dev(4);
}	
int eserial4_tstc(void)
{
    return serial_tstc_dev(4);
}
void eserial4_putc(const char c) 
{
    serial_putc_dev(4, c);
}
void eserial4_puts(const char *s) 
{
    serial_puts_dev(4, s);
}

struct serial_device eserial4_device =
{
    "eserial3",
    eserial4_init,
    NULL,
    eserial4_setbrg,
    eserial4_getc,
    eserial4_tstc,
    eserial4_putc,
    eserial4_puts, 
};

__weak struct serial_device *default_serial_console(void)
{
    #ifdef CONFIG_ALLWINNER
    switch( gd->uart_console ) 
    {  
      case 0:
        return &eserial1_device;
      break;
      case 1:
        return &eserial2_device;
      break;
      case 2:
        return &eserial3_device;
      break;
      case 3:
        return &eserial4_device;
      break;
      default:
        return &eserial1_device;
        //error "Bad CONFIG_CONS_INDEX."
      break;
    }
    #else
    #if CONFIG_CONS_INDEX == 1
    return &eserial1_device;
    #elif CONFIG_CONS_INDEX == 2
    return &eserial2_device;
    #elif CONFIG_CONS_INDEX == 3
    return &eserial3_device;
    #elif CONFIG_CONS_INDEX == 4
    return &eserial4_device;
    #else
    #error "Bad CONFIG_CONS_INDEX."
    #endif
    #endif
}

#endif 
//CONFIG_SERIAL_MULTI






#undef readl
#undef writel

#define readl(addr) (*(volatile u32 *) (addr))
#define writel(val, addr) ((*(volatile u32 *) (addr)) = (val))


static __u32 port = 2;
#define REGS_BASE    0x01C00000		//寄存器物理地址
                                                                                         
// 物理地址
#define CCMU_REGS_BASE         ( REGS_BASE + 0x20000 )    //clock manager unit               
#define PIOC_REGS_BASE         ( REGS_BASE + 0x20800 )    //general perpose I/O                       
#define TMRC_REGS_BASE         ( REGS_BASE + 0x20c00 )    //timer                     
                                                                                   
#define LRAC_REGS_BASE         ( REGS_BASE + 0x22800 )    //lradc
#define ADDA_REGS_BASE         ( REGS_BASE + 0x22c00 )    //AD/DA

#define UART0_REGS_BASE        ( REGS_BASE + 0x28000 )    //uart0 base
#define UART1_REGS_BASE        ( REGS_BASE + 0x28400 )    //uart1 base
#define UART2_REGS_BASE        ( REGS_BASE + 0x28800 )    //uart2 base
#define UART3_REGS_BASE        ( REGS_BASE + 0x28C00 )    //uart3 base
#define UART4_REGS_BASE        ( REGS_BASE + 0x29000 )    //uart4 base

#define TWIC0_REGS_BASE        ( REGS_BASE + 0x2AC00 )    //twi0
#define TWIC1_REGS_BASE        ( REGS_BASE + 0x2B000 )    //twi1
#define TWIC2_REGS_BASE        ( REGS_BASE + 0x2B400 )    //twi2

#define DMAC_REGS_BASE         ( REGS_BASE + 0x02000 )    //4k    DMA controller

#define SDMC0_REGS_BASE        ( REGS_BASE + 0x0f000 )    //sdmmc0 controller
#define SDMC1_REGS_BASE        ( REGS_BASE + 0x10000 )    //sdmmc1 controller
#define SDMC2_REGS_BASE        ( REGS_BASE + 0x11000 )    //sdmmc2 controller
//#define SDMC3_REGS_BASE        ( REGS_BASE + 0x12000 )              //sdmmc3 controller

#define ARMA9_GIC_BASE		   ( 0x01c81000)
#define ARMA9_CPUIF_BASE	   ( 0x01c82000)

#define R_PRCM_REGS_BASE       ( 0x01f01400)
#define RUART0_REGS_BASE	   ( 0x01f02800)
#define P2WI_REGS_BASE         ( 0x01F03400)


#define  DRAM_MEM_BASE         ( 0x40000000)








#define  UART_REGS_BASE    			UART0_REGS_BASE

#define UART_REG_o_RBR              0x00
#define UART_REG_o_THR              0x00
#define UART_REG_o_DLL              0x00
#define UART_REG_o_DLH              0x04
#define UART_REG_o_IER              0x04
#define UART_REG_o_IIR              0x08
#define UART_REG_o_FCR              0x08
#define UART_REG_o_LCR              0x0C
#define UART_REG_o_MCR              0x10
#define UART_REG_o_LSR              0x14
#define UART_REG_o_MSR              0x18
#define UART_REG_o_SCH              0x1C
#define UART_REG_o_USR              0x7C
#define UART_REG_o_TFL              0x80
#define UART_REG_o_RFL              0x84
#define UART_REG_o_HALT             0xA4


#define UART_REG_RBR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_RBR  )
#define UART_REG_THR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_THR  )
#define UART_REG_DLL(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_DLL  )
#define UART_REG_DLH(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_DLH  )
#define UART_REG_IER(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_IER  )
#define UART_REG_IIR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_IIR  )
#define UART_REG_FCR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_FCR  )
#define UART_REG_LCR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_LCR  )
#define UART_REG_MCR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_MCR  )
#define UART_REG_LSR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_LSR  )
#define UART_REG_MSR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_MSR  )
#define UART_REG_SCH(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_SCH  )
#define UART_REG_USR(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_USR  )
#define UART_REG_TFL(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_TFL  )
#define UART_REG_RFL(port)          readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_RFL  )
#define UART_REG_HALT(port)         readl( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_HALT )





#define   UART_OFFSET   0x400
#define   TxFIFOSize    1024
//UART Baudrate Control
#define   UART_BAUD    115200      // Baud rate for UART
                                   // Compute the divisor factor
// UART Line Control Parameter
#define   PARITY       0           // Parity: 0,2 - no parity; 1 - odd parity; 3 - even parity
#define   STOP         0           // Number of Stop Bit: 0 - 1bit; 1 - 2(or 1.5)bits
#define   DLEN         3           // Data Length: 0 - 5bits; 1 - 6bits; 2 - 7bits; 3 - 8bits


#define SERIAL_CHAR_READY()     ( UART_REG_LSR(port) & ( 1 << 0 ) )
#define SERIAL_READ_CHAR()      UART_REG_RBR(port)
#define SERIAL_READ_STATUS()    ( UART_REG_LSR(port) & 0xFF )
#define SERIAL_WRITE_STATUS()	UART_REG_LSR(port)
#define SERIAL_WRITE_READY()	( UART_REG_LSR(port) & ( 1 << 6 ) )
#define SERIAL_WRITE_CHAR(c)	( ( UART_REG_THR(port) ) = ( c ) )








#define PIOC_REG_o_CFG0                 0x00
#define PIOC_REG_o_CFG1                 0x04
#define PIOC_REG_o_CFG2                 0x08
#define PIOC_REG_o_CFG3                 0x0C
#define PIOC_REG_o_DATA                 0x10
#define PIOC_REG_o_DRV0                 0x14
#define PIOC_REG_o_DRV1                 0x18
#define PIOC_REG_o_PUL0                 0x1C
#define PIOC_REG_o_PUL1                 0x20

  /* offset */
#define PIO_REG_CFG(n, i)               ((unsigned int *)( PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00))
#define PIO_REG_DLEVEL(n, i)            ((unsigned int *)( PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14))
#define PIO_REG_PULL(n, i)              ((unsigned int *)( PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C))
#define PIO_REG_DATA(n) 	            ((unsigned int *)( PIOC_REGS_BASE + ((n)-1)*0x24 + 0x10))

#define PIO_REG_CFG_VALUE(n, i)          readl( PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00)
#define PIO_REG_DLEVEL_VALUE(n, i)       readl( PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14)
#define PIO_REG_PULL_VALUE(n, i)         readl( PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C)
#define PIO_REG_DATA_VALUE(n) 	         readl( PIOC_REGS_BASE + ((n)-1)*0x24 + 0x10)

#define PIO_REG_BASE(n)                  ((unsigned int *)( PIOC_REGS_BASE + ((n)-1)*0x24))








__s32 boot_set_gpio( void *user_gpio_list, __u32 group_count_max, __s32 set_gpio )
{
	normal_gpio_cfg    *tmp_user_gpio_data, *gpio_list;
	__u32				first_port;                      
	//保存真正有效的GPIO的个数
	__u32    tmp_group_func_data = 0;
	__u32    tmp_group_pull_data = 0;
	__u32    tmp_group_dlevel_data = 0;
	__u32    tmp_group_data_data = 0;
	__u32    data_change = 0;
	__u32    *tmp_group_port_addr = NULL;
	__u32    *tmp_group_func_addr = NULL, *tmp_group_pull_addr = NULL;
	__u32    *tmp_group_dlevel_addr = NULL, *tmp_group_data_addr = NULL;
	__u32  	 port, port_num, port_num_func, port_num_pull;
	__u32  	 pre_port = 0, pre_port_num_func = 0;
	__u32  	 pre_port_num_pull = 0;
	__s32    i, tmp_val;
    
    //准备第一个GPIO数据
    gpio_list = user_gpio_list;
    for( first_port = 0; first_port < group_count_max; first_port++ ){
         tmp_user_gpio_data = gpio_list + first_port;
         port = tmp_user_gpio_data->port;                         
		 //读出端口数值
	     port_num = tmp_user_gpio_data->port_num;                     
		 //读出端口中的某一个GPIO
	    if( !port ){
	    	continue;
	    }
	    port_num_func = (port_num >> 3);
        port_num_pull = (port_num >> 4);

		tmp_group_port_addr    = PIO_REG_BASE(port);

		tmp_group_func_addr    = tmp_group_port_addr + port_num_func;						  //更新功能寄存器地址
		tmp_group_pull_addr    = tmp_group_port_addr + (PIOC_REG_o_PUL0>>2) + port_num_pull;  //更新pull寄存器
		tmp_group_dlevel_addr  = tmp_group_port_addr + (PIOC_REG_o_DRV0>>2) + port_num_pull;  //更新driver level寄存器
		tmp_group_data_addr    = tmp_group_port_addr + (PIOC_REG_o_DATA>>2); 				  //更新data寄存器

        tmp_group_func_data    = readl(tmp_group_func_addr);
        tmp_group_pull_data    = readl(tmp_group_pull_addr);
        tmp_group_dlevel_data  = readl(tmp_group_dlevel_addr);
        tmp_group_data_data    = readl(tmp_group_data_addr);

        pre_port          = port;
        pre_port_num_func = port_num_func;
        pre_port_num_pull = port_num_pull;
        //更新功能寄存器
        tmp_val = (port_num - (port_num_func << 3)) << 2;
        tmp_group_func_data &= ~(                              0x07  << tmp_val);
        if(set_gpio)
        {
        	tmp_group_func_data |=  (tmp_user_gpio_data->mul_sel & 0x07) << tmp_val;
        }
        //根据pull的值决定是否更新pull寄存器
        tmp_val              =  (port_num - (port_num_pull << 4)) << 1;
        if(tmp_user_gpio_data->pull >= 0)
        {
        	tmp_group_pull_data &= ~(                           0x03  << tmp_val);
        	tmp_group_pull_data |=  (tmp_user_gpio_data->pull & 0x03) << tmp_val;
        }
        //根据driver level的值决定是否更新driver level寄存器
        if(tmp_user_gpio_data->drv_level >= 0)
        {
        	tmp_group_dlevel_data &= ~(                                0x03  << tmp_val);
        	tmp_group_dlevel_data |=  (tmp_user_gpio_data->drv_level & 0x03) << tmp_val;
        }
        //根据用户输入，以及功能分配决定是否更新data寄存器
        if(tmp_user_gpio_data->mul_sel == 1)
        {
            if(tmp_user_gpio_data->data >= 0)
            {
            	tmp_val = tmp_user_gpio_data->data & 1;
                tmp_group_data_data &= ~(1 << port_num);
                tmp_group_data_data |= tmp_val << port_num;
                data_change = 1;
            }
        }

        break;
	}
	//检查是否有数据存在
	if(first_port >= group_count_max)
	{
	    return -1;
	}
	//保存用户数据
	for(i = first_port + 1; i < group_count_max; i++)
	{
		tmp_user_gpio_data = gpio_list + i;                 //gpio_set依次指向用户的每个GPIO数组成员
	    port     = tmp_user_gpio_data->port;                //读出端口数值
	    port_num = tmp_user_gpio_data->port_num;            //读出端口中的某一个GPIO
	    if(!port)
	    {
	    	break;
	    }
        port_num_func = (port_num >> 3);
        port_num_pull = (port_num >> 4);

        if((port_num_pull != pre_port_num_pull) || (port != pre_port))    //如果发现当前引脚的端口不一致，或者所在的pull寄存器不一致
        {
            writel(tmp_group_func_data, tmp_group_func_addr);     //回写功能寄存器
            writel(tmp_group_pull_data, tmp_group_pull_addr);     //回写pull寄存器
            writel(tmp_group_dlevel_data, tmp_group_dlevel_addr); //回写driver level寄存器
            if(data_change)
            {
                data_change = 0;
                writel(tmp_group_data_data, tmp_group_data_addr); //回写data寄存器
            }

			tmp_group_port_addr    = PIO_REG_BASE(port);

			tmp_group_func_addr    = tmp_group_port_addr + port_num_func;						  //更新功能寄存器地址
			tmp_group_pull_addr    = tmp_group_port_addr + (PIOC_REG_o_PUL0>>2) + port_num_pull;  //更新pull寄存器
			tmp_group_dlevel_addr  = tmp_group_port_addr + (PIOC_REG_o_DRV0>>2) + port_num_pull;  //更新driver level寄存器
			tmp_group_data_addr    = tmp_group_port_addr + (PIOC_REG_o_DATA>>2); 				  //更新data寄存器

            tmp_group_func_data    = readl(tmp_group_func_addr);
            tmp_group_pull_data    = readl(tmp_group_pull_addr);
            tmp_group_dlevel_data  = readl(tmp_group_dlevel_addr);
            tmp_group_data_data    = readl(tmp_group_data_addr);
        }
        else if(pre_port_num_func != port_num_func)                       //如果发现当前引脚的功能寄存器不一致
        {
            writel(tmp_group_func_data, tmp_group_func_addr);    //则只回写功能寄存器
            tmp_group_func_addr    = PIO_REG_CFG(port, port_num_func);   //更新功能寄存器地址

            tmp_group_func_data    = readl(tmp_group_func_addr);
        }
		//保存当前硬件寄存器数据
        pre_port_num_pull = port_num_pull;                      //设置当前GPIO成为前一个GPIO
        pre_port_num_func = port_num_func;
        pre_port          = port;

        //更新功能寄存器
        tmp_val = (port_num - (port_num_func << 3)) << 2;
        if(tmp_user_gpio_data->mul_sel)
        {
        	tmp_group_func_data &= ~(                              0x07  << tmp_val);
        	if(set_gpio)
        	{
        		tmp_group_func_data |=  (tmp_user_gpio_data->mul_sel & 0x07) << tmp_val;
        	}
        }
        //根据pull的值决定是否更新pull寄存器
        tmp_val              =  (port_num - (port_num_pull << 4)) << 1;
        if(tmp_user_gpio_data->pull >= 0)
        {
        	tmp_group_pull_data &= ~(                           0x03  << tmp_val);
        	tmp_group_pull_data |=  (tmp_user_gpio_data->pull & 0x03) << tmp_val;
        }
        //根据driver level的值决定是否更新driver level寄存器
        if(tmp_user_gpio_data->drv_level >= 0)
        {
        	tmp_group_dlevel_data &= ~(                                0x03  << tmp_val);
        	tmp_group_dlevel_data |=  (tmp_user_gpio_data->drv_level & 0x03) << tmp_val;
        }
        //根据用户输入，以及功能分配决定是否更新data寄存器
        if(tmp_user_gpio_data->mul_sel == 1)
        {
            if(tmp_user_gpio_data->data >= 0)
            {
            	tmp_val = tmp_user_gpio_data->data & 1;
                tmp_group_data_data &= ~(1 << port_num);
                tmp_group_data_data |= tmp_val << port_num;
                data_change = 1;
            }
        }
    }
    //for循环结束，如果存在还没有回写的寄存器，这里写回到硬件当中
    if( tmp_group_func_addr )                         //只要更新过寄存器地址，就可以对硬件赋值
    {                                               //那么把所有的值全部回写到硬件寄存器
        writel(tmp_group_func_data, tmp_group_func_addr);   //回写功能寄存器
        writel(tmp_group_pull_data, tmp_group_pull_addr);   //回写pull寄存器
        writel(tmp_group_dlevel_data, tmp_group_dlevel_addr); //回写driver level寄存器
        if( data_change ){
            writel(tmp_group_data_data, tmp_group_data_addr); //回写data寄存器
        }
    }

    return 0;
}





void UART_open( __s32 uart_port, void  *uart_ctrl, __u32 apb_freq )
{
  	__u32   temp=0, i;
  	__u32   uart_clk;
  	__u32   lcr;
  	volatile unsigned int   *reg;

	port = uart_port;

	// config clock
	if(port > 7)
	{
		return ;
	}
	reg = (volatile unsigned int *)0x01c2006C;
	*reg &= ~(1 << (16 + port));
	for( i = 0; i < 100; i++ );
	*reg |=  (1 << (16 + port));
	//Bus Clock Gating Register 3 
	//总线时钟门控寄存器
	//开启对应串口的时钟源
    
	(*(volatile unsigned int *)0x01c202D8) |= (1 << (16 + port));
    //Bus Software Reset Register 4
    //软件复位对应串口
        
	// config uart gpio
	// config tx gpio
	boot_set_gpio((void *)uart_ctrl, 2, 1);



	
    // Set Baudrate
    uart_clk = ( apb_freq + 8*UART_BAUD ) / (16*UART_BAUD);
    lcr = UART_REG_LCR(port);
    UART_REG_HALT(port) = 1;
    UART_REG_LCR(port) = lcr | 0x80;
    UART_REG_DLH(port) = uart_clk>>8;
    UART_REG_DLL(port) = uart_clk&0xff;
    UART_REG_LCR(port) = lcr & (~0x80);
	UART_REG_HALT(port) = 0;
    // Set Lin Control Register
    temp = ((PARITY&0x03)<<3) | ((STOP&0x01)<<2) | (DLEN&0x03);
    UART_REG_LCR(port) = temp;

    // Disable FIFOs
    UART_REG_FCR(port) = 0x06; 
}                                                                       






#if 0
uboot_spare_head.boot_data.uart_port = 2;
    uboot_spare_head.boot_data.uart_gpio[0].port = 2;
	uboot_spare_head.boot_data.uart_gpio[0].port_num = 0;
	uboot_spare_head.boot_data.uart_gpio[0].mul_sel = 2;
	                       
	uboot_spare_head.boot_data.uart_gpio[1].port = 2;
    uboot_spare_head.boot_data.uart_gpio[1].port_num = 1; 
	uboot_spare_head.boot_data.uart_gpio[1].mul_sel = 2;
	UART_open( uboot_spare_head.boot_data.uart_port, (void *)&uboot_spare_head.boot_data.uart_gpio[0], 24*1000*1000 );
#endif
	

void asm_UART2_open( void )
{
  	__u32   temp=0, i;
  	__u32   uart_clk;
  	__u32   lcr;
  	volatile unsigned int   *reg;
     int port;
             
     struct spare_boot_head_t  uboot_spare_head_local; 
	 void  *uart_ctrl;
	 __u32 apb_freq;
	 //return;
      uboot_spare_head.boot_data.uart_port = 2;
	 uboot_spare_head_local.boot_data.uart_port = 2;
    uboot_spare_head_local.boot_data.uart_gpio[0].port = 2;
	uboot_spare_head_local.boot_data.uart_gpio[0].port_num = 0;
	uboot_spare_head_local.boot_data.uart_gpio[0].mul_sel = 2;
	                       
	uboot_spare_head_local.boot_data.uart_gpio[1].port = 2;
    uboot_spare_head_local.boot_data.uart_gpio[1].port_num = 1; 
	uboot_spare_head_local.boot_data.uart_gpio[1].mul_sel = 2;
	port = 2;

	// config clock
	if(port > 7)
	{
		return ;
	}
	reg = (volatile unsigned int *)0x01c2006C;
	*reg &= ~(1 << (16 + port));
	for( i = 0; i < 100; i++ );
	*reg |=  (1 << (16 + port));
	//Bus Clock Gating Register 3 
	//总线时钟门控寄存器
	//开启对应串口的时钟源
    
	(*(volatile unsigned int *)0x01c202D8) |= (1 << (16 + port));
    //Bus Software Reset Register 4
    //软件复位对应串口
        
	// config uart gpio
	// config tx gpio

	uart_ctrl = (void *)&uboot_spare_head_local.boot_data.uart_gpio[0];
	
	boot_set_gpio((void *)uart_ctrl, 2, 1);

     apb_freq = 24*1000*1000;

	
    // Set Baudrate
    uart_clk = ( apb_freq + 8*UART_BAUD ) / (16*UART_BAUD);
    lcr = UART_REG_LCR(port);
    UART_REG_HALT(port) = 1;
    UART_REG_LCR(port) = lcr | 0x80;
    UART_REG_DLH(port) = uart_clk>>8;
    UART_REG_DLL(port) = uart_clk&0xff;
    UART_REG_LCR(port) = lcr & (~0x80);
	UART_REG_HALT(port) = 0;
    // Set Lin Control Register
    temp = ((PARITY&0x03)<<3) | ((STOP&0x01)<<2) | (DLEN&0x03);
    UART_REG_LCR(port) = temp;

    // Disable FIFOs
    UART_REG_FCR(port) = 0x06; 
}                                                                       





#define  MASK_LOW4      0xf
#define  NEGATIVE       1
#define  POSITIVE       0
#define  HEX_x          'x'
#define  HEX_X          'X'








/*
******************************************************************************************************************
*
*Function Name : int_to_string_dec
*
*Description : This function is to convert an 'int' data 'input' to a string in decimalism, and the string
*              converted is in 'str'.
*
*Input : int input : 'int' data to be converted.
*        char * str : the start address of the string storing the converted result.
*
*Output : void
*
*call for :
*
*Others : None at present.
*
*******************************************************************************************************************
*/



//void _sys_exit( void )
//{
//	return;
//}



//#pragma arm section  code="int_to_string_dec"
void int_to_string_dec( int input , char * str)
{
	char stack[12];
	char sign_flag = POSITIVE ;      // 'sign_flag indicates wheater 'input' is positive or negative, default
	int i ;                           // value is 'POSITIVE'.
	int j ;



	if( input == 0 )
	{
		str[0] = '0';
		str[1] = '\0';                   // 'str' must end with '\0'
		return ;
	}

	if( input < 0 )                      // If 'input' is negative, 'input' is assigned to its absolute value.
	{
		sign_flag = NEGATIVE ;
		input = -input ;
	}

	for( i = 0; input > 0; ++i )
	{
		stack[i] = input%10 + '0';      // characters in reverse order are put in 'stack' .
		input /= 10;
	}                                   // at the end of 'for' loop, 'i' is the number of characters.


    j = 0;
	if( sign_flag == NEGATIVE )
		str[j++] = '-';		            // If 'input' is negative, minus sign '-' is placed in the head.
	for( --i  ; i >= 0; --i, ++j )
		str[j] = stack[i];
	str[j] = '\0';				        // 'str' must end with '\0'
}







/*
******************************************************************************************************************
*
*Function Name : Uint_to_string_dec
*
*Description : This function is to convert an 'unsigned int' data 'input' to a string in decimalism, and the
*              string converted is in 'str'.
*
*Input : int input : 'unsigned int' data to be converted.
*        char * str : the start address of the string storing the converted result.
*
*Output : void
*
*call for :
*
*Others : None at present.
*
*******************************************************************************************************************
*/
//#pragma arm section  code="Uint_to_string_dec"
void Uint_to_string_dec( unsigned int input, char * str )
{
	char stack[11] ;
	int i ;
	int j ;


	if( input == 0 )
	{
		str[0] = '0';
		str[1] = 0;                   // 'str' must end with '\0'
		return ;
	}

	for( i = 0; input > 0; ++i )
	{
		stack[i] = input%10 + '0';       // characters in reverse order are put in 'stack' .
		input /= 10;
	}                                    // at the end of 'for' loop, 'i' is the number of characters.

	for( --i, j = 0; i >= 0; --i, ++j )
		str[j] = stack[i];
	str[j] = '\0';		                 // 'str' must end with '\0'
}








/*
******************************************************************************************************************
*
*Function Name : int_to_string_hex
*
*Description : This function is to convert an 'int' data 'input' to a string in hex, and the string
*              converted is in 'str'.
*
*Input : int input : 'int' data to be converted.
*        char * str : the start address of the string storing the converted result.
*        int hex_flag : hex_flag is just a option to distinguish whether hex format is '0f' or '0F'.
*
*Output : void
*
*call for :
*
*Others : None at present.
*
*******************************************************************************************************************
*/
//#pragma arm section  code="int_to_string_hex"
void int_to_string_hex( int input, char * str, int hex_flag )
{
	int i;
	static char base[] = "0123456789abcdef";


	for( i = 9; i > 1; --i )
	{
		str[i] = base[input&MASK_LOW4];
		input >>= 4;
	}

	str[0] = '0';
	str[1] = 'x';                         // Hex format shoud start with "0x" or "0X".
	str[10] = '\0';                        // 'str' must end with '\0'

	return;
}








void UART_putchar(char c)
{
	while (!SERIAL_WRITE_READY())
	  ; 					  /* nothing */

	SERIAL_WRITE_CHAR(c);
}

void UART_puts_no_newline( const char * str )
{
	while( *str != '\0' )
	{
		if( *str == '\n' )						// if current character is '\n', insert and output '\r'
			UART_putchar( '\r' );

		UART_putchar( *str++ );
	}
}
/*
******************************************************************************************************************
*
*Function Name : UART_printf
*
*Description : This function is to formatedly output through UART, similar to ANSI C function printf().
*				 This function can support and only support the following Conversion Specifiers:
*			   %d		Signed decimal integer.
*			   %u		Unsigned decimal integer.
*			   %x		Unsigned hexadecimal integer, using hex digits 0f.
*			   %X		Unsigned hexadecimal integer, using hex digits 0F.
*			   %c		Single character.
*			   %s		Character string.
*			   %p		A pointer.
*
*Input : refer to ANSI C function printf().
*
*Output : void, different form ANSI C function printf().
*
*call for : void int_to_string_dec( __s32 input, char * str ), defined in format_transformed.c.
*			void int_to_string_hex( __s32 input, char * str );	defined in format_transformed.c.
*			void Uint_to_string_dec( __u32 input, char * str );  defined in format_transformed.c.
*			void UART_putchar( __s32 ch); defined in boot loader.
*			void UART_puts( const char * string ); defined in boot loader.
*
*Others : None at present.
*
*******************************************************************************************************************
*/
void UART_printf2( const char * str, ...)
{
	char string[13];
	char *p;
	__s32 hex_flag ;
	va_list argp;

	va_start( argp, str );

	while( *str )
	{
		if( *str == '%' )
		{
			++str;
			p = string;
			hex_flag = HEX_X;
			switch( *str )
			{
				case 'd': int_to_string_dec( va_arg( argp,	__s32 ), string );
						  UART_puts_no_newline( p );
						  ++str;
						  break;
				case 'x': hex_flag = HEX_x; 		 // jump to " case 'X' "
				case 'p':
				case 'X': int_to_string_hex( va_arg( argp,	__s32 ), string, hex_flag );
						  UART_puts_no_newline( p );
						  ++str;
						  break;
				case 'u': Uint_to_string_dec( va_arg( argp,  __s32 ), string );
						  UART_puts_no_newline( p );
						  ++str;
						  break;
				case 'c': UART_putchar( va_arg( argp,  __s32 ) );
						  ++str;
						  break;
				case 's': UART_puts_no_newline( va_arg( argp, char * ) );
						  ++str;
						  break;
				default : UART_putchar( '%' );		 // if current character is not Conversion Specifiers 'dxpXucs',
						  UART_putchar( *str ); 		// output directly '%' and current character, and then
						  ++str;						// let 'str' point to next character.
			}
		}

		else
		{
			if( *str == '\n' )						// if current character is '\n', insert and output '\r'
				UART_putchar( '\r' );

			UART_putchar( *str++ );
		}
	}

	va_end( argp );
}
          
//UART_open( BT0_head.prvt_head.uart_port, (void *)BT0_head.prvt_head.uart_ctrl, 24*1000*1000 );



