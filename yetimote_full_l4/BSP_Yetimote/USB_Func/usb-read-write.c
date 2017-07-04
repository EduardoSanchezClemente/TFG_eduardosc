/******************************************************************************/
//NEWLIB
/*
 * usb-read-writes.c
 *
 *  Created on: 15 Feb 2015
 *      Author: r.rodriguezz
 */
#include "stm32l4xx_hal.h"
#include "yetimote-conf.h"
#include "usbd_cdc_if.h"
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>


#ifndef STDOUT_USB
#define STDOUT_USB
#endif

#ifndef STDERR_USB
#define STDERR_USB
#endif

#ifndef STDIN_USB
#define STDIN_USB
#endif

#undef errno
extern int errno;


extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status) {
    _write(1, "exit", 4);
    while (1) {
        ;
    }
}

int _close(int file) {
    return -1;
}
/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid() {
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {

    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */


int _read(int file, char *ptr, int len) {
//    int n;
    int num = 0;
//    switch (file) {
//    case STDIN_FILENO:
//        for (n = 0; n < len; n++) {
//#if   STDIN_USART == 1
//            while ((USART1->SR & USART_FLAG_RXNE) == (uint16_t)RESET) {}
//            char c = (char)(USART1->DR & (uint16_t)0x01FF);
//#elif STDIN_USART == 2
//            while ((USART2->SR & USART_FLAG_RXNE) == (uint16_t) RESET) {}
//            char c = (char) (USART2->DR & (uint16_t) 0x01FF);
//#elif STDIN_USART == 3
//            while ((USART3->SR & USART_FLAG_RXNE) == (uint16_t)RESET) {}
//            char c = (char)(USART3->DR & (uint16_t)0x01FF);
//#endif
//            *ptr++ = c;
//            num++;
//        }
//        break;
//    default:
//        errno = EBADF;
//        return -1;
//    }
    return num;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) {
//    int n;
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef  *pdev = hpcd->pData;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
#if DEBUG_USB
#ifdef STDOUT_USB

		if(pdev->dev_state != USBD_STATE_SUSPENDED){	//De este modo nos aseguramos que no se envia nada si el cable USB no está conectado
			CDC_Transmit_FS((uint8_t*)ptr, len);					//Si no entraria en infinite loop al usar un printf
		}

#endif
#endif
        break;
//    case STDERR_FILENO: /* stderr */
//        for (n = 0; n < len; n++) {
//#ifdef STDOUT_USB
//        	CDC_Transmit_FS(ptr, len);
//#endif
//        }
//        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}
/******************************************************************************/
