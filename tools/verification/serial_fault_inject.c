/* Test-only LD_PRELOAD: force partial nonblocking TTY writes and EAGAIN.
 * Never linked into CompanionBot or used against a real serial device. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static ssize_t (*actual_write)(int,const void *,size_t);
static unsigned chunk=17,again_every=5,delay_us;
static _Atomic unsigned long calls,partials,again,accepted;
__attribute__((constructor)) static void initialize(void) {
    const char *value;
    *(void **)(&actual_write)=dlsym(RTLD_NEXT,"write");
    if((value=getenv("CB_SERIAL_MAX_WRITE")))chunk=(unsigned)strtoul(value,0,10);
    if((value=getenv("CB_SERIAL_EAGAIN_EVERY")))again_every=(unsigned)strtoul(value,0,10);
    if((value=getenv("CB_SERIAL_WRITE_DELAY_US")))delay_us=(unsigned)strtoul(value,0,10);
    if(!chunk || !actual_write)_exit(99);
}
ssize_t write(int fd,const void *buffer,size_t length) {
    unsigned long call;size_t limited;ssize_t result;
    if(!isatty(fd))return actual_write(fd,buffer,length);
    call=atomic_fetch_add(&calls,1)+1;
    if(delay_us)usleep(delay_us);
    if(again_every && call%again_every==0) {
        atomic_fetch_add(&again,1);errno=EAGAIN;return -1;
    }
    limited=length<chunk?length:chunk;
    result=actual_write(fd,buffer,limited);
    if(result>0) {
        atomic_fetch_add(&accepted,(unsigned long)result);
        if((size_t)result<length)atomic_fetch_add(&partials,1);
    }
    return result;
}
__attribute__((destructor)) static void finish(void) {
    const char *path=getenv("CB_SERIAL_PROBE_OUTPUT");char data[256];int n,fd;
    if(!path || !actual_write)return;
    fd=open(path,O_CREAT|O_TRUNC|O_WRONLY,0600);if(fd<0)return;
    n=snprintf(data,sizeof(data),"{\"tty_write_calls\":%lu,\"partial_writes\":%lu,\"forced_eagain\":%lu,\"accepted_bytes\":%lu}\n",
        atomic_load(&calls),atomic_load(&partials),atomic_load(&again),atomic_load(&accepted));
    if(n>0)(void)actual_write(fd,data,(size_t)n);
    close(fd);
}
