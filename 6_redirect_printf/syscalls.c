#include "hal.h"

#define STDOUT_FD (1)

int _write(int fd, char *ptr, int len)
{
  (void) fd, (void) ptr, (void) len;
  if (fd == STDOUT_FD) usart_write_buffer(USART2, ptr, (size_t) len);
  return -1;
}

void *_sbrk(int incr)
{
    extern char _end;
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;
    if (heap == NULL) heap = (unsigned char *) &_end;
    prev_heap = heap;
    heap += incr;
    return prev_heap;
}

// NOTE(23.11.24): STUB
int _close(int fd)
{
    (void) fd;
    return -1;
}

// NOTE(23.11.24): STUB
int _fstat(int fd, struct stat * st)
{
    if (fd < 0) return -1;
    st->st_mode = S_IFCHR;
    return 0;
}

// NOTE(23.11.24): STUB
int _isatty(int fd)
{
    (void) fd;
    return 1;
}

// NOTE(23.11.24): STUB
int _lseek(int file, int ptr, int dir)
{
    (void) file, (void) ptr, (void) dir;
    return 0;
}

// NOTE(23.11.24): STUB
int _read(int fd, char *buffer, int nbytes)
{
    (void) fd, (void) buffer, (void) nbytes;
    return -1;
}
