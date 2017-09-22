#pragma once

#include <windows.h>
#include <stdio.h>

#define MAXSTRPATH  1024                /* max length of stream path */
#define MAXSTRMSG   1024                /* max length of stream message */
#define TIMETAGH_LEN        64          /* time tag file header length */

#define VER_RTKLIB  "2.4.2"             /* library version */

#ifdef WIN32
#define thread_t    HANDLE
#define lock_t      CRITICAL_SECTION
#define initlock(f) InitializeCriticalSection(f)
#define lock(f)     EnterCriticalSection(f)
#define unlock(f)   LeaveCriticalSection(f)
#define FILEPATHSEP '\\'
#else
#define thread_t    pthread_t
#define lock_t      pthread_mutex_t
#define initlock(f) pthread_mutex_init(f,NULL)
#define lock(f)     pthread_mutex_lock(f)
#define unlock(f)   pthread_mutex_unlock(f)
#define FILEPATHSEP '/'
#endif

#ifdef WIN32
#define dev_t               HANDLE
#define socket_t            SOCKET
typedef int socklen_t;
#else
#define dev_t               int
#define socket_t            int
#define closesocket         close
#endif

#define STR_MODE_R  0x1                 /* stream mode: read */
#define STR_MODE_W  0x2                 /* stream mode: write */
#define STR_MODE_RW 0x3                 /* stream mode: read/write */

#define STR_NONE     0                  /* stream type: none */
#define STR_SERIAL   1                  /* stream type: serial */
#define STR_FILE     2                  /* stream type: file */
#define STR_TCPSVR   3                  /* stream type: TCP server */
#define STR_TCPCLI   4                  /* stream type: TCP client */
#define STR_UDP      5                  /* stream type: UDP stream */
#define STR_NTRIPSVR 6                  /* stream type: NTRIP server */
#define STR_NTRIPCLI 7                  /* stream type: NTRIP client */
#define STR_FTP      8                  /* stream type: ftp */
#define STR_HTTP     9                  /* stream type: http */

typedef struct {            /* serial control type */
	dev_t dev;              /* serial device */
	int error;              /* error state */
#ifdef WIN32
	int state, wp, rp;        /* state,write/read pointer */
	int buffsize;           /* write buffer size (bytes) */
	HANDLE thread;          /* write thread */
	lock_t lock;            /* lock flag */
	unsigned char *buff;    /* write buffer */
#endif
} serial_t;

typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;

typedef struct {            /* file control type */
	FILE *fp;               /* file pointer */
	FILE *fp_tag;           /* file pointer of tag file */
	FILE *fp_tmp;           /* temporary file pointer for swap */
	FILE *fp_tag_tmp;       /* temporary file pointer of tag file for swap */
	char path[MAXSTRPATH];  /* file path */
	char openpath[MAXSTRPATH]; /* open file path */
	int mode;               /* file mode */
	int timetag;            /* time tag flag (0:off,1:on) */
	int repmode;            /* replay mode (0:master,1:slave) */
	int offset;             /* time offset (ms) for slave */
	gtime_t time;           /* start time */
	gtime_t wtime;          /* write time */
	unsigned int tick;      /* start tick */
	unsigned int tick_f;    /* start tick in file */
	unsigned int fpos;      /* current file position */
	double start;           /* start offset (s) */
	double speed;           /* replay speed (time factor) */
	double swapintv;        /* swap interval (hr) (0: no swap) */
	lock_t lock;            /* lock flag */
} file_t;



typedef struct {        /* stream type */
	int type;           /* type (STR_???) */
	int mode;           /* mode (STR_MODE_?) */
	int state;          /* state (-1:error,0:close,1:open) */
	unsigned int inb, inr;   /* input bytes/rate */
	unsigned int outb, outr; /* output bytes/rate */
	unsigned int tick, tact; /* tick/active tick */
	unsigned int inbt, outbt; /* input/output bytes at tick */
	lock_t lock;        /* lock flag */
	void *port;         /* type dependent port control struct */
	char path[MAXSTRPATH]; /* stream path */
	char msg[MAXSTRMSG];  /* stream message */
} stream_t;

extern void strinit(stream_t *stream);
extern int stropen(stream_t *stream, int type, int mode, const char *path);
extern int strread(stream_t *stream, unsigned char *buff, int n);
extern void strclose(stream_t *stream);