//#pragma comment(lib,"ws2_32.lib") //lwf
#pragma comment(lib, "winmm.lib ")

#include "datastream.h"
#include <time.h>
#include <math.h>
#include <mmsystem.h>

static int buffsize = 32768; /* receive/send buffer size (bytes) */
static double timeoffset_ = 0.0;
static unsigned int tick_master = 0; /* time tick master for replay */
static int tirate = 1000;  /* avraging time for data rate (ms) */

const static double leaps[][7] = { /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
		{ 2012, 7, 1, 0, 0, 0, -16 },
		{ 2009, 1, 1, 0, 0, 0, -15 },
		{ 2006, 1, 1, 0, 0, 0, -14 },
		{ 1999, 1, 1, 0, 0, 0, -13 },
		{ 1997, 7, 1, 0, 0, 0, -12 },
		{ 1996, 1, 1, 0, 0, 0, -11 },
		{ 1994, 7, 1, 0, 0, 0, -10 },
		{ 1993, 7, 1, 0, 0, 0, -9 },
		{ 1992, 7, 1, 0, 0, 0, -8 },
		{ 1991, 1, 1, 0, 0, 0, -7 },
		{ 1990, 1, 1, 0, 0, 0, -6 },
		{ 1988, 1, 1, 0, 0, 0, -5 },
		{ 1985, 7, 1, 0, 0, 0, -4 },
		{ 1983, 7, 1, 0, 0, 0, -3 },
		{ 1982, 7, 1, 0, 0, 0, -2 },
		{ 1981, 7, 1, 0, 0, 0, -1 }
};

const static double gpst0[] = { 1980, 1, 6, 0, 0, 0 }; /* gps time reference */

/* get tick time ---------------------------------------------------------------
* get current tick in ms
* args   : none
* return : current tick in ms
*-----------------------------------------------------------------------------*/
extern unsigned int tickget(void)
{
#ifdef WIN32
	return (unsigned int)timeGetTime();
#else
	struct timespec tp = { 0 };
	struct timeval  tv = { 0 };

	/* linux kernel > 2.6.28 */
	if (!clock_gettime(CLOCK_MONOTONIC_RAW, &tp)) {
		return tp.tv_sec * 1000u + tp.tv_nsec / 1000000u;
	}
	else {
		gettimeofday(&tv, NULL);
		return tv.tv_sec * 1000u + tv.tv_usec / 1000u;
	}
#endif
}

wchar_t* c2w(const char *str)
{
	int length = strlen(str) + 1;
	wchar_t *t = (wchar_t*)malloc(sizeof(wchar_t)*length);
	memset(t, 0, length*sizeof(wchar_t));
	MultiByteToWideChar(CP_ACP, 0, str, strlen(str), t, length);
	return t;
}

/* sleep ms --------------------------------------------------------------------
* sleep ms
* args   : int   ms         I   miliseconds to sleep (<0:no sleep)
* return : none
*-----------------------------------------------------------------------------*/
extern void sleepms(int ms)
{
#ifdef WIN32
	if (ms<5) Sleep(1); else Sleep(ms);
#else
	struct timespec ts;
	if (ms <= 0) return;
	ts.tv_sec = (time_t)(ms / 1000);
	ts.tv_nsec = (long)(ms % 1000 * 1000000);
	nanosleep(&ts, NULL);
#endif
}

extern void strlock(stream_t *stream) { lock(&stream->lock); }
extern void strunlock(stream_t *stream) { unlock(&stream->lock); }


/* read/write serial buffer --------------------------------------------------*/
#ifdef WIN32
static int readseribuff(serial_t *serial, unsigned char *buff, int nmax)
{
	int ns;

//	tracet(5, "readseribuff: dev=%d\n", serial->dev);

	lock(&serial->lock);
	for (ns = 0; serial->rp != serial->wp&&ns<nmax; ns++) {
		buff[ns] = serial->buff[serial->rp];
		if (++serial->rp >= serial->buffsize) serial->rp = 0;
	}
	unlock(&serial->lock);
//	tracet(5, "readseribuff: ns=%d rp=%d wp=%d\n", ns, serial->rp, serial->wp);
	return ns;
}
static int writeseribuff(serial_t *serial, unsigned char *buff, int n)
{
	int ns, wp;

//	tracet(5, "writeseribuff: dev=%d n=%d\n", serial->dev, n);

	lock(&serial->lock);
	for (ns = 0; ns<n; ns++) {
		serial->buff[wp = serial->wp] = buff[ns];
		if (++wp >= serial->buffsize) wp = 0;
		if (wp != serial->rp) serial->wp = wp;
		else {
//			tracet(2, "serial buffer overflow: size=%d\n", serial->buffsize);
			break;
		}
	}
	unlock(&serial->lock);
//	tracet(5, "writeseribuff: ns=%d rp=%d wp=%d\n", ns, serial->rp, serial->wp);
	return ns;
}
#endif /* WIN32 */

/* write serial thread -------------------------------------------------------*/
#ifdef WIN32
static DWORD WINAPI serialthread(void *arg)
{
	serial_t *serial = (serial_t *)arg;
	unsigned char buff[128];
	unsigned int tick;
	DWORD ns;
	int n;

//	tracet(3, "serialthread:\n");

	serial->state = 1;

	for (;;) {
		tick = tickget();
		while ((n = readseribuff(serial, buff, sizeof(buff)))>0) {
			if (!WriteFile(serial->dev, buff, n, &ns, NULL)) serial->error = 1;
		}
		if (!serial->state) break;
		sleepms(10 - (int)(tickget() - tick)); /* cycle=10ms */
	}
	free(serial->buff);
	return 0;
}
#endif /* WIN32 */

/* open serial ---------------------------------------------------------------*/
static serial_t *openserial(const char *path, int mode, char *msg)
{
	const int br[] = {
		300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400
	};
	serial_t *serial;
	int i, brate = 115200, bsize = 8, stopb = 1;
	char *p, parity = 'N', dev[128], port[128], fctr[64] = "";
#ifdef WIN32
	DWORD error, rw = 0, siz = sizeof(COMMCONFIG);
	COMMCONFIG cc = { 0 };
	COMMTIMEOUTS co = { MAXDWORD, 0, 0, 0, 0 }; /* non-block-read */
	char dcb[64] = "";
#else
	const speed_t bs[] = {
		B300, B600, B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400
	};
	struct termios ios = { 0 };
	int rw = 0;
#endif
	//tracet(3, "openserial: path=%s mode=%d\n", path, mode);

	if (!(serial = (serial_t *)malloc(sizeof(serial_t)))) return NULL;

	if ((p = (char *)strchr(path, ':'))) {
		strncpy(port, path, p - path); port[p - path] = '\0';
		sscanf(p, ":%d:%d:%c:%d:%s", &brate, &bsize, &parity, &stopb, fctr);
	}
	else strcpy(port, path);

	for (i = 0; i<11; i++) if (br[i] == brate) break;
	if (i >= 12) {
		sprintf(msg, "bitrate error (%d)", brate);
	//	tracet(1, "openserial: %s path=%s\n", msg, path);
		free(serial);
		return NULL;
	}
	parity = (char)toupper((int)parity);

#ifdef WIN32
	// sprintf(dev,"\\\\.\\%s",port);
	sprintf(dev, "%s", port); //lwf
	if (mode&STR_MODE_R) rw |= GENERIC_READ;
	if (mode&STR_MODE_W) rw |= GENERIC_WRITE;

	wchar_t *devnum = c2w(dev); //lwf
	serial->dev = CreateFile(devnum, rw, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (serial->dev == INVALID_HANDLE_VALUE)
	{
		sprintf(msg, "device open error (%d)", (int)GetLastError());
	//	tracet(1, "openserial: %s path=%s\n", msg, path);
		free(serial);
		return NULL;
	}

	if (!GetDefaultCommConfig(devnum, &cc, &siz))
	{
		sprintf(msg, "getconfig error (%d)", (int)GetLastError());
	//	tracet(1, "openserial: %s\n", msg);
		CloseHandle(serial->dev);
		free(serial);
		return NULL;
	}

	wchar_t *dcbtran = c2w(dcb);//lwf
	sprintf(dcb, "baud=%d parity=%c data=%d stop=%d", brate, parity, bsize, stopb);
	if (!BuildCommDCB(dcbtran, &cc.dcb))
	{
		sprintf(msg, "buiddcb error (%d)", (int)GetLastError());
	//	tracet(1, "openserial: %s\n", msg);
		CloseHandle(serial->dev);
		free(serial);
		return NULL;
	}
	if (!strcmp(fctr, "rts"))
	{
		cc.dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
	}
	free(devnum); free(dcbtran);



	/* create write thread */
	initlock(&serial->lock);
	serial->state = serial->wp = serial->rp = serial->error = 0;
	serial->buffsize = buffsize;
	if (!(serial->buff = (unsigned char *)malloc(buffsize)))
	{
		CloseHandle(serial->dev);
		free(serial);
		return NULL;
	}
	if (!(serial->thread = CreateThread(NULL, 0, serialthread, serial, 0, NULL)))
	{
		sprintf(msg, "serial thread error (%d)", (int)GetLastError());
	//	tracet(1, "openserial: %s\n", msg);
		CloseHandle(serial->dev);
		free(serial);
		return NULL;
	}
	return serial;
#else
	sprintf(dev, "/dev/%s", port);

	if ((mode&STR_MODE_R) && (mode&STR_MODE_W)) rw = O_RDWR;
	else if (mode&STR_MODE_R) rw = O_RDONLY;
	else if (mode&STR_MODE_W) rw = O_WRONLY;

	if ((serial->dev = open(dev, rw | O_NOCTTY | O_NONBLOCK))<0) {
		sprintf(msg, "device open error (%d)", errno);
		tracet(1, "openserial: %s dev=%s\n", msg, dev);
		free(serial);
		return NULL;
	}
	tcgetattr(serial->dev, &ios);
	ios.c_iflag = 0;
	ios.c_oflag = 0;
	ios.c_lflag = 0;     /* non-canonical */
	ios.c_cc[VMIN] = 0; /* non-block-mode */
	ios.c_cc[VTIME] = 0;
	cfsetospeed(&ios, bs[i]);
	cfsetispeed(&ios, bs[i]);
	ios.c_cflag |= bsize == 7 ? CS7 : CS8;
	ios.c_cflag |= parity == 'O' ? (PARENB | PARODD) : (parity == 'E' ? PARENB : 0);
	ios.c_cflag |= stopb == 2 ? CSTOPB : 0;
	ios.c_cflag |= !strcmp(fctr, "rts") ? CRTSCTS : 0;
	tcsetattr(serial->dev, TCSANOW, &ios);
	tcflush(serial->dev, TCIOFLUSH);
	return serial;
#endif
}

extern double timediff(gtime_t t1, gtime_t t2)
{
	return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

extern gtime_t epoch2time(const double *ep)
{
	const int doy[] = { 1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 };
	gtime_t time = { 0 };
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year<1970 || 2099<year || mon<1 || 12<mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}

extern void time2epoch(gtime_t t, double *ep)
{
	const int mday[] = { /* # of days in a month */
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
		31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
	int days, sec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon<48; mon++) {
		if (day >= mday[mon]) day -= mday[mon]; else break;
	}
	ep[0] = 1970 + days / 1461 * 4 + mon / 12; ep[1] = mon % 12 + 1; ep[2] = day + 1;
	ep[3] = sec / 3600; ep[4] = sec % 3600 / 60; ep[5] = sec % 60 + t.sec;
}

extern gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec += sec; tt = floor(t.sec); t.time += (int)tt; t.sec -= tt;
	return t;
}

extern gtime_t utc2gpst(gtime_t t)
{
	int i;

	for (i = 0; i<(int)sizeof(leaps) / (int)sizeof(*leaps); i++) {
		if (timediff(t, epoch2time(leaps[i])) >= 0.0) return timeadd(t, -leaps[i][6]);
	}
	return t;
}

extern double time2gpst(gtime_t t, int *week)
{
	gtime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - w * 86400 * 7) + t.sec;
}

extern gtime_t timeget(void)
{
	double ep[6] = { 0 };
#ifdef WIN32
	SYSTEMTIME ts;

	GetSystemTime(&ts); /* utc */
	ep[0] = ts.wYear; ep[1] = ts.wMonth;  ep[2] = ts.wDay;
	ep[3] = ts.wHour; ep[4] = ts.wMinute; ep[5] = ts.wSecond + ts.wMilliseconds*1E-3;
#else
	struct timeval tv;
	struct tm *tt;

	if (!gettimeofday(&tv, NULL) && (tt = gmtime(&tv.tv_sec))) {
		ep[0] = tt->tm_year + 1900; ep[1] = tt->tm_mon + 1; ep[2] = tt->tm_mday;
		ep[3] = tt->tm_hour; ep[4] = tt->tm_min; ep[5] = tt->tm_sec + tv.tv_usec*1E-6;
	}
#endif
	return timeadd(epoch2time(ep), timeoffset_);
}

extern void timeset(gtime_t t)
{
	timeoffset_ += timediff(t, timeget());
}

/* replace string ------------------------------------------------------------*/
static int repstr(char *str, const char *pat, const char *rep)
{
	int len = strlen(pat);
	char buff[1024], *p, *q, *r;

	for (p = str, r = buff; *p; p = q + len) {
		if (!(q = strstr(p, pat))) break;
		strncpy(r, p, q - p);
		r += q - p;
		r += sprintf(r, "%s", rep);
	}
	if (p <= str) return 0;
	strcpy(r, p);
	strcpy(str, buff);
	return 1;
}


extern int reppath(const char *path, char *rpath, gtime_t time, const char *rov,
	const char *base)
{
	double ep[6], ep0[6] = { 2000, 1, 1, 0, 0, 0 };
	int week, dow, doy, stat = 0;
	char rep[64];

	strcpy(rpath, path);

	if (!strstr(rpath, "%")) return 0;
	if (*rov) stat |= repstr(rpath, "%r", rov);
	if (*base) stat |= repstr(rpath, "%b", base);
	if (time.time != 0) {
		time2epoch(time, ep);
		ep0[0] = ep[0];
		dow = (int)floor(time2gpst(time, &week) / 86400.0);
		doy = (int)floor(timediff(time, epoch2time(ep0)) / 86400.0) + 1;
		sprintf(rep, "%02d", ((int)ep[3] / 3) * 3);   stat |= repstr(rpath, "%ha", rep);
		sprintf(rep, "%02d", ((int)ep[3] / 6) * 6);   stat |= repstr(rpath, "%hb", rep);
		sprintf(rep, "%02d", ((int)ep[3] / 12) * 12); stat |= repstr(rpath, "%hc", rep);
		sprintf(rep, "%04.0f", ep[0]);              stat |= repstr(rpath, "%Y", rep);
		sprintf(rep, "%02.0f", fmod(ep[0], 100.0));  stat |= repstr(rpath, "%y", rep);
		sprintf(rep, "%02.0f", ep[1]);              stat |= repstr(rpath, "%m", rep);
		sprintf(rep, "%02.0f", ep[2]);              stat |= repstr(rpath, "%d", rep);
		sprintf(rep, "%02.0f", ep[3]);              stat |= repstr(rpath, "%h", rep);
		sprintf(rep, "%02.0f", ep[4]);              stat |= repstr(rpath, "%M", rep);
		sprintf(rep, "%02.0f", floor(ep[5]));       stat |= repstr(rpath, "%S", rep);
		sprintf(rep, "%03d", doy);                stat |= repstr(rpath, "%n", rep);
		sprintf(rep, "%04d", week);               stat |= repstr(rpath, "%W", rep);
		sprintf(rep, "%d", dow);                stat |= repstr(rpath, "%D", rep);
		sprintf(rep, "%c", 'a' + (int)ep[3]);     stat |= repstr(rpath, "%H", rep);
		sprintf(rep, "%02d", ((int)ep[4] / 15) * 15); stat |= repstr(rpath, "%t", rep);
	}
	else if (strstr(rpath, "%ha") || strstr(rpath, "%hb") || strstr(rpath, "%hc") ||
		strstr(rpath, "%Y") || strstr(rpath, "%y") || strstr(rpath, "%m") ||
		strstr(rpath, "%d") || strstr(rpath, "%h") || strstr(rpath, "%M") ||
		strstr(rpath, "%S") || strstr(rpath, "%n") || strstr(rpath, "%W") ||
		strstr(rpath, "%D") || strstr(rpath, "%H") || strstr(rpath, "%t")) {
		return -1; /* no valid time */
	}
//	trace(3, "reppath : rpath=%s\n", rpath);
	return stat;
}

extern void createdir(const char *path)
{
	char buff[1024], *p;
	wchar_t* Buff;

//	tracet(3, "createdir: path=%s\n", path);

	strcpy(buff, path);
	if (!(p = strrchr(buff, FILEPATHSEP))) return;
	*p = '\0';

#ifdef WIN32
	Buff=c2w(buff);
	CreateDirectory(Buff, NULL);
#else
	mkdir(buff, 0777);
#endif
}

static int openfile_(file_t *file, gtime_t time, char *msg)
{
	FILE *fp;
	char *rw, tagpath[MAXSTRPATH + 4] = "";
	char tagh[TIMETAGH_LEN + 1] = "";

//	tracet(3, "openfile_: path=%s time=%s\n", file->path, time_str(time, 0));

	file->time = utc2gpst(timeget());
	file->tick = file->tick_f = tickget();
	file->fpos = 0;

	/* replace keywords */
	reppath(file->path, file->openpath, time, "", "");

	/* create directory */
	if ((file->mode&STR_MODE_W) && !(file->mode&STR_MODE_R)) {
		createdir(file->openpath);
	}
	if (file->mode&STR_MODE_R) rw = "rb"; else rw = "wb";

	if (!(file->fp = fopen(file->openpath, rw))) {
		sprintf(msg, "file open error: %s", file->openpath);
	//	tracet(1, "openfile: %s\n", msg);
		return 0;
	}
	//tracet(4, "openfile_: open file %s (%s)\n", file->openpath, rw);

	sprintf(tagpath, "%s.tag", file->openpath);

	if (file->timetag) { /* output/sync time-tag */

		if (!(file->fp_tag = fopen(tagpath, rw))) {
			sprintf(msg, "tag open error: %s", tagpath);
	//		tracet(1, "openfile: %s\n", msg);
			fclose(file->fp);
			return 0;
		}
	//	tracet(4, "openfile_: open tag file %s (%s)\n", tagpath, rw);

		if (file->mode&STR_MODE_R) {
			if (fread(&tagh, TIMETAGH_LEN, 1, file->fp_tag) == 1 &&
				fread(&file->time, sizeof(file->time), 1, file->fp_tag) == 1) {
				memcpy(&file->tick_f, tagh + TIMETAGH_LEN - 4, sizeof(file->tick_f));
			}
			else {
				file->tick_f = 0;
			}
			/* adust time to read playback file */
			timeset(file->time);
		}
		else {
			sprintf(tagh, "TIMETAG RTKLIB %s", VER_RTKLIB);
			memcpy(tagh + TIMETAGH_LEN - 4, &file->tick_f, sizeof(file->tick_f));
			fwrite(&tagh, 1, TIMETAGH_LEN, file->fp_tag);
			fwrite(&file->time, 1, sizeof(file->time), file->fp_tag);
			/* time tag file structure   */
			/*   HEADER(60)+TICK(4)+TIME(12)+ */
			/*   TICK0(4)+FPOS0(4/8)+    */
			/*   TICK1(4)+FPOS1(4/8)+... */
		}
	}
	else if (file->mode&STR_MODE_W) { /* remove time-tag */
		if ((fp = fopen(tagpath, "rb"))) {
			fclose(fp);
			remove(tagpath);
		}
	}
	return 1;
}

/* open file (path=filepath[::T[::+<off>][::x<speed>]][::S=swapintv]) --------*/
static file_t *openfile(const char *path, int mode, char *msg)
{
	file_t *file;
	gtime_t time, time0 = { 0 };
	double speed = 0.0, start = 0.0, swapintv = 0.0;
	char *p;
	int timetag = 0;

//	tracet(3, "openfile: path=%s mode=%d\n", path, mode);

	if (!(mode&(STR_MODE_R | STR_MODE_W))) return NULL;

	/* file options */
	for (p = (char *)path; (p = strstr(p, "::")); p += 2) { /* file options */
		if (*(p + 2) == 'T') timetag = 1;
		else if (*(p + 2) == '+') sscanf(p + 2, "+%lf", &start);
		else if (*(p + 2) == 'x') sscanf(p + 2, "x%lf", &speed);
		else if (*(p + 2) == 'S') sscanf(p + 2, "S=%lf", &swapintv);
	}
	if (start <= 0.0) start = 0.0;
	if (swapintv <= 0.0) swapintv = 0.0;

	if (!(file = (file_t *)malloc(sizeof(file_t)))) return NULL;

	file->fp = file->fp_tag = file->fp_tmp = file->fp_tag_tmp = NULL;
	strcpy(file->path, path);
	if ((p = strstr(file->path, "::"))) *p = '\0';
	file->openpath[0] = '\0';
	file->mode = mode;
	file->timetag = timetag;
	file->repmode = 0;
	file->offset = 0;
	file->time = file->wtime = time0;
	file->tick = file->tick_f = file->fpos = 0;
	file->start = start;
	file->speed = speed;
	file->swapintv = swapintv;
	initlock(&file->lock);

	time = utc2gpst(timeget());

	/* open new file */
	if (!openfile_(file, time, msg)) {
		free(file);
		return NULL;
	}
	return file;
}

extern void strinit(stream_t *stream)
{
//	tracet(3, "strinit:\n");

	stream->type = 0;
	stream->mode = 0;
	stream->state = 0;
	stream->inb = stream->inr = stream->outb = stream->outr = 0;
	stream->tick = stream->tact = stream->inbt = stream->outbt = 0;
	initlock(&stream->lock);
	stream->port = NULL;
	stream->path[0] = '\0';
	stream->msg[0] = '\0';
}

/* open stream -----------------------------------------------------------------
* open stream for read or write
* args   : stream_t *stream IO  stream
*          int type         I   stream type (STR_SERIAL,STR_FILE,STR_TCPSVR,...)
*          int mode         I   stream mode (STR_MODE_???)
*          char *path       I   stream path (see below)
* return : status (0:error,1:ok)
* notes  : see reference [1] for NTRIP
*          STR_FTP/HTTP needs "wget" in command search paths
*
* stream path ([] options):
*
*   STR_SERIAL   port[:brate[:bsize[:parity[:stopb[:fctr]]]]]
*                    port  = COM?? (windows), tty??? (linuex, omit /dev/)
*                    brate = bit rate     (bps)
*                    bsize = bit size     (7|8)
*                    parity= parity       (n|o|e)
*                    stopb = stop bits    (1|2)
*                    fctr  = flow control (off|rts)
*   STR_FILE     file_path[::T][::+start][::xseppd][::S=swap]
*                    ::T   = enable time tag
*                    start = replay start offset (s)
*                    speed = replay speed factor
*                    swap  = output swap interval (hr) (0: no swap)
*   STR_TCPSVR   :port
*   STR_TCPCLI   address:port
*   STR_NTRIPSVR user[:passwd]@address[:port]/moutpoint[:string]
*   STR_NTRIPCLI [user[:passwd]]@address[:port][/mountpoint]
*   STR_FTP      [user[:passwd]]@address/file_path[::T=poff[,tint[,toff,tret]]]]
*   STR_HTTP     address/file_path[::T=poff[,tint[,toff,tret]]]]
*                    poff  = time offset for path extension (s)
*                    tint  = download interval (s)
*                    toff  = download time offset (s)
*                    tret  = download retry interval (s) (0:no retry)
*-----------------------------------------------------------------------------*/
extern int stropen(stream_t *stream, int type, int mode, const char *path)
{
//	tracet(3, "stropen: type=%d mode=%d path=%s\n", type, mode, path);

	stream->type = type;
	stream->mode = mode;
	strcpy(stream->path, path);
	stream->inb = stream->inr = stream->outb = stream->outr = 0;
	stream->tick = tickget();
	stream->inbt = stream->outbt = 0;
	stream->msg[0] = '\0';
	stream->port = NULL;
	switch (type) {
	case STR_SERIAL: stream->port = openserial(path, mode, stream->msg); break;
	case STR_FILE: stream->port = openfile(path, mode, stream->msg); break;
	default: stream->state = 0; return 1;
	}
	stream->state = !stream->port ? -1 : 1;
	return stream->port != NULL;
}

/* read serial ---------------------------------------------------------------*/
static int readserial(serial_t *serial, unsigned char *buff, int n, char *msg)
{
#ifdef WIN32
	DWORD nr;
#else
	int nr;
#endif
//	tracet(4, "readserial: dev=%d n=%d\n", serial->dev, n);
	if (!serial) return 0;
#ifdef WIN32
	if (!ReadFile(serial->dev, buff, n, &nr, NULL)) return 0;
#else
	if ((nr = read(serial->dev, buff, n))<0) return 0;
#endif
//	tracet(5, "readserial: exit dev=%d nr=%d\n", serial->dev, nr);
	return nr;
}

static int readfile(file_t *file, unsigned char *buff, int nmax, char *msg)
{
	unsigned int nr = 0, t, tick, fpos;

//	tracet(4, "readfile: fp=%d nmax=%d\n", file->fp, nmax);

	if (!file) return 0;

	if (file->fp_tag) {
		if (file->repmode) { /* slave */
			t = (unsigned int)(tick_master + file->offset);
		}
		else { /* master */
			t = (unsigned int)((tickget() - file->tick)*file->speed + file->start*1000.0);
		}
		for (;;) { /* seek file position */
			if (fread(&tick, sizeof(tick), 1, file->fp_tag)<1 ||
				fread(&fpos, sizeof(fpos), 1, file->fp_tag)<1) {
				fseek(file->fp, 0, SEEK_END);
				sprintf(msg, "end");
				break;
			}
			if (file->repmode || file->speed>0.0) {
				if ((int)(tick - t)<1) continue;
			}
			if (!file->repmode) tick_master = tick;

			sprintf(msg, "T%+.1fs", (int)tick<0 ? 0.0 : (int)tick / 1000.0);

			if ((int)(fpos - file->fpos) >= nmax) {
				fseek(file->fp, fpos, SEEK_SET);
				file->fpos = fpos;
				return 0;
			}
			nmax = (int)(fpos - file->fpos);

			if (file->repmode || file->speed>0.0) {
				fseek(file->fp_tag, -(long)sizeof(tick) * 2, SEEK_CUR);
			}
			break;
		}
	}
	if (nmax>0) {
		nr = fread(buff, 1, nmax, file->fp);
		file->fpos += nr;
		if (nr <= 0) sprintf(msg, "end");
	}
//	tracet(5, "readfile: fp=%d nr=%d fpos=%d\n", file->fp, nr, file->fpos);
	return (int)nr;
}

extern int strread(stream_t *stream, unsigned char *buff, int n)
{
	unsigned int tick;
	char *msg = stream->msg;
	int nr;

//	tracet(4, "strread: n=%d\n", n);

	if (!(stream->mode&STR_MODE_R) || !stream->port) return 0;

	strlock(stream);

	switch (stream->type) {
	case STR_SERIAL: nr = readserial((serial_t *)stream->port, buff, n, msg); break;
	case STR_FILE: nr = readfile((file_t   *)stream->port, buff, n, msg); break;
	default:
		strunlock(stream);
		return 0;
	}
	stream->inb += nr;
	tick = tickget(); if (nr>0) stream->tact = tick;

	if ((int)(tick - stream->tick) >= tirate)
	{
		stream->inr = (stream->inb - stream->inbt) * 8000 / (tick - stream->tick);
		stream->tick = tick; stream->inbt = stream->inb;
	}
	strunlock(stream);
	return nr;
}

static void closeserial(serial_t *serial)
{
//	tracet(3, "closeserial: dev=%d\n", serial->dev);

	if (!serial) return;
#ifdef WIN32
	serial->state = 0;
	WaitForSingleObject(serial->thread, 10000);
	CloseHandle(serial->dev);
	CloseHandle(serial->thread);
#else
	close(serial->dev);
#endif
	free(serial);
}

static void closefile_(file_t *file)
{
//	tracet(3, "closefile_: path=%s\n", file->path);

	if (file->fp) fclose(file->fp);
	if (file->fp_tag) fclose(file->fp_tag);
	if (file->fp_tmp) fclose(file->fp_tmp);
	if (file->fp_tag_tmp) fclose(file->fp_tag_tmp);
	file->fp = file->fp_tag = file->fp_tmp = file->fp_tag_tmp = NULL;
}

/* close file ----------------------------------------------------------------*/
static void closefile(file_t *file)
{
//	tracet(3, "closefile: fp=%d\n", file->fp);

	if (!file) return;
	closefile_(file);
	free(file);
}

extern void strclose(stream_t *stream)
{
//	tracet(3, "strclose: type=%d mode=%d\n", stream->type, stream->mode);

	if (stream->port) {
		switch (stream->type) {
		case STR_SERIAL: closeserial((serial_t *)stream->port); break;
		case STR_FILE: closefile((file_t   *)stream->port); break;
		}
	}
	else {
//		trace(2, "no port to close stream: type=%d\n", stream->type);
	}
	stream->type = 0;
	stream->mode = 0;
	stream->state = 0;
	stream->inr = stream->outr = 0;
	stream->path[0] = '\0';
	stream->msg[0] = '\0';
	stream->port = NULL;
}