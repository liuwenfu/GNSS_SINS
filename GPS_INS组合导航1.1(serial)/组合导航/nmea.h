#pragma once

#define NMEA_DEF_PARSEBUFF  (1024)
#define NMEA_MIN_PARSEBUFF  (256)
#define NMEA_CONVSTR_BUF    (256)
#define NMEA_TIMEPARSE_BUF  (256)

#define NMEA_MAXSAT         (36)

#define NMEA_TOKS_COMPARE   (1)
#define NMEA_TOKS_PERCENT   (2)
#define NMEA_TOKS_WIDTH     (3)
#define NMEA_TOKS_TYPE      (4)

#define NMEA_SATINPACK      (4)

#define NMEA_SIG_BAD        (0)
#define NMEA_SIG_LOW        (1)
#define NMEA_SIG_MID        (2)
#define NMEA_SIG_HIGH       (3)

#define NMEA_FIX_BAD        (1)
#define NMEA_FIX_2D         (2)
#define NMEA_FIX_3D         (3)

#define NMEA_TUD_KNOTS      (1.852)         /**< Knots, kilometer / NMEA_TUD_KNOTS = knot */

typedef struct _nmeaPARSER
{
	void *top_node;
	void *end_node;
	unsigned char *buffer;
	int buff_size;
	int buff_use;

} nmeaPARSER;

typedef struct _nmeaParserNODE
{
	int packType;
	void *pack;
	struct _nmeaParserNODE *next_node;

} nmeaParserNODE;

typedef void(*nmeaTraceFunc)(const char *str, int str_size);
typedef void(*nmeaErrorFunc)(const char *str, int str_size);
typedef struct _nmeaPROPERTY
{
	nmeaTraceFunc   trace_func;
	nmeaErrorFunc   error_func;
	int             parse_buff_size;

} nmeaPROPERTY;

enum nmeaPACKTYPE
{
	GPNON = 0x0000,   /**< Unknown packet type. */
	GPGGA = 0x0001,   /**< GGA - Essential fix data which provide 3D location and accuracy data. */
	GPGSA = 0x0002,   /**< GSA - GPS receiver operating mode, SVs used for navigation, and DOP values. */
	GPGSV = 0x0004,   /**< GSV - Number of SVs in view, PRN numbers, elevation, azimuth & SNR values. */
	GPRMC = 0x0008,   /**< RMC - Recommended Minimum Specific GPS/TRANSIT Data. */
	GPVTG = 0x0010    /**< VTG - Actual track made good and speed over ground. */
};

typedef struct _nmeaTIME
{
	int     year;       /**< Years since 1900 */
	int     mon;        /**< Months since January - [0,11] */
	int     day;        /**< Day of the month - [1,31] */
	int     hour;       /**< Hours since midnight - [0,23] */
	int     minute;        /**< Minutes after the hour - [0,59] */
	int     second;        /**< Seconds after the minute - [0,59] */
	int     hsec;       /**< Hundredth part of second - [0,99] */

} nmeaTIME;

typedef struct _nmeaSATELLITE
{
	int     id;         /**< Satellite PRN number */
	int     in_use;     /**< Used in position fix */
	int     elv;        /**< Elevation in degrees, 90 maximum */
	int     azimuth;    /**< Azimuth, degrees from true north, 000 to 359 */
	int     sig;        /**< Signal, 00-99 dB */

} nmeaSATELLITE;


typedef struct _nmeaSATINFO
{
	int     inuse;      /**< Number of satellites in use (not those in view) */
	int     inview;     /**< Total number of satellites in view */
	nmeaSATELLITE sat[NMEA_MAXSAT]; /**< Satellites information */

} nmeaSATINFO;

typedef struct _nmeaINFO
{
	int     smask;      /**< Mask specifying types of packages from which data have been obtained */

	nmeaTIME utc;       /**< UTC of position */

	int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
	int     fix;        /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */

	double  PDOP;       /**< Position Dilution Of Precision */
	double  HDOP;       /**< Horizontal Dilution Of Precision */
	double  VDOP;       /**< Vertical Dilution Of Precision */

	double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
	double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
	double  elv;        /**< Antenna altitude above/below mean sea level (geoid) in meters */
	double  speed;      /**< Speed over the ground in kilometers/hour */
	double  direction;  /**< Track angle in degrees True */
	double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */

	nmeaSATINFO satinfo; /**< Satellites information */

	int timeflag;

} nmeaINFO;

typedef struct _nmeaGPGGA
{
	nmeaTIME utc;       /**< UTC of position (just time) */
	double  lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
	char    ns;         /**< [N]orth or [S]outh */
	double  lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
	char    ew;         /**< [E]ast or [W]est */
	int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
	int     satinuse;   /**< Number of satellites in use (not those in view) */
	double  HDOP;       /**< Horizontal dilution of precision */
	double  elv;        /**< Antenna altitude above/below mean sea level (geoid) */
	char    elv_units;  /**< [M]eters (Antenna height unit) */
	double  diff;       /**< Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level. '-' = geoid is below WGS-84 ellipsoid) */
	char    diff_units; /**< [M]eters (Units of geoidal separation) */
	double  dgps_age;   /**< Time in seconds since last DGPS update */
	int     dgps_sid;   /**< DGPS station ID number */

} nmeaGPGGA;

typedef struct _nmeaGPGSA
{
	char    fix_mode;   /**< Mode (M = Manual, forced to operate in 2D or 3D; A = Automatic, 3D/2D) */
	int     fix_type;   /**< Type, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */
	int     sat_prn[NMEA_MAXSAT]; /**< PRNs of satellites used in position fix (null for unused fields) */
	double  PDOP;       /**< Dilution of precision */
	double  HDOP;       /**< Horizontal dilution of precision */
	double  VDOP;       /**< Vertical dilution of precision */

} nmeaGPGSA;

typedef struct _nmeaGPGSV
{
	int     pack_count; /**< Total number of messages of this type in this cycle */
	int     pack_index; /**< Message number */
	int     sat_count;  /**< Total number of satellites in view */
	nmeaSATELLITE sat_data[NMEA_SATINPACK];

} nmeaGPGSV;

typedef struct _nmeaGPRMC
{
	nmeaTIME utc;       /**< UTC of position */
	char    status;     /**< Status (A = active or V = void) */
	double  lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
	char    ns;         /**< [N]orth or [S]outh */
	double  lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
	char    ew;         /**< [E]ast or [W]est */
	double  speed;      /**< Speed over the ground in knots */
	double  direction;  /**< Track angle in degrees True */
	double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */
	char    declin_ew;  /**< [E]ast or [W]est */
	char    mode;       /**< Mode indicator of fix type (A = autonomous, D = differential, E = estimated, N = not valid, S = simulator) */

} nmeaGPRMC;

typedef struct _nmeaGPVTG
{
	double  dir;        /**< True track made good (degrees) */
	char    dir_t;      /**< Fixed text 'T' indicates that track made good is relative to true north */
	double  dec;        /**< Magnetic track made good */
	char    dec_m;      /**< Fixed text 'M' */
	double  spn;        /**< Ground speed, knots */
	char    spn_n;      /**< Fixed text 'N' indicates that speed over ground is in knots */
	double  spk;        /**< Ground speed, kilometers per hour */
	char    spk_k;      /**< Fixed text 'K' indicates that speed over ground is in kilometers/hour */

} nmeaGPVTG;

extern int nmea_parser_init(nmeaPARSER *parser);
extern int nmea_parser_pop(nmeaPARSER *parser, void **pack_ptr);
extern int nmea_parse(nmeaPARSER *parser, const unsigned char *buff, int buff_sz, nmeaINFO *info);
