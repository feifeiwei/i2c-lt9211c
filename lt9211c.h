
#ifndef _TYPE_H
#define _TYPE_H
 
//Variable Type Definition
typedef 	char 			    CHAR, *PCHAR ;
typedef 	unsigned char 		uchar, *puchar ;
typedef 	unsigned char 		UCHAR, *PUCHAR ;
typedef 	unsigned char 		byte, *pbyte ;
typedef 	unsigned char 		BYTE, *PBYTE ;

typedef 	short 			    SHORT, *PSHORT ;
typedef 	unsigned short 		ushort, *pushort ;
typedef 	unsigned short 		USHORT, *PUSHORT ;
typedef 	unsigned short 		word, *pword ;
typedef 	unsigned short 		WORD, *PWORD ;

typedef 	long 			    LONG, *PLONG ;
typedef 	unsigned long 		ulong, *pulong ;
typedef 	unsigned long 		ULONG, *PULONG ;
typedef 	unsigned long 		dword, *pdword ;
typedef 	unsigned long 		DWORD, *PDWORD ;

typedef 	unsigned int 		UINT, uint;
//Public Definition
#define IN
#define OUT
#define INOUT


//state
#define     LOW                        0
#define     HIGH                       1
#define     OFF                        0
#define     ON                         1
#define     LED_ON                     0
#define     LED_OFF                    1
#define     ENABLED                    1
#define     DISABLED                   0
#define     NEGITVE                    0
#define     POSITIVE                   1
#define     FAIL                       0
#define     SUCCESS                    1

#define     UNSTABLE                   0
#define     STABLE                     1
#define     STABLE_UNKNOWN             2

#define 	LVDS_IN_LVDS_OUT	 	   0
#define 	LVDS_IN_MIPI_OUT	 	   1
#define 	LVDS_IN_TTL_OUT	 	       2
#define 	MIPI_IN_LVDS_OUT	 	   3
#define 	MIPI_IN_MIPI_OUT	 	   4
#define 	MIPI_IN_TTL_OUT	 	       5
#define 	TTL_IN_LVDS_OUT	 	       6
#define 	TTL_IN_MIPI_OUT	 	       7
#define 	TTL_IN_TTL_OUT	 	       8
#define     MIPI_REPEATER              9   
#define     MIPI_LEVEL_SHIFT           10
#define     PATTERN_OUT                11

#define     NO_TX_PATTERN              0
#define     MIPITX_PATTERN             1
#define     LVDSTX_PATTERN             2
#define     TTLTX_PATTERN              3

#define   RGB                          0
#define   YUV422                       1
#define   YUV444                       2
#define   YUV420                       3

#define 	P_MODE 	                   0
#define 	I_MODE	 	               1

#define 	DEEP_8     	 	           8
#define 	DEEP_10   	 	           10
#define 	DEEP_12   	               12

//---------------------------LVDS MODE------------------------
#define 	LVDS_No_SSC	 	           0
#define 	LVDS_W_SSC                 1

#define     FOUR_LANE                     0
#define     FIVE_LANE                     1

#define     DE_MODE                       0
#define     SYNC_MODE                     1

#define     VESA                          0
#define     JEIDA                         1

#define     DEPTH_6BIT                    0
#define     DEPTH_8BIT                    1
#define     DEPTH_10BIT                   2
#define     DEPTH_12BIT                   3

#define     P_FORMAT                      0
#define     I_FORMAT                      1

#define     NON_REPECTIVE                 0
#define     REPECTIVE                     1

#define     VID_640x480_60Hz            0
#define     VID_720x480_60Hz            1
#define     VID_1280x720_60Hz           2
#define     VID_1366x768_60Hz           3
#define     VID_1280x720_30Hz           4
#define     VID_1920x720_60Hz           5
#define     VID_1920x1080_30Hz          6
#define     VID_1920x1080_60Hz          7
#define     VID_1920x1200_60Hz          8
#define     VID_3840x2160_30Hz          9

#define     NO_SSC                        0
#define     SSC_1920x1080_30k5         1
#define     SSC_3840x2160_30k5         2

//---------------------------MIPI MODE------------------------
#define     MIPI_DSI                 0
#define     MIPI_CSI                 1

#define     MIPI_DPHY                0
#define     MIPI_CPHY                1

#define 	MIPITX_DSC                 0
#define 	MIPITX_CSC	 	           1

#define     PORTA                      0
#define     PORTB                      1
#define     DOU_PORT                   2
#define     NO_COPY                    3

#define     MIPI_NEGITVE               1
#define     MIPI_POSITIVE              0

#define     MIPITX_PLL_LOW             400000
#define     MIPITX_PLL_HIGH            2700000

#define     CTS_DATARATE               1900000


//MIPI LEVEL SHIFT
#define     RX_LS                       0
#define     TX_LS                       1

#define     HS_PORTA_LP_PORTB           0
#define     LP_PORTA_HS_PORTB           1

//---------------------------TTL MODE------------------------
#define     SDR                 0
#define     DDR                 1

#define     BT1120              0
#define     BTA_T1004           1
#define     BT656               2

#define     PLUS                0
#define     MINUS               1

#define     INPUT_RGB888               0
#define     INPUT_RGB666               1
#define     INPUT_RGB565               2
#define     INPUT_YCBCR444             3
#define     INPUT_YCBCR422_8BIT        4
#define     INPUT_YCBCR422_10BIT       5
#define     INPUT_YCBCR422_12BIT       6
#define     INPUT_YCBCR422_16BIT       7
#define     INPUT_YCBCR422_20BIT       8
#define     INPUT_YCBCR422_24BIT       9
#define     INPUT_BT656_8BIT           10
#define     INPUT_BT656_10BIT          11
#define     INPUT_BT656_12BIT          12
#define     INPUT_BT1120_8BIT          13
#define     INPUT_BT1120_10BIT         14
#define     INPUT_BT1120_12BIT         15
#define     INPUT_BTA_T1004_16BIT      16
#define     INPUT_BTA_T1004_20BIT      17
#define     INPUT_BTA_T1004_24BIT      18
#define     INPUT_BT1120_16BIT         19
#define     INPUT_BT1120_20BIT         20
#define     INPUT_BT1120_24BIT         21

#define     OUTPUT_RGB888               0
#define     OUTPUT_RGB666               1
#define     OUTPUT_RGB565               2
#define     OUTPUT_YCBCR444             3
#define     OUTPUT_YCBCR422_8BIT        4
#define     OUTPUT_YCBCR422_10BIT       5
#define     OUTPUT_YCBCR422_12BIT       6
#define     OUTPUT_YCBCR422_16BIT       7
#define     OUTPUT_YCBCR422_20BIT       8
#define     OUTPUT_YCBCR422_24BIT       9
#define     OUTPUT_BT656_8BIT           10
#define     OUTPUT_BT656_10BIT          11
#define     OUTPUT_BT656_12BIT          12
#define     OUTPUT_BT1120_8BIT          13
#define     OUTPUT_BT1120_10BIT         14
#define     OUTPUT_BT1120_12BIT         15
#define     OUTPUT_BTA_T1004_16BIT      16
#define     OUTPUT_BTA_T1004_20BIT      17
#define     OUTPUT_BTA_T1004_24BIT      18
#define     OUTPUT_BT1120_16BIT         19
#define     OUTPUT_BT1120_20BIT         20
#define     OUTPUT_BT1120_24BIT         21

#define BIT0_1  0x01
#define BIT1_1  0x02
#define BIT2_1  0x04
#define BIT3_1  0x08
#define BIT4_1  0x10
#define BIT5_1  0x20
#define BIT6_1  0x40
#define BIT7_1  0x80

#define BIT0_0  0xFE
#define BIT1_0  0xFD
#define BIT2_0  0xFB
#define BIT3_0  0xF7
#define BIT4_0  0xEF
#define BIT5_0  0xDF
#define BIT6_0  0xBF
#define BIT7_0  0x7F

struct lt9211_data {
    struct i2c_client *client;
    //int pwr_gpio; // no use
    int rst_gpio;

    unsigned int hact;
    unsigned int vact;
    unsigned int hbp;
    unsigned int hfp;
    unsigned int hs;
    unsigned int vbp;
    unsigned int vfp;
    unsigned int vs;
    unsigned int pclk;
    unsigned int htotal;
    unsigned int vtotal;

    unsigned int format;//lvds_format 0:VESA 1:JEIDA
    unsigned char mipi_lane;
    unsigned int color; //Color Depth 0:6bit 1:8bit
    unsigned int test;
};

//TX
typedef struct VidChkTiming
{
	u16 usHfp;
	u16 usHs;
	u16 usHbp;
	u16 usHact;
	u16 usHtotal;
	
	u16 usVfp;
	u16 usVs;
	u16 usVbp;
	u16 usVact;
	u16 usVtotal;
	
	u32 ulPclk_Khz;
	u8  ucFrameRate;
	
    u8 ucHspol;
    u8 ucVspol;
}StructVidChkTiming;

//RX

typedef enum
{
    MIPIRX_VIDEO_ON_EVENT = 1,
    MIPIRX_VIDEO_OFF_EVENT,
    MIPIRX_CSC_EVENT,
}EnumChipRxEvent;

typedef struct ChipRx
{
    u8 b1RxStateChanged;
    u8 b1VidChkScanFlg;
    u8 ucPixelEncoding;
    u8 ucRxFormat;
    u8 ucRxState;
    // void (*pHdmiRxNotify)(EnumChipRxEvent ucEvent);   // mf add
}StructChipRx;


typedef struct ChipRxVidTiming
{
    u16 usHfp;
    u16 usHs;
    u16 usHbp;
    u16 usHact;
    u16 usHtotal;
    
    u16 usVfp;
    u16 usVs;
    u16 usVbp;
    u16 usVact;
    u16 usVtotal;
    
    u32 ulPclk_Khz;
    u8  ucFrameRate;
}StructChipRxVidTiming;


//PCR setting
typedef struct
{
    u32 Pcr_M;
    u32 Pcr_K;
    u8  Pcr_UpLimit;
    u8  Pcr_DownLimit;
}StructPcrPara;


typedef enum
{
    LOG_DEBUG =0x00,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_CRITICAL,
    LOG_NOTRACE,
}LogLevel;

typedef enum
{
    MIPIRX_4LANE = 0x00,
    MIPIRX_1LANE = 0x01,
    MIPIRX_2LANE = 0x02,
    MIPIRX_3LANE = 0x03,
}Enum_MIPIRXPORTLANE_NUM;

typedef enum
{
    RXPLL_PIX_CLK     = 0x00,
    DESSCPLL_PIX_CLK  = 0x01,
    RXPLL_DEC_DDR_CLK = 0x02,
    MLRX_BYTE_CLK     = 0x03,
    
}Enum_VIDCHK_PIXCLK_SRC_SEL;

typedef enum
{
    RGB_6Bit = 0,
    RGB_8Bit,
    RGB_10Bit,
    RGB_12Bit,
    YUV444_8Bit,
    YUV444_10Bit,
    YUV422_8bit,
    YUV422_10bit,
    YUV422_12bit,
    YUV420_8bit,
    YUV420_10bit,
    MIPITX_FORMAT_CNT,
}Enum_MIPI_FORMAT;

#define MIPIRX_FORMAT_CNT   0x0f
char* g_szStrRxFormat[MIPIRX_FORMAT_CNT] = 
{
    "",
    "DSI YUV422 10bit",
    "DSI YUV422 12bit",
    "YUV422 8bit",
    "RGB 10bit",
    "RGB 12Bit",
    "YUV420 8bit",
    "RGB 565",
    "RGB 666",
    "DSI RGB 6L",
    "RGB 8Bit",
    "RAW8",
    "RAW10",
    "RAW12",
    "CSI YUV422 10",
};

typedef struct VideoTimingList
{
    u16 usHfp;
    u16 usHs;
    u16 usHbp;
    u16 usHact;
    u16 usHtotal;
    
    u16 usVfp;
    u16 usVs;
    u16 usVbp;
    u16 usVact;
    u16 usVtotal;
    
    u8  ucFrameRate;

};

StructPcrPara g_stPcrPara;




typedef  enum
{
    HTOTAL_POS    =    0,
    HACTIVE_POS,
    HFP_POS,
    HSW_POS,
    HBP_POS,
    
    VTOTAL_POS,
    VACTIVE_POS,
    VFP_POS,
    VSW_POS,
    VBP_POS,
    
    HSPOL_POS,
    VSPOL_POS,
}POS_INDEX;

typedef enum
{
    AD_MLTX_READ_CLK    = 0x08,   //0x08
    AD_MLTX_WRITE_CLK   = 0x09,   //0x09
    AD_DESSCPLL_PIX_CLK    = 0x10,   //0x10
    AD_RXPLL_PIX_CLK    = 0x1a,   //0x1a
    AD_DESSCPLL_PCR_CLK = 0x14,   //0x14
    AD_MLRX_BYTE_CLK    = 0x18,
}Enum_FM_CLK;

typedef enum
{
    LVDSRX  = 0x00,     
    MIPIRX  = 0x01,     //pcr recover video timing
    TTLRX   = 0x02,
    PATTERN = 0x03,
    LVDSDEBUG = 0x04,
    MIPIDEBUG = 0x05,
    TTLDEBUG  = 0x06,
    
}EnumChipRxSrc;

//MIPI RX Videotiming Debug
typedef struct VideoTiming_Get
{   
    unsigned short uswc;
    unsigned short usHact;
    unsigned short usVact;
    unsigned char ucFmt;
    unsigned char ucPa_Lpn;
    unsigned char ucFrameRate;
    unsigned char ucLane0SetNum;
    unsigned char ucLane1SetNum;
    unsigned char ucLane2SetNum;
    unsigned char ucLane3SetNum;
    unsigned char ucLane0SotData;
    unsigned char ucLane1SotData;
    unsigned char ucLane2SotData;
    unsigned char ucLane3SotData;
}SrtuctMipiRx_VidTiming_Get;

StructChipRx g_stChipRx;
StructChipRxVidTiming g_stRxVidTiming;
StructVidChkTiming g_stVidChk;

SrtuctMipiRx_VidTiming_Get g_stMipiRxVidTiming_Get;
//hfp    hs     hbp     hact     htotal   vfp   vs   vbp   vact    vtotal  framerate
struct VideoTimingList resolution[]  = {
 {8,     96,    40,     640,     800,     33,   2,   10,   480,    525,   60},  //video_640x480_60Hz
 {16,    62,    60,     720,     858,     9,    6,   30,   480,    525,   60},  //video_720x480_60Hz
 {12,    64,    88,     720,     864,     5,    5,   39,   576,    625,   50},  //video_720x576_50Hz
 {48,    128,   88,     800,     1056,    1,    4,   23,   600,    628,   60},  //video_800x600_60Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   30},  //video_1280x720_30Hz
 {440,   40,    220,    1280,    1980,    5,    5,   20,   720,    750,   50},  //video_1280x720_50Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   60},  //video_1280x720_60Hz
 {24,    136,   160,    1024,    1344,    3,    6,   29,   768,    806,   60},  //video_1024x768_60Hz
 {26,    110,   110,    1366,    1592,    13,   6,   13,   768,    800,   60},  //video_1366x768_60Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   30},  //video_1280x720_30Hz
 {48,    32,    80,     1920,    2080,    5,    5,   20,   720,    750,   60},  //video_1920x720_60Hz
 {48,    112,   248,    1280,    1688,    1,    3,   38,   1024,   1066,  60},  //video_1028x1024_60Hz
 {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,  30},  //video_1920x1080_30Hz
 {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,  50},  //video_1920x1080_60Hz
 {64,    192,   304,    1600,    2160,    1,    3,   46,   1200,   1250,  60},  //video_1600x1200_60Hz
 {48,    32,    80,     1920,    2080,    3,    6,   26,   1200,   1235,  60},  //video_1920x1200_60Hz
 {1276,  88,    296,    3840,    5500,    8,    10,  72,   2160,   2250,  24},  //video_3840x2160_24Hz
 {1056,  88,    296,    3840,    5280,    8,    10,  72,   2160,   2250,  25},  //video_3840x2160_25Hz
 {176,   88,    296,    3840,    4400,    8,    10,  72,   2160,   2250,  30},  //video_3840x2160_30Hz
 {1056,  88,    296,    3840,    5280,    8,    10,  72,   2160,   2250,  50},  //video_3840x2160_50Hz
 {176,   88,    296,    3840,    4400,    8,    10,  72,   2160,   2250,  60},  //video_3840x2160_60Hz

 };






// mf add

 typedef struct MipiTx
{
    u8 b1MipiClockburst;
    u8 b1DphyCsi8Lane;
    u8 ucTxFormat;    
    u8 ucBpp;
    u8 ucTxPortNum;
    u8 ucTxLaneNum;
    u32 ulMipiDataRate;
    u32 ulMipiInClk;
    u32 ulMipiByteClk;
//    void (*pMipiTxNotify)(EnumMipiTxEvent ucEvent);
}StructMipiTx;

StructMipiTx g_stMipiTx;


typedef enum
{
    HALF_WRITE_CLK = 0,
    WRITE_CLK,
}EnumWrClkSel;



//PHY timing :
typedef struct MipiTxDPhy
{
    u8 ucClkZero;      
    u8 ucClkPre;       
    u8 ucClkPost;      
    u8 ucHsLpx;        
    u8 ucHsPrep;       
    u8 ucHsTrail;          
    u8 ucHsRqStPre;
}StructMipiTxDPhy;
StructMipiTxDPhy g_stMipiTxDPhy;


#define LPT_DI  0x29

typedef enum
{
    LVDSTX  = 0x00,     
    MIPITX  = 0x01,
    
}EnumChipTxSel;

typedef enum
{
    SPort  = 0x01, // Single port : 1port
    DPorts = 0x02,// Dual ports : 2port
}Enum_PORT_NUM;



typedef enum
{

    MIPITX_1LANE = 1,
    MIPITX_2LANE = 2,
    MIPITX_3LANE = 3,
    MIPITX_4LANE = 4,
    MIPITX_8LANE = 8,
}Enum_PORTLANE_NUM;


#endif