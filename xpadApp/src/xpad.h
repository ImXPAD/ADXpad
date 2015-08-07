#ifndef xpad_H
#define xpad_H
/**<xpad.h
 * 
 * This is a driver for any XPAD detector.
 * It uses a TCP/IP socket to communicate with the XpadXXXServer in version 3.x XXX being either PCI or USB.
 * It can set most of the parameter the detector (and thus server) has to offer.
 *
 * 
 * Author: Nicolas Delrio       nclsdlr(at)gmail.com
 * 			 			@ ImXPAD
 * Based on  
 * 			Mark Rivers
 *        			 University of Chicago
 * 					And the EPICS community's  
 * 										work
 *
 * Created:  June 20, 2015
 * 
 * 
 * 
 * 
 *VERSION 1.0.0 aka. Oirled
 *
 */
 /**Random infos:
  * 
  * Timeouts on the asyn interface are in seconds 
  * 
  * */

#include <stdint.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>
#include <asynOctet.h>
#include <epicsExport.h>



#include "ADDriver.h"


//You can change any of the folowing define
///Size of the commands sent
#define MAX_MESSAGE_SIZE 256
///For calibration only
#define MAX_OCT_BUFF_SIZE 4096
///Lenght of filepaths and filenames
#define MAX_FILENAME_LEN 256
#define XPAD_SOCKET_TIMEOUT 1.0
///Waiting time for simple commands
/**30 seconds is the real server command timeout*/
#define XPAD_COMMAND_TIMEOUT 30.0
///Waiting time in polling loops
/**SHorter for fast networks */
#define XPAD_POLL_DELAY .02
///Should never be too small 
/**The bigger the better tested with values between 256 and 15 000*/
#define MAX_RETURN_SIZE 4096
///Time added to theoretical times, particularly usefull for slow networks
#define XPAD_SUPP_DELAY 10 


/// Aquisition mode choices
typedef enum {

	xmode_aquire,
	xmode_calib,
	xmode_scalib,
	xmode_config,
	xmode_loadingcalib,
	xmode_white,
	digitaltest,
	xmode_idle,
	xmode_changin,
	xmode_reset,
	xmode_prompts,
	xmode_promptr
} xpad_mode;


typedef enum {
	AM_standard,
	AM_detector_burst,
	AM_comupter_burst,
	AM_stacking_16b,
	AM_stacking_32b,
	AM_single_bunch_16b,
	AM_single_bunch_32b,
} xpad_acquisition_mode;


///brief Trigger mode choices 
typedef enum {
    IS_internal = ADTriggerInternal,
    IS_external = ADTriggerExternal,
    IS_external_multiple,
    IS_external_single
} xpad_input_signal;

/// type of server returns 
typedef enum {
	OS_exposure_busy,
	OS_shutter_busy,
	OS_busy_update_overflow,
	OS_pixel_counter_on,
	OS_external_gate,
	OS_exposure_read_done,
	OS_data_transfer,
	OS_RAMready_imgbusy,
	OS_to_localDDR,
	OS_localDDR_to_PC
} xpad_output_signal;
typedef enum {
	UNPACK_CALIB,
	UNPACK_DETECTOR_SIZE,
	UNPACK_QUOTE,
	UNPACK_LIST
} unpack_mode;
/// Status choices 
typedef enum {
    xpadStatusIdle,
    xpadStatusExpose,
    xpadStatusScan,
    xpadStatusErase,
    xpadStatusChangeMode,
    xpadStatusAborting,
    xpadStatusError,
    xpadStatusWaiting,
    xpadStatusConfigorCalib
} xpadStatus_t;



/** Driver-specific parameter strings for the xpad driver */

#define xpadChangeModeString    "XPAD_CHANGE_MODE"

#define xpadAbortString         "XPAD_ABORT"
#define xpad_acq_modeString "XPAD_ACQUISITION_MODE" 
#define xpad_stacksizeString "XPAD_STACKSIZE" 
#define xpad_geo_correctionString "XPAD_GEOMETRICAL_CORRECTION_FLAG"
#define xpad_flat_fieldString "XPAD_FLAT_FIELD_CORRECTION_FLAG"
#define xpad_img_transferString "XPAD_IMAGE_TRANSFER_FLAG"
#define xpad_filepathString "XPAD_FILEPATH"
#define xpad_load_calibString "XPAD_LOAD_CALIB"
#define xpad_save_calibString "XPAD_SAVE_CALIB"
#define xpad_outputString "XPAD_OUTPUT_SIGNAL"
#define xpad_overflowString "XPAD_OVERFLOW_TIME"
#define xpad_outformatString "XPAD_OUTPUT_FORMAT"
#define xpad_outpathString "XPAD_OUTPUT_SERVER_FILEPATH"
#define xpad_white_imageString "XPAD_WHITE_IMAGE_ACTIVATOR"
#define xpad_chose_whiteString "XPAD_CHOSE_WHITE"
#define xpad_show_whiteString "XPAD_SHOW_WHITE"
#define xpad_sendString "XPAD_SEND_TOSERVER"
#define xpad_whitepathString "XPAD_WHITEPATH"
#define xpad_readString "XPAD_READ_FROMSERVER"
#define xpad_resetString "XPAD_RESET"
#define xpad_otnString "XPAD_OTN"
#define xpad_otn_pulseString "XPAD_OTN_PULSE"
#define xpad_beamcalib_timeString "XPAD_BEAMCALIB_TIME"
#define xpad_ITHL_maxString "XPAD_ITHL_MAX"
#define xpad_speedString "XPAD_SPEED"
#define xpad_beamString "XPAD_BEAM"


static const char *driverName = "xpad";

/** Driver for xpad online pixel array detectors; communicates with the XpadPCIServer and XPadUSBServer programs 
  * over a TCP/IP socket .
  * The XPadXXXServer program must be running listening for commands on a
  * given port.  This is done by running it on a specified port f.e.
  *  "XpadPCIServer 3456" 
  * In this example 3456 is the TCP/IP port number that the XpadXXXServer and this driver will use to
  * communicate.
  * If you are running the server on another port please change the value in the st.cmd file
  */
class xpad : public ADDriver {
public:
    xpad(const char *portName, const char *xpadPort,int maxBuffers, size_t maxMemory,int priority, int stackSize);
                 
    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual void report(FILE *fp, int details);
    epicsEventId startEventId; /**< Should be private but accessed from C, must be public */
    epicsEventId stopEventId; 
    epicsEventId abortEventId; 
    void xpadTask();
    void xpadAbortTask();   


protected:
//activation flags (medm button f e)
	int xpadChangeMode;
	#define FIRST_XPAD_PARAM xpadChangeMode
	int xpad_load_calib;
	int xpad_save_calib;
	int xpad_white_image;
	int xpad_chose_white;
	int xpad_show_white;
	int xpad_reset;
	int xpadAbort;
	int xpad_send;
	int xpad_read;
	int xpad_beam;
	int xpad_otn;
	int xpad_otn_pulse;
	

///Name of the white image file
	int xpad_whitepath;
///Output server filepath 
    int xpad_outpath;
///Calibration filepath
	int xpad_filepath;


	
	//Exposure parameters (which are not already in ADDriver.h)
	
	///Acquisition mode
	int xpad_acq_mode;
    ///Toggle geometrical correction: default 0
	int xpad_geo_correction;
    ///Toggle flat field correction: default 0
    int xpad_flat_field;
   ///image transfer flag 
   /** At the end of exposure if set to true the images will be send in binary via tcp
     *  if set to 0 they will be saved in "output server filepath" */
    int xpad_img_transfer;
    ///Output signal
    int xpad_output;
    int xpad_overflow;
    ///Basicaly useless for epics users
    int xpad_outformat;
    ///Stacksize for stacking mode
    int xpad_stacksize;

//Calib param
	int xpad_beamcalib_time;
	int xpad_ITHL_max;
	int xpad_speed;
	
	    #define LAST_XPAD_PARAM xpad_speed

private:      
	asynStatus unpackServer(char* input	,char * output,int mode,int param);
	asynStatus unpackServer(int mode);
    asynStatus writeServer(const char *output);
    asynStatus readServer(char *input, size_t maxChars, double timeout);
    asynStatus waitForCompletion(const char *doneString,char * returnstr, double timeout);
    asynStatus loadConfigFromFile(const char * fileName);
	asynStatus saveConfigToFile( const char * fileName);
	asynStatus createWhiteImage(char *);
    asynStatus setExposureParameters(void);
    asynStatus changeMode(void);
    asynStatus getImageStream(void);
	asynStatus xpadInit();

    /* data treatment  */
    epicsTimerId timerId;
    xpad_mode mode;
    char toServer[MAX_MESSAGE_SIZE];
    char fromServer[MAX_RETURN_SIZE] ;
    asynUser *pasynUserServer;
    asynUser *pasynAbortServer;
    bool ready;
    char serverPort[32];
    epicsThreadId mainTask;
	epicsThreadId abortTask;

};

#define NUM_XPAD_PARAMS ((int)(&LAST_XPAD_PARAM - &FIRST_XPAD_PARAM + 1))

#endif
