/**<xpad.cpp
 * 
 * This is a driver for any XPAD detector.
 * It uses a TCP/IP socket to communicate with the XpadXXXServer in version 2.x and 3.0 XXX being either PCI or USB.
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
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

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


#define MAX_MESSAGE_SIZE 256
#define MAX_OCT_BUFF_SIZE 256
#define MAX_FILENAME_LEN 256
#define XPAD_SOCKET_TIMEOUT 1.0
#define XPAD_COMMAND_TIMEOUT 30.0
#define XPAD_POLL_DELAY .02
#define MAX_RETURN_SIZE 256
///NON NUL
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

    #define LAST_XPAD_PARAM xpad_stacksize

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


///Start the exposure with the previously given parameters catch the image sent by the server
asynStatus xpad::getImageStream()
{
	/**
	 * Set of first parameters
	 * */    
	const char *functionName = "getImageStream";

	uint32_t dataLen;
	int abort, ival=0,trigger, nCopied, eomReason,maxRead, headerLen,img_rest,imgtot=0;
    double dval,eval, timeout=0;
    NDArray *pImage=NULL;
    epicsTimeStamp now;
    asynStatus status=asynSuccess;
    char *pOut=NULL;
    char *pIn;
	size_t nRead_=0, nWrite_=0,dims[2];
    asynUser *pasynUser = this->pasynUserServer;
    
    
	getIntegerParam(ADNumImages,&img_rest);

    headerLen = 12;//12;    
	getDoubleParam(ADAcquireTime,&dval);
	getDoubleParam(ADAcquirePeriod,&eval);
	if (dval>eval){
		eval=dval;
	}
	epicsSnprintf(this->toServer, sizeof(this->toServer),"StartExposure");
	/**
	 * Start the exposure
	 * and wait for the server resposne
	 * 
	 */

	getIntegerParam(ADTriggerMode,&trigger);
	timeout=1.06*eval+XPAD_SUPP_DELAY+1;
	if(trigger!=IS_internal)asynPrint(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s%s:WARNING: trigger enabled, the driver will wait forever if the image is not sent. \n",driverName,functionName);


	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s%s:gonna writeread with %lf s timeout \n",driverName,functionName,timeout);
	setStringParam(ADStringToServer,toServer);
	setIntegerParam(ADStatus,xpadStatusExpose);
	callParamCallbacks();
	getIntegerParam(ADStatus, &abort);
	if(abort==ADStatusAborted || abort==xpadStatusAborting)return asynError;
unlock();

	status=pasynOctetSyncIO->writeRead(pasynUser,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),timeout,&nWrite_, &nRead_, &eomReason);
	while(trigger!=IS_internal && nRead_<=3){
		getIntegerParam(ADStatus, &abort);
		if(abort!=ADStatusAborted && abort!=xpadStatusAborting)pasynOctetSyncIO->read(pasynUser, fromServer, sizeof(fromServer),10, &nRead_, &eomReason);
		else break;
	}
lock();	
		

	 
	 
	 if(eomReason==2){// eomReason="\r"(=0x0a=10) this means the pasynOctetSyncIO interface stopped reading from the server because of a "\r" char so we put back this char because ..well we are not really dealing with chars. a uint32_t version of this function would be much welcome 
		//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "\n eom \n");
		fromServer[nRead_]=(int32_t)10;
		nRead_++;
	}
    if(strncmp(">",fromServer,1)==0){
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s%s:  >   recieved, readout shift 2 bytes \n",driverName,functionName);

		ival=2;
		headerLen=14;//14;
	}
	
	/**
	 * We use the first recieved bytes to set and allocate the logical structure that will serve as a buffer for our image
	 * 
	 * Conversion from recieved octets to 32bits integers
	 * 
	 * the image is streamed that way: (datasize)(width)(height)(data) :: (uint32)(uint32)(uint32)((uint32)*width*height)
	 */ 
	dataLen=(uint8_t)fromServer[ival+3]<<24|(uint8_t)fromServer[ival+2]<<16|(uint8_t)fromServer[ival+1]<<8|(uint8_t)fromServer[ival+0];
	dims[1]=(size_t)((uint8_t)fromServer[ival+7]<<24|(uint8_t)fromServer[ival+6]<<16|(uint8_t)fromServer[ival+5]<<8|(uint8_t)fromServer[ival+4]);
	dims[0]=(size_t)((uint8_t)fromServer[ival+11]<<24|(uint8_t)fromServer[ival+10]<<16|(uint8_t)fromServer[ival+9]<<8|(uint8_t)fromServer[ival+8]);
	//dataLen=dims[0]*dims[1]*4;
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s%s: Image dimensions  %d*%d \nimagesize:%d \n",driverName,functionName,(int)dims[0],(int)dims[1],dataLen);
	if ((dims[0] <= 0) || (dims[1] <= 0)){	//||dataLen!=(int)(dims[0]*dims[1]*4)){
		callParamCallbacks();
		return asynError;
	}



	if (nRead_==0) return asynError;

	else{
		setIntegerParam(NDArraySizeY,dims[1]);
		setIntegerParam(NDArraySizeX, dims[0]);
		setIntegerParam(NDArraySize, dataLen);
		pImage = this->pNDArrayPool->alloc(2, dims, NDInt32, 0, NULL);
	}
			


    while(img_rest>0){
		//reception premiere trame: (datasize)(width)(height)(DATA...
		if(imgtot>0){
			unlock();	
			maxRead=sizeof(fromServer);
			if(trigger==IS_internal)status=pasynOctetSyncIO->read(pasynUser, fromServer, maxRead,timeout, &nRead_, &eomReason);
			else{
				nRead_=0;
				while(nRead_==0){
					pasynOctetSyncIO->read(pasynUser, fromServer, maxRead,XPAD_POLL_DELAY, &nRead_, &eomReason);
				}
			}
			lock();	

			if(eomReason==2){// eomReason="\r"(=0x0a=10) this means the pasynOctetSyncIO interface stopped reading from the server because of a "\r" char so we put back this char because ..well we are not really dealing with chars. a uint32_t version of this function would be much welcome 
				fromServer[nRead_]=(int32_t)10;
				nRead_++;
			}
			if(fromServer[0]==0&&fromServer[1]==0&& fromServer[2]==0&& fromServer[3]==0) {
				asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: WARNING: empty image, if you didn't abort yourself check the system\n", driverName, functionName);
				//asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:\n eval=%lf\nvalues 1:%d, 2;%d, 3:%d, 4:%d, 5:%d, 6;%d, 7:%d, 8:%d, 9:%d, 10;%d, 11:%d, 12:%d,13:%d \n", driverName, functionName,eval,fromServer[0],fromServer[1],fromServer[2],fromServer[3],fromServer[4],fromServer[5],fromServer[6],fromServer[7],fromServer[8],fromServer[9],fromServer[10],fromServer[11],fromServer[12]);

				img_rest=1;
			}
		}
		pOut = (char *)	pImage->pData;
		pIn = fromServer+headerLen;
		
		nRead_-=headerLen;

		headerLen=12;//12;
		if(((int)nRead_)<0){
			 nRead_=0;
		 }
		memcpy(pOut, pIn, nRead_);
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s: Recieving image %d (%d remaining) \n",driverName, functionName,imgtot,img_rest-1);

		
		/** 
		 * Transfer of the recieved frames in the buffer
		 */

	    pOut+= nRead_;
	    for (nCopied=nRead_; (uint32_t)nCopied<dataLen; nCopied+=(int)nRead_) {
			
			maxRead = sizeof(fromServer);
				

			if (maxRead > (dataLen - nCopied)) maxRead = dataLen - nCopied;
			status = pasynOctetSyncIO->read(pasynUser, fromServer, maxRead,1, &nRead_, &eomReason);
			

			getIntegerParam(ADStatus, &abort);

			if (status != asynSuccess) {
				break;
			}
			if(eomReason==2){
				fromServer[nRead_]=(int32_t)10;
				nRead_++;
			}
			pIn = fromServer; 
	        memcpy(pOut, pIn, nRead_);

	        pOut+= nRead_;

	       
	    }
		/**Last parameters (could be compared to metadatas) are then set
		*/

	    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s: end of image %d readout\n",driverName, functionName,imgtot);
	    getIntegerParam(NDArrayCounter, &ival);
	    epicsTimeGetCurrent(&now);
	    pImage->uniqueId = ival;
	    pImage->timeStamp = now.secPastEpoch + now.nsec / 1.e9;
	    updateTimeStamp(&pImage->epicsTS);
	    this->getAttributes(pImage->pAttributeList);
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s: calling NDArray callback\n", driverName, functionName);
	    /** Once complete the buffer is transfered where it is needed */
	    this->unlock();
	    doCallbacksGenericPointer(pImage, NDArrayData, 0);
	    this->lock();
	 	    
	    //this is for the testers 
	    //FILE * fd=NULL;
		//fd=fopen("/home/nicolas/Documents/test", "w+");
		//char test[105000];
		//memcpy(test, pImage->pData,105000);
		//if(fd!=NULL){	
			////fwrite((void*)pImage->pData,dataLen,1,fd);
			//for(int i=0; i<15000;i++){
				
				//fprintf(fd,"%02x ",test[i]);
			//}
			//fclose(fd);
		//}
		//else asynPrint(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "CANNOT OPEN FILE");
	    img_rest--;
	    imgtot++;
		status=pasynOctetSyncIO->write(pasynUser, "R", 2, XPAD_SOCKET_TIMEOUT,&nWrite_);

	    nCopied=0;
	    setIntegerParam(ADNumImagesCounter,imgtot);
	    callParamCallbacks();
	    
	}

	/**Memory release*/
	pImage->release();
	
	//theoricaly useless
	//nRead_=1;
	//while(nRead_>0){
	//pasynOctetSyncIO->read(pasynUser, fromServer, sizeof(fromServer),.1, &nRead_, &eomReason);
	//status=pasynOctetSyncIO->write(pasynUser, "R", 2, XPAD_SOCKET_TIMEOUT,&nWrite_);
	//pasynOctetSyncIO->read(pasynUser, fromServer, sizeof(fromServer),1, &nRead_, &eomReason);
	//}
	setStringParam(ADStringFromServer,fromServer);
	callParamCallbacks();
    return asynSuccess;
}
///Generate a white image in the server from the current exposure parameters
asynStatus xpad::createWhiteImage(char * saymyname){
	double exptime=0;
	int ival;
	asynStatus status=asynError;
	ival=strlen(saymyname);
	if(strncmp(saymyname+(ival-4),".dat",4)!=0){
		saymyname[ival]='.';
		saymyname[ival+1]='d';
		saymyname[ival+2]='a';
		saymyname[ival+3]='t';
		saymyname[ival+4]='\0';
	}
		
	epicsSnprintf(this->toServer, sizeof(this->toServer),"CreateWhiteImage %s",saymyname);
	writeServer(this->toServer);
	getDoubleParam(ADAcquireTime,&exptime);
	status=waitForCompletion("* 0",this->fromServer,1.65*exptime+XPAD_COMMAND_TIMEOUT);
	return status;
}
///Save the current detector calibration in the file at given path
asynStatus xpad::saveConfigToFile( const char * fileName){
	const char* functionName="SaveConfigToFile";

	char fileName_cpy[MAX_FILENAME_LEN],buff[MAX_OCT_BUFF_SIZE];
	char *pOut;
	size_t nWrite_,nRead_,nCopied;
	FILE * saveFile;
	int eomReason,ival,maxRead;
	uint32_t ui32val;

	strcpy(fileName_cpy,fileName);
	ival=strlen(fileName_cpy);

	if(strncmp(".cfg",fileName_cpy+ival-4,4)!=0){
		fileName_cpy[ival+4]='\0';

		fileName_cpy[ival+3]='g';
		fileName_cpy[ival+2]='f';
		fileName_cpy[ival+1]='c';
		fileName_cpy[ival]='.';
	}
	setStringParam(xpad_filepath,&fileName_cpy[0]);

	saveFile=fopen(fileName_cpy, "w+");
	if(saveFile==NULL){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR File not found:%s \n \n", driverName, functionName,fileName);
		return asynError;
	}
	asynStatus status=asynSuccess;

		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG AMPTP");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,31);
		fprintf(saveFile,"%s",buff);
		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG IMFP");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,59);
		fprintf(saveFile,"%s",buff);
		
		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG IOTA");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,60);
		
		fprintf(saveFile,"%s",buff);
		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG IPRE");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,61);
		fprintf(saveFile,"%s",buff);
		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG ITHL");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);  
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,62);
		fprintf(saveFile,"%s",buff);
		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG ITUNE");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,63);
		fprintf(saveFile,"%s",buff);
		
		epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigG IBUFF");
		unlock();
		status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
		setStringParam(ADStringFromServer, fromServer);
		callParamCallbacks();
		lock();
		unpackServer(fromServer,buff,UNPACK_CALIB,64);
		fprintf(saveFile,"%s",buff);


	fclose(saveFile);
	saveFile=NULL;
	fileName_cpy[strlen(fileName_cpy)-1]='l';
	saveFile=fopen(fileName_cpy,"w+");
	epicsSnprintf(this->toServer, sizeof(this->toServer),"ReadConfigL");
	status=pasynOctetSyncIO->writeRead(this->pasynUserServer,this->toServer,sizeof(toServer),fromServer,sizeof(fromServer),XPAD_COMMAND_TIMEOUT,&nWrite_, &nRead_, &eomReason);
	ival=0;
	if(fromServer[0]=='>'){
		ival=2;
	}
	ui32val=(uint8_t)fromServer[ival+3]<<24|(uint8_t)fromServer[ival+2]<<16|(uint8_t)fromServer[ival+1]<<8|(uint8_t)fromServer[ival+0];	 
	if(eomReason==2){// eomReason="\r"(=0x0a=10) this means the pasynOctetSyncIO interface stopped reading from the server because of a "\r" char so we put back this char because ..well we are not really dealing with chars. a uint32_t version of this function would be much welcome 
		//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "\n eom \n");
		fromServer[nRead_]=10;
		nRead_++;
	}
	 pOut=(fromServer+8+ival);
	fprintf(saveFile,"%s",pOut);
	nCopied=nRead_-8-ival;
	maxRead=MAX_RETURN_SIZE;

	while(nCopied<ui32val){
		status=pasynOctetSyncIO->read(pasynUserServer, fromServer, maxRead,XPAD_POLL_DELAY, &nRead_, &eomReason);
		if(eomReason==2){// eomReason="\r"(=0x0a=10) this means the pasynOctetSyncIO interface stopped reading from the server because of a "\r" char so we put back this char because ..well we are not really dealing with chars. a uint32_t version of this function would be much welcome 
			//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "\n eom \n");
			fromServer[nRead_]=10;
			nRead_++;
		}
		
		if(status==asynSuccess){
			fprintf(saveFile,"%s",(uint8_t*)fromServer);
			nCopied+=nRead_;
		}
		nRead_=0;
	}
	
	writeServer("ack\n");
	waitForCompletion("* 0",fromServer,XPAD_COMMAND_TIMEOUT);
	fclose(saveFile);
	return asynSuccess;
}

///Alows to use file at given path as a Calibration file
asynStatus xpad::loadConfigFromFile( const char * fileName){
	const char* functionName="loadConfigFromFile";
	char  fileName_cpy[MAX_FILENAME_LEN];
	char* pOut=NULL;

	FILE * fc;
	size_t nWrite,nSet;
	uint32_t fileSize,filesentsize;
	uint8_t * buffer=NULL;
	int ival;
	bool g_or_l;
	asynPrint(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,  "%s:%s: ENTERED FUNC file:%s\n",  driverName, functionName,fileName);
	
	strcpy(fileName_cpy,fileName);
	ival=strlen(fileName);

	g_or_l=(fileName_cpy[ival-1]=='l');
	
	fc=fopen(fileName, "r");
	if(fc==NULL){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR File not found:%s \n \n", driverName, functionName,fileName);
		return asynError;
	}
	
	
	// obtain file size:
	fseek (fc , 0 , SEEK_END);
	fileSize =(uint32_t) ftell (fc);
	filesentsize=fileSize+4;
	rewind (fc);
	buffer = (uint8_t*) malloc (sizeof(uint8_t)*fileSize+4);
	
	//Preparation to send size with the rest (a raw one)
	pOut=(char *)buffer;
	buffer[3]=(uint8_t)((fileSize&0b11111111000000000000000000000000)>>24);
	buffer[2]=(uint8_t)((fileSize&0b00000000111111110000000000000000)>>16);
	buffer[1]=(uint8_t)((fileSize&0b00000000000000001111111100000000)>>8);
	buffer[0]=(uint8_t)(fileSize&0b00000000000000000000000011111111);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s%s:size=%d converted in octets: %2x %2x %2x %2x \n",driverName,functionName,fileSize ,(uint8_t) buffer[0],(uint8_t)  buffer[1], (uint8_t) buffer[2],(uint8_t)  buffer[3]);
	
	//Reading file (might need to be modified in case of decreased MAX MESSAGE SIZE
	pOut+=4;
	nSet=fread(pOut,1	,fileSize,fc);
	pOut-=4;
	fclose(fc);
	if(nSet!=fileSize)	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:WARNING: Only %d bytes read in file %s\n\n", driverName, functionName,(int)nSet,fileName_cpy);

	//Sending File
	epicsSnprintf(this->toServer, sizeof(this->toServer),"LoadConfig%cFromFile",(true-g_or_l)*'G'+g_or_l*'L') ;	
	writeServer(toServer);
	nSet=0;nWrite=0;
	while(nSet<filesentsize){
		pasynOctetSyncIO->write(this->pasynUserServer, pOut,filesentsize-nSet, XPAD_SOCKET_TIMEOUT,&nWrite);
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s: nWrite: %d nSet:%d\n", driverName, functionName,(int)nWrite,(int) nSet);
		nSet+=nWrite;
		pOut+=nWrite;
	}
	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:WARNING: set=%d\nfilesentsize=%d%s\n\n", driverName, functionName,(int)nSet,filesentsize,fileName_cpy);

	free(buffer);
	
	//Same with local config 
	if(!g_or_l){
		fileName_cpy[ival-1]='l';
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s: local filename is :%s\n", driverName, functionName,fileName_cpy);
		if(loadConfigFromFile(fileName_cpy)==asynError) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:WARNING:No local configuration has been made \n", driverName, functionName);
	}
	epicsEventWaitWithTimeout(abortEventId,10+XPAD_SUPP_DELAY/2);
	return asynSuccess;
}

asynStatus xpad::writeServer(const char *output)
{
    size_t nwrite;
    asynStatus status;
    asynUser *pasynUser = this->pasynUserServer;
    const char *functionName="writeServer";
    
    
    /* Flush any stale input, since the next operation is likely to be a read */
    pasynOctetSyncIO->flush(pasynUser);
    status = pasynOctetSyncIO->write(pasynUser, output,strlen(output), XPAD_SOCKET_TIMEOUT, &nwrite);                                  
    if (status) asynPrint(pasynUser, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,  "%s:%s: ERROR writing on server status=%d, sent:%s\n",  driverName, functionName, status, output);

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringToServer, output);
    callParamCallbacks();
    //GetWhiteImagesInDir pour avoir la liste des white

    return(status);
}
asynStatus xpad::unpackServer(int mode){
	return unpackServer(fromServer,fromServer,mode,0);
}

///Remove quotes and * and everything useless for treatment and display. 2 possible modes
/**
 * The first mode is UNPACK_CALIB which take out quotes, * and add the number of module by deduction and register number passed by a parameter. The result is stored in the output char *
 * The second mode is UNPACK_QUOTE which put the text string existing between the quotes in the output char * variable , if no quotes are in the input it will copy the input in the output 
 * 
 * 
 * */
asynStatus xpad::unpackServer(char* input,char * output,int mode,int param)
{
	asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,"%s ENTREE DE UNPACKER PARSER avec input=%s\n",driverName,input);
	
	int indstart,indstop;
	bool turn=true;
	
	if(mode==UNPACK_CALIB){
		char charval[50];
		int ival=1,offset=0,itemp=0;
		indstart=0;indstop=strlen(input);
		while(turn){
			for(int i=0; i<4096;i++){
				if(input[i]==':'){
					indstart=i+1;
						asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,"\n\nINDSTART = %d \n",indstart);

				}
				if(input[i]==';'){
					indstop=i;
					asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,"\n\nINDSTOP = %d \n",indstop);
					asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,"\n\nOFFSET = %d \n",offset);

					sprintf(charval,"%d ",ival);
					itemp=strlen(charval);
					strcpy(output+offset,charval);
					offset+=itemp;
					sprintf(charval,"%d", param);
					

					strcpy(output+offset,charval);
					itemp=strlen(charval);
					offset+=itemp;

					strncpy(output+offset,input+indstart,indstop-indstart);
					offset+=indstop-indstart;
					output[offset]='\n';
					offset+=1;
					ival=ival*2;
				}
				if(input[i]=='"' && indstart!=0){
					output[offset]='\0';
					
					return asynSuccess;
				}
				
			}
		}
		return asynError;
		asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,"%s: Sortie DE UNPACKER PARSER avec s=%s \n\n",driverName, output);
	}
		

		

	
	
	if(mode==UNPACK_QUOTE){
		indstart=0;indstop=strlen(input);
		for(int i=0; i<1024;i++){
			if(input[i]=='"'){
				if(indstart==0){
					indstart=i+1;
				}
				else{
					indstop=i;
					strncpy(output,input+indstart,indstop-indstart);
					output[indstop-indstart]='\0';

					return asynSuccess;
				}

			}
		}
	}
	return asynError;
}

///Read server and store the result in input variable (usualy "fromServer")
asynStatus xpad::readServer(char *input, size_t maxChars, double timeout)
{

    size_t nread=0;
    asynStatus status=asynSuccess;
    asynUser *pasynUser = this->pasynUserServer;
    int eomReason;
    const char *functionName="readServer";
	    
    unlock();
    while(nread<2){
		status = pasynOctetSyncIO->read(pasynUser, input, maxChars, timeout,&nread, &eomReason);
	}
    lock();
    if (nread == 0) return(status);
    if (status) asynPrint(pasynUser, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,"%s:%s: ERROR reading server failed\ntimeout=%f, status=%d received %lu bytes\n%s\n", driverName, functionName, timeout, status, (unsigned long)nread, input);
    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringFromServer, input);
    callParamCallbacks();
    return(status);
}

///* This function is called when the exposure time timer expires */
extern "C" {static void timerCallbackC(void *drvPvt)
{
    xpad *pPvt = (xpad *)drvPvt;
    
   epicsEventSignal(pPvt->stopEventId);
}}

///Read server until it sends a string identical to "donestring" or an error message ot it timeouts 
asynStatus xpad::waitForCompletion(const char *doneString,char * resultstr, double timeout){

    asynStatus status=asynSuccess;
    double elapsedTime;
    epicsTimeStamp start, now;
    int abort;
    const char *functionName = "waitForCompletion";
   	asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s  waiting for %s from server\n", driverName, functionName,doneString);
    epicsTimeGetCurrent(&start);
    while (1) {
		getIntegerParam(ADStatus, &abort);
		if(abort==ADStatusAborted || abort==xpadStatusAborting)return asynError;
        status = readServer(resultstr, sizeof(resultstr), XPAD_POLL_DELAY);
        if (status == asynSuccess) {

			            if (strncmp(resultstr,doneString,sizeof(doneString))==0){
							 return(asynSuccess);
						 }
			            else if(resultstr[0]=='!'){
							asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,"%s:%s:  XpadXXXServer sent error:  %s \n", driverName, functionName, resultstr);
							 return asynError;
						} else if(resultstr[0]=='#'){
							asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,"%s:%s:  XpadXXXServer sent Warning:  %s \n", driverName, functionName, resultstr);
							 return asynError;
						}
						//else if(resultstr[0]=='*' && strcmp(resultstr,"* -1")!=false){
							 
										//return(asynSuccess);
									
						//}
               
        }
        epicsTimeGetCurrent(&now);
        elapsedTime = epicsTimeDiffInSeconds(&now, &start);
        if (elapsedTime > timeout) { asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,"%s:%s: error waiting for response from XpadXXXServer\n",driverName, functionName);
            return(asynError);
        }
    }
}

///At the end of an operation reset detector driver status
//dont mind that it is basicaly useless
asynStatus xpad::changeMode(){
    asynStatus status=asynSuccess;
    setIntegerParam(ADStatus, xpadStatusChangeMode);
    callParamCallbacks();
    setIntegerParam(ADStatus, xpadStatusIdle);
    setIntegerParam(xpadChangeMode, 0);
    callParamCallbacks();
    return(status);
}

///Initialize the detector and asks if it is ready
asynStatus xpad::xpadInit()
{
	setIntegerParam(ADStatus, ADStatusInitializing);
	const char *functionName = "xpadInit";
	asynStatus status;
	if(ready==false){
		epicsSnprintf(this->toServer, sizeof(this->toServer), "Init");
	    writeServer(this->toServer);	
	    asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s:XPad is initializing\n",driverName,functionName);
	    status= waitForCompletion("* 0",fromServer,XPAD_COMMAND_TIMEOUT);
	    if(status) { 
		   asynPrint(pasynUserServer, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: XpadXXXServer failed Init, Try again \n",driverName,functionName);
		   ready=false;
		   return status;	    
			}
	    else{
			writeServer("GetDetectorModel");
			status=readServer(fromServer,MAX_MESSAGE_SIZE,5);
			if(status==asynSuccess){
				unpackServer(UNPACK_QUOTE);
				setStringParam(ADModel,fromServer);
			}
				
			ready=true;
			return asynSuccess	;
	}
	}else {
		epicsSnprintf(this->toServer, sizeof(this->toServer), "AskReady");
	    writeServer(this->toServer);	
	    asynPrint(pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s:XPAD is already initialized, Ask Ready \n. ",driverName,functionName);
	    status= waitForCompletion("* 0",fromServer,XPAD_COMMAND_TIMEOUT);
	    if(status==asynError) { 
		   asynPrint(pasynUserServer, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: XpadXXXServer failed Init, Try again \n ",driverName,functionName);
		   ready=false;
		  	    
		}
		readServer(fromServer,MAX_MESSAGE_SIZE,0.5);
		return status;
	}
}

///Control and possible resetting of every exposure parameter 
/**This control parameters set by the user and send it to the XPad
 * 
 * 
 * 
 * */
asynStatus xpad::setExposureParameters() {
	//Many local variables, it is not good for memory but better for network load
	// You can edit this code to send exposure parameters one by one
	//TODO : Possibly update only modified parameters, could be cool
	
	//Exposure parameters
	const char* functionName="setExposureParameters";
	int  acqumode,  geocor,  flatfcor,  imgtrans,  input,  output, numimg,  overflow, outformat, stacksize;
	double  exptime, waittime;
	char servfilepath[MAX_FILENAME_LEN];
	asynStatus status=asynSuccess;
	long double buffer,buffer2;//These are for not losing any precision when converting seconds to ms; Especialy usefull for very big or very small exposure times and periods
	/**if it is the first exposure or for a reason ther was a bug on the last one 
	 * 		We initialize the XPAD detector */
	status=xpadInit();
	if(status)return status;
	
//	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "=============Exposure parameters setting============================\n.\n.");

	
	/**The core of this function mainly control the values of every exposure parameters and reset them in case of bad values
	 *Parameters such as: */
	 
	 

	 /** Acquisiton mode*/
	status=getIntegerParam(xpad_acq_mode,& acqumode);
	if(status!=asynSuccess ||  acqumode<0 ||  acqumode>6){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR Could not set ascquisition mode, Reset to DEFAULT: Standard\n ", driverName, functionName);
		 acqumode=AM_standard;
		setIntegerParam(xpad_acq_mode, acqumode);
	}
	
	/**Geometrical correction flag*/
	status=getIntegerParam(xpad_geo_correction,& geocor);
	if(status!=asynSuccess ||  geocor<0 ||  geocor>1){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR Could not set geometrical correction flag, Reset to DEFAULT: false (no correction) \n", driverName, functionName);
		 geocor=0;
		setIntegerParam(xpad_geo_correction, geocor);
	}

	/**Flat field correction flag*/
	status=getIntegerParam(xpad_flat_field,& flatfcor);
	if(status!=asynSuccess ||  flatfcor<0 ||  flatfcor>1){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR Could not set flat field enable flag, Reset to DEFAULT: false (no correction)\n ", driverName, functionName);
		 flatfcor=1;
		setIntegerParam(xpad_flat_field, flatfcor);
	}
	
	/**Image transfer flag */
	status=getIntegerParam(xpad_img_transfer,& imgtrans);
	if(status!=asynSuccess ||  imgtrans<0 ||  imgtrans>1){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR Could not set image transfer flag, Reset to DEFAULT: True (streaming bitmap on network right after the end of acquisition) \n", driverName, functionName);
		 imgtrans=1;
		setIntegerParam(xpad_img_transfer, imgtrans);
	}
	
	/**Input signals  (trigger modes)*/
	status=getIntegerParam(ADTriggerMode,& input);
	if(status!=asynSuccess ||  input<0 ||  input>3){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR Could not set input signal, Reset to DEFAULT: Internal \n", driverName, functionName);
		 input=IS_internal;
		setIntegerParam(ADTriggerMode, input);
	}

	/**Output signal */
	status=getIntegerParam(xpad_output,&output);
	if(status!=asynSuccess ||  output<0 ||  output>9){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s:ERROR: Could not set output signal, Reset to DEFAULT: Exposure busy\n", driverName, functionName);
		 output=OS_shutter_busy;
		setIntegerParam(xpad_output, output);
	}
	
	/** Number of Images */
	status=getIntegerParam(ADNumImages,& numimg);
	if(status!=asynSuccess ||  numimg<1){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: ERROR: number of images must be greater that 1, Reset to DEFAULT: 1 \n", driverName, functionName);
		 numimg=1;
		setIntegerParam(ADNumImages, numimg);
	}else if(numimg>65535){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: Warning: Number of images is greater than 16Bit maximum integer value, the detector will not take the %d images \n", driverName, functionName,numimg);
		numimg=15000;
	}
	if(numimg>1)    setIntegerParam(ADImageMode, ADImageMultiple);


	/** Exposure Time */
	status=getDoubleParam(ADAcquireTime,& exptime);
	if(status!=asynSuccess ||  exptime<0.000001){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: ERROR: exposure time must be a positive value  min= 1µs, Reset to DEFAULT: 1 second\n", driverName, functionName);
		 exptime=1.;
		setDoubleParam(ADAcquireTime, exptime);
	}
	if(exptime>4294.967295){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: ERROR: Out of 32bits uint range, reset close to max value\n", driverName, functionName);
		 exptime=4294.967280;
		setDoubleParam(ADAcquireTime, exptime);
	}
	buffer2=1000000.0*exptime;
	
	/** Time between images*/
	status=getDoubleParam(ADAcquirePeriod,& waittime);
	if(status!=asynSuccess ||  waittime<=0.000001 ||exptime>waittime){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: ERROR: time between images must be a positive value min=1µs\n adaquireperiod must be greater than AcquireTime , Reset to DEFAULT: time between images 15 ms\n", driverName, functionName);
		 waittime=0.015+ exptime;
		setDoubleParam(ADAcquirePeriod, waittime);
	}
	buffer=1000000*waittime;
	buffer-=buffer2;
	
	if(buffer>4294967295){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: ERROR: time between images is too big, reset to 32 bit unsigned max value\n", driverName, functionName);
		buffer=4294967295;
		setDoubleParam(ADAcquirePeriod,4294,967295);

	}
		
	/**Overflow Time*/
	status=getIntegerParam(xpad_overflow,& overflow);
	if(status!=asynSuccess ||  overflow<4000){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: Overflow time must be superior to 4000 µs \n", driverName, functionName);
		 overflow=4000;
		setIntegerParam(xpad_overflow, overflow);
	}
	
	/**Output format*/
	status=getIntegerParam(xpad_outformat,& outformat);
	if(status!=asynSuccess ||  outformat<0 || outformat>1){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: Only 2 output format possibe: 0:Binary  1: ascii , reset to Binary (default) \n", driverName, functionName);
		 outformat=0;
		setIntegerParam(xpad_outformat, outformat);
	}

	/**Image number/stack*/
	status=getIntegerParam(xpad_stacksize,& stacksize);
	if(status!=asynSuccess ||  stacksize<0 ){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: Warning: server stacksize reset to 1(default) \n", driverName, functionName);
		 stacksize=1;
		setIntegerParam(xpad_stacksize, stacksize);
	}

	/**Output server filepath */
	status=getStringParam(xpad_outpath,MAX_MESSAGE_SIZE, servfilepath);
	if(status!=asynSuccess ){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: Warning:Output server filepath reset to /opt/imXPAD/tmp_corrected/ (default) \n", driverName, functionName);
		epicsSnprintf(servfilepath, sizeof(servfilepath),"/opt/imXPAD/tmp_corrected/"); 
		setStringParam(xpad_outpath,servfilepath);
	}
	 

	
	/**Sendin config to server */
	epicsSnprintf(this->toServer, sizeof(this->toServer),"setExposureParameters %i %u %u %i %i %i %i %i %d %d %i %i %s", numimg,  (uint32_t)buffer2,(uint32_t)buffer  , overflow, input,  output, geocor, flatfcor,imgtrans,outformat,  acqumode, stacksize, servfilepath); 	
	writeServer(this->toServer);
	status=waitForCompletion("* 0",fromServer,XPAD_COMMAND_TIMEOUT);	
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s%s: End of setting the exposure parameters \n FRAME SENT: setExposureParameters %i %i %i %i %i %i %i %i %d %d %i %i %s\n Setting returned %s\n",driverName,functionName, numimg, (uint32_t)buffer2, (uint32_t)buffer, overflow, input,  output, geocor, flatfcor,imgtrans,outformat,  acqumode, stacksize, servfilepath,fromServer); 

	/**We need the image size to prepare the buffer recieving the image
	 * This can change depending on geometrical corection status 
	 * this cannot work on XpadServer under version 3.0 so for now its on commented in the source till servers are updated*/
	 
	 
	//Size of image parameters (and related parsing):
	epicsSnprintf(this->toServer, sizeof(this->toServer),"GetImageSize"); 
	writeServer(this->toServer);
	readServer(fromServer,64,25);
	int indstart, indmult,indstop;
	indstart=0;
	indstop=strlen(fromServer);
	indmult=indstop/2;
	int sizex, sizey;

	for(int i=0; i<32;i++){
		if(fromServer[i]=='"'){
			if(indstart==0){
				indstart=i+1;
			}
			else{
				indstop=i+1;
			}
		}
		if(fromServer[i]=='x'){
			indmult=i+1;
		}
	}
	sizey=atoi(fromServer+indstart);
	sizex=atoi(fromServer+indmult);
	setIntegerParam(NDArraySizeX,sizex);
	setIntegerParam(NDArraySizeY,sizey);
	setIntegerParam(NDArraySize,sizex*sizey*sizeof(epicsInt32));	
	callParamCallbacks();
	
	return status;
}


///Task launcher 
static void xpadTaskC(void *drvPvt){
    xpad *pPvt = (xpad *)drvPvt;
    pPvt->xpadTask();
}
static void xpadAbortTaskC(void *drvPvt){
    xpad *pPvt = (xpad *)drvPvt;
    pPvt->xpadAbortTask();
}

///Main xpad Thread 
/** This thread controls handling of slow events - erase, acquire, change mode */
void xpad::xpadTask(){
	char strval[MAX_FILENAME_LEN];
    int status = asynSuccess;
    int ival;
    double dval;
    int imageMode;
    const char *functionName = "xpadTask";            
    this->lock();
	    /* Main Loop */
    while (1) {
        setStringParam(ADStatusMessage, "Waiting for event");
        callParamCallbacks();
        /* Release the lock while we wait for an event that says acquire has started, then lock again */
        this->unlock();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "%s:%s: Waiting for start event\n", driverName, functionName);
        status = epicsEventWait(this->startEventId);
        this->lock();

        switch(this->mode) {		    	


            case xmode_aquire:
				this->mode=xmode_config;
				setIntegerParam(ADStatus, xpadStatusChangeMode);
				callParamCallbacks();
				setIntegerParam(ADNumImagesCounter,0);
				status=setExposureParameters();

				if(status==asynSuccess){
					getIntegerParam(xpad_img_transfer,&imageMode);
					if(imageMode==1){
						
						this->mode=xmode_aquire;
						callParamCallbacks();
						status=getImageStream();
						if (status){
							this->mode = xmode_idle;
							setIntegerParam(ADAcquire, 0);
							setIntegerParam(ADStatus, xpadStatusIdle);
							break;
						}
					}
				
					else{
						getDoubleParam(ADAcquirePeriod,&dval);
						getIntegerParam(ADNumImages,&ival);
						writeServer("StartExposure");

						callParamCallbacks();
						waitForCompletion("* 0",fromServer,(1.65*dval)*ival+XPAD_SUPP_DELAY);
					}
				}else{
					this->mode = xmode_idle;
					setIntegerParam(ADAcquire, 0);
					setIntegerParam(ADStatus, xpadStatusIdle);
					 break;
				 }
                if (status) break;
                    /* We get out of the loop in single shot mode or if acquire was set to 0 by client */

                this->mode = xmode_idle;
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, xpadStatusIdle);
                break;

            case xmode_config:
				this->setExposureParameters();
				this->changeMode();
                break;
            case xmode_calib:
				status=xpadInit();
				if(status)break;
				getStringParam(xpad_filepath,MAX_FILENAME_LEN,strval);
				this->loadConfigFromFile(strval);
				setIntegerParam(ADStatus,xpadStatusIdle);
				this->mode = xmode_idle;
				callParamCallbacks();
				break;
				
            case xmode_scalib:
				status=xpadInit();
				if(status)break;
				getStringParam(xpad_filepath,MAX_FILENAME_LEN,strval);
				this->saveConfigToFile(strval);
				setIntegerParam(ADStatus,xpadStatusIdle);
				this->mode = xmode_idle;
				callParamCallbacks();
				break;
			case xmode_white:
				status=setExposureParameters();
				getStringParam(xpad_whitepath,MAX_FILENAME_LEN,strval);
				if(!status)createWhiteImage(strval);
				else		asynPrint(this->pasynUserServer, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,  "%s%s:Preparation for create white image failed\n ",driverName,functionName);

				setIntegerParam(ADStatus,xpadStatusIdle);
				this->mode = xmode_idle;
				callParamCallbacks();
				break;
			case xmode_reset:
				writeServer("ResetDetector");
				waitForCompletion("* 0",fromServer,XPAD_COMMAND_TIMEOUT);
				setIntegerParam(ADStatus,xpadStatusIdle);
				this->mode = xmode_idle;
				callParamCallbacks();
				break;
			case xmode_prompts:
				getStringParam(ADStringToServer,MAX_MESSAGE_SIZE, strval);
				writeServer(strval);
				this->mode = xmode_idle;
				break;
			case xmode_promptr:
				readServer(fromServer,MAX_MESSAGE_SIZE,XPAD_SUPP_DELAY);
				this->mode = xmode_idle;
				break;
            case xmode_changin:
                this->changeMode();
                this->mode = xmode_idle;
                break;
            default:
                break;
        }

        /* Call the callbacks to update any changes */
        callParamCallbacks();
    }
}

///In case the main task don't work as planned or the server is blocked this task is independant and communicate on another Client connexion than the main tasks
void xpad::xpadAbortTask(){
	size_t nWrite_,nRead_;
	int eomReason;

	while(1){
//unlock();
        epicsEventWait(this->abortEventId);

        //mainTask.epicsThreadSuspendSelf();
        pasynOctetSyncIO->connect("XpadAbort", 1, &this->pasynAbortServer, NULL);

		asynPrint(this->pasynUserServer, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER|ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER|ASYN_TRACEIO_DEVICE|ASYN_TRACEIO_FILTER|ASYN_TRACEIO_DRIVER ,  "\n\n%s: ABORTER THREAD HAS AWOKEN \n\n\n ",driverName);
		epicsSnprintf(this->toServer, sizeof(this->toServer),"AbortCurrentProcess");

		pasynOctetSyncIO->write(pasynAbortServer, toServer, sizeof(toServer),XPAD_COMMAND_TIMEOUT,&nWrite_);

		nRead_=2;
		if(this->mode!=xmode_aquire && mode!=xmode_white && mode!=xmode_scalib){
			while(nRead_>3){
				pasynOctetSyncIO->read(pasynUserServer,fromServer, sizeof(fromServer),XPAD_SUPP_DELAY, &nRead_, &eomReason);
				setStringParam(ADStringFromServer, fromServer);
				callParamCallbacks();
			}
		}setIntegerParam(ADAcquire,0);
		pasynOctetSyncIO->flush(this->pasynUserServer);
		pasynOctetSyncIO->disconnect(pasynAbortServer);
		setIntegerParam(ADStatus,ADStatusAborted);
		callParamCallbacks();
//lock();
	}


}
///Does not only write, called when a parameter is written, this performs associated actions
/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, xpadErase, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus xpad::writeInt32(asynUser *pasynUser, epicsInt32 value)
{

    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32"; 
    asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,  "%s%s: Entering Write32\n ",driverName,functionName);
    status = setIntegerParam(function, value);
    char strval[MAX_FILENAME_LEN];

    if (function == ADAcquire) {
		 asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, "reason ADAcquire \n");
        if (value && (this->mode == xmode_idle)) {
			asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, " case value && (this->mode == xmode_idle \n");
            /* Send an event to wake up the xpad task.  */
            this->mode=xmode_aquire;
            setIntegerParam(ADNumImagesCounter,0);
            callParamCallbacks();
            epicsEventSignal(this->startEventId);
        } 
        if (!value && (this->mode != xmode_idle)) {
			asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,  " case !value && (this->mode != xmode_idle \n");
            /* Stop acquiring (ends exposure, does not abort) */
         //    epicsThreadCreate("xpadAbortTask",epicsThreadPriorityHigh,epicsThreadGetStackSize(epicsThreadStackMedium),(EPICSTHREADFUNC)xpadAbortTaskC,this) == NULL;
            epicsEventSignal(this->stopEventId);
        

        }
    }  
    else if (function == xpadChangeMode) {
		 asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,  "reason XpadChangeMode  \n");
        if (value && (this->mode == xmode_idle)) {
           this->mode = xmode_changin;
            /* Send an event to wake up the xpad task.  */
            epicsEventSignal(this->startEventId);
        } 
    } else if (function == xpadAbort) {
		 asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,  "reason ABORT \n");
       
            /* Abort operation */
            setIntegerParam(ADStatus, xpadStatusAborting);
			callParamCallbacks();
            epicsEventSignal(this->abortEventId);        
    }else if (function == xpad_reset) {
		 asynPrint(this->pasynUserServer, ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER,  "reason ABORT \n");
			this->mode=xmode_reset;
            setIntegerParam(ADStatus, xpadStatusWaiting);
			callParamCallbacks();
            epicsEventSignal(this->startEventId);
        
    }else if (function == xpad_load_calib) {
		this->mode=xmode_calib;
		setIntegerParam(ADStatus, ADStatusInitializing);
		callParamCallbacks();
		epicsEventSignal(startEventId);
	}else if (function == xpad_save_calib) {
		this->mode=xmode_scalib;
		setIntegerParam(ADStatus, ADStatusReadout);
		callParamCallbacks();
		epicsEventSignal(startEventId);
	}else if (function == xpad_white_image) {
		this->mode=xmode_white;
		epicsEventSignal(startEventId);
	}else if (function == xpad_chose_white) {
		xpadInit();
		getStringParam(xpad_whitepath,MAX_FILENAME_LEN,strval);
		sprintf(toServer,"SetWhiteImage %s",strval);
		writeServer(toServer);
		setIntegerParam(ADStatus,xpadStatusIdle);
		this->mode = xmode_idle;
		callParamCallbacks();
	}else if (function == xpad_show_white) {
		xpadInit();	
		writeServer("GetWhiteImagesInDir");
		readServer(fromServer,MAX_MESSAGE_SIZE,XPAD_COMMAND_TIMEOUT);
		//unpackServer(UNPACK_QUOTE);
		//setStringParam(ADStringFromServer,fromServer);
		callParamCallbacks();
		setIntegerParam(ADStatus,xpadStatusIdle);
		this->mode = xmode_idle;
		callParamCallbacks();
	}else if (function == xpad_white_image) {
		this->mode=xmode_white;
		epicsEventSignal(startEventId);
	}else if (function == xpad_send) {
		this->mode=xmode_prompts;
		epicsEventSignal(startEventId);
	}else if (function == xpad_read) {
		this->mode=xmode_promptr;
		epicsEventSignal(startEventId);
	}
 
    else {
		 //asynPrint(this->pasynUserServer, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "reason other: %s/\n",function.functionName );
        /* If this is not a parameter we have handled call the base class */
        if (function < FIRST_XPAD_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }
        
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status)  asynPrint(pasynUser, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "%s:%s: ERROR: status=%d function=%d, value=%d\n", driverName, functionName, status, function, value);

    return status;
}




///Report status of the driver.
/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void xpad::report(FILE *fp, int details)
{
    fprintf(fp, "XPAD detector %s\n", this->portName);
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

///Create an instence of the driver 
/**
 * Creates an instence of the driver, 
 * 
 * IMPORTANT NOTE: This method is visible from the epics iocsh
 * 
 * */
extern "C" int xpadConfig(const char *portName, const char *serverPort,  int maxBuffers, size_t maxMemory,int priority, int stackSize)
{
	new xpad(portName, serverPort, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

///Create and initialize the driver and it's parameters 
/** Constructor for xpad driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values for the parameters defined in this class and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] serverPort The name of the asyn port driver previously created with drvAsynIPPortConfigure
  *            connected to the xpaddtb program.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 



  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread.
  * \param[in] stackSize The stack size for the asyn port driver thread.
  */
xpad::xpad(const char *portName, const char *serverPort,int maxBuffers, size_t maxMemory,int priority, int stackSize)
    : ADDriver(portName, 3, NUM_XPAD_PARAMS, maxBuffers, maxMemory,
               0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=1, autoConnect=1 */
               priority, stackSize){
	
	ready=false;
    int status = asynSuccess;
    epicsTimerQueueId timerQ;
	strcpy(this->serverPort,serverPort);
    const char *functionName = "xpad";
	//strcpy(this->serverPort,serverPort);
	if(strcasecmp(serverPort,"XpadAbort")!=0){
	    createParam(xpadChangeModeString,asynParamInt32, &xpadChangeMode);
	    createParam(xpadAbortString,     asynParamInt32, &xpadAbort);		
		createParam(xpad_stacksizeString, asynParamInt32, &xpad_stacksize);
		createParam(xpad_outformatString, asynParamInt32, &xpad_outformat);
		createParam(xpad_load_calibString, asynParamInt32, &xpad_load_calib);
		createParam(xpad_save_calibString, asynParamInt32, &xpad_save_calib);
		createParam(xpad_outputString, asynParamInt32, &xpad_output);
		createParam(xpad_geo_correctionString, asynParamInt32, &xpad_geo_correction);
		createParam(xpad_flat_fieldString, asynParamInt32,&xpad_flat_field);
		createParam(xpad_img_transferString, asynParamInt32,&xpad_img_transfer);
		createParam(xpad_acq_modeString, asynParamInt32, &xpad_acq_mode);
		createParam(xpad_overflowString, asynParamInt32, &xpad_overflow);
		createParam(xpad_outpathString, asynParamOctet, &xpad_outpath);
		createParam(xpad_filepathString, asynParamOctet, &xpad_filepath);
		createParam(xpad_whitepathString, asynParamOctet, &xpad_whitepath);
		createParam(xpad_white_imageString, asynParamInt32, &xpad_white_image);
		createParam(xpad_chose_whiteString, asynParamInt32, &xpad_chose_white);
		createParam(xpad_show_whiteString, asynParamInt32, &xpad_show_white);
		createParam(xpad_resetString, asynParamInt32, &xpad_reset);
		createParam(xpad_readString, asynParamInt32, &xpad_read);
		createParam(xpad_sendString, asynParamInt32, &xpad_send);
	
	    this->mode = xmode_idle;
	    
	    /* Create the epicsEvents for signaling to the xpad task when acquisition starts and stops */
	    this->startEventId = epicsEventCreate(epicsEventEmpty);
	    if (!this->startEventId) {
	        printf("%s:%s epicsEventCreate failure for start event\n", driverName, functionName);
	        return;
	    }
	    this->stopEventId = epicsEventCreate(epicsEventEmpty);
	    if (!this->stopEventId) {
	        printf("%s:%s epicsEventCreate failure for stop event\n", driverName, functionName);
	        return;
	    }
	    this->abortEventId = epicsEventCreate(epicsEventEmpty);
	    if (!this->abortEventId) {
	        printf("%s:%s epicsEventCreate failure for abort event\n", driverName, functionName);
	        return;
	    }
	
	
	    /* Create the epicsTimerQueue for exposure time handling */
	    timerQ = epicsTimerQueueAllocate(1, epicsThreadPriorityScanHigh);
	    this->timerId = epicsTimerQueueCreateTimer(timerQ, timerCallbackC, this);
	    
	    /* Connect to server */
	    status = pasynOctetSyncIO->connect(serverPort, 0, &this->pasynUserServer, NULL);
	    setIntegerParam(ADMaxSizeX, 4200);
		setIntegerParam(ADMaxSizeY, 4200);
	    /* Allocate the raw buffer we use to files.  Only do this once */    
		status|=setIntegerParam(NDColorMode, NDColorModeMono);
	    /* Set some default values for parameters */
	    status |=  setStringParam (ADManufacturer, "ImXPAD");
	    status |=  setIntegerParam (xpad_load_calib, 0);
	    status |=  setIntegerParam (xpad_save_calib, 0);
	    status |= setStringParam (ADModel, "XPAD");
	    status |= setIntegerParam(NDDataType,  NDInt32);
	    status |= setIntegerParam(ADImageMode, ADImageSingle);
	    status |= setIntegerParam(ADTriggerMode, IS_internal);
	    status |= setDoubleParam (ADAcquireTime, 1.);
	    status |= setDoubleParam (ADAcquirePeriod, 1.015);
	    status |= setIntegerParam(ADNumImages, 1);
		status |= setIntegerParam (xpad_acq_mode,AM_standard) ;
	   status |= setIntegerParam(xpad_geo_correction,0);//no geometrical correction raw hardware bitmap
	    status |= setIntegerParam( xpad_flat_field, 0);// flat fiel correction enabled
	    status |= setIntegerParam(xpad_img_transfer,1);//streaming images over the network, not usefull if the client is on the same machine as the server
	    status |= setIntegerParam(ADTriggerMode,IS_internal);
	    status |= setIntegerParam(xpad_overflow,4000);//Minimum overflow time
	    status |= setIntegerParam(xpad_output,OS_exposure_busy);
	    status |= setIntegerParam(xpad_outformat,1);//Binary
	    status |= setStringParam(xpad_outpath,"/opt/imXPAD/tmp_corrected/");
	    status |= setStringParam(xpad_filepath,"/EpicsOirledCalib.cfg");// 
	    status |= setStringParam(xpad_whitepath,"EpicsOirled.dat");
		callParamCallbacks();
	    /* Create the thread that collects the data */
	    mainTask=epicsThreadCreate("xpadTask",epicsThreadPriorityMedium,epicsThreadGetStackSize(epicsThreadStackMedium),(EPICSTHREADFUNC)xpadTaskC,this);
		status = (mainTask == NULL);
	    if (status) {
	        printf("%s:%s epicsThreadCreate failure for data collection task\n",  driverName, functionName);
	        return;
	    }	

		abortTask=epicsThreadCreate("xpadAbortTask",epicsThreadPriorityHigh,epicsThreadGetStackSize(epicsThreadStackMedium),(EPICSTHREADFUNC)xpadAbortTaskC,this) ;
	
	}
}






/* Code for iocsh registration */
static const iocshArg xpadConfigArg0 = {"Port name", iocshArgString};
static const iocshArg xpadConfigArg1 = {"server port name", iocshArgString};
static const iocshArg xpadConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg xpadConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg xpadConfigArg4 = {"priority", iocshArgInt};
static const iocshArg xpadConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const xpadConfigArgs[] =  {&xpadConfigArg0,
                                                     &xpadConfigArg1,
                                                     &xpadConfigArg2,
                                                     &xpadConfigArg3,
                                                     &xpadConfigArg4,
                                                     &xpadConfigArg5};
static const iocshFuncDef configXPAD = {"xpadConfig", 6, xpadConfigArgs};
static void configXPADCallFunc(const iocshArgBuf *args)
{
    xpadConfig(args[0].sval, args[1].sval, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival);
}


static void xpadRegister(void)
{

    iocshRegister(&configXPAD, configXPADCallFunc);
}

extern "C" {
epicsExportRegistrar(xpadRegister);
}
