< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/xpadApp.dbd")
xpadApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "13XPAD_1:")
# The port name for the detector
epicsEnvSet("PORT" ,  "XPAD")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "1120")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1200")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

###
# Create the asyn port to talk to the XpadXXXServer on default port 3456

drvAsynIPPortConfigure("XpadXXXServer","127.0.0.1:3456")
# Set the input and output terminators.
asynOctetSetInputEos("XpadXXXServer", 0, "\n")
asynOctetSetOutputEos("XpadXXXServer", 0, "\n")
asynSetTraceIOMask("XpadXXXServer",0,2)
#asynSetTraceMask("XpadXXXServer",0,255)



drvAsynIPPortConfigure("XpadAbort","127.0.0.1:3456")
asynOctetSetInputEos("XpadAbort", 1, "\n")
asynOctetSetOutputEos("XpadAbort", 1, "\n")
asynSetTraceIOMask("XpadAbort",1,2)
#asynSetTraceMask("XpadAbort",1,255)

#asynSetTraceIOMask("$(PORT)",0,3)
#asynSetTraceMask("$(PORT)",0,255)

#asynSetTraceIOMask("XPADABORTER",1,2)
#asynSetTraceMask("XPADABORTER",1,255)

xpadConfig("$(PORT)", "XpadXXXServer", 0, 0)


dbLoadRecords("$(XPAD)/db/xpad.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,XPADXXXSERVER_PORT=XpadXXXServer")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1",3, 0, "$(PORT)", 0)


dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=2000000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(XPAD)/xpadApp/Db")

#asynSetTraceMask("$(PORT)",0,3)
#asynSetTraceIOMask("$(PORT)",0,4)



# save things every thirty seconds
iocInit()

create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")
