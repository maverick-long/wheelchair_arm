#ifdef KINOVAAPIUSBCOMMANDLAYER_EXPORTS
#define KINOVAAPIUSBCOMMANDLAYER_API __attribute__ ((visibility ("default")))
#else
#define KINOVAAPIUSBCOMMANDLAYER_API __attribute__ ((visibility ("default")))
#endif

/**
 * @file Kinova.API.UsbCommandLayerUbuntu.h
 * @brief This file contains header of all available functions of this API.
 */

#include <vector>
#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include <stdio.h>

//This defines the the location of the communication layer.(Kinova.DLL.CommLayer.dll)
#define COMM_LAYER_PATH "Kinova.API.CommLayerUbuntu.so"

//This indicates the success of the current operation
#define SUCCESS 1
#define KINOVA_ 1

// ***** E R R O R   C O D E S ******
#define ERROR_INIT_API 2001      // Error while initializing the API
#define ERROR_LOAD_COMM_DLL 2002 // Error while loading the communication layer

//Those 3 codes are mostly for internal use
#define JACO_NACK_FIRST 2003
#define JACO_COMM_FAILED 2004
#define JACO_NACK_NORMAL 2005

//Unable to initialize the communication layer.
#define ERROR_INIT_COMM_METHOD  2006

//Unable to load the Close() function from the communication layer.
#define ERROR_CLOSE_METHOD  2007

//Unable to load the GetDeviceCount() function from the communication layer.
#define ERROR_GET_DEVICE_COUNT_METHOD  2008

//Unable to load the SendPacket() function from the communication layer.
#define ERROR_SEND_PACKET_METHOD  2009

//Unable to load the SetActiveDevice() function from the communication layer.
#define ERROR_SET_ACTIVE_DEVICE_METHOD 2010

//Unable to load the GetDeviceList() function from the communication layer.
#define ERROR_GET_DEVICES_LIST_METHOD 2011

//Unable to initialized the system semaphore.
#define ERROR_SEMAPHORE_FAILED 2012

//A function's parameter is not valid.
#define ERROR_INVALID_PARAM 2100

//The API is not initialized.
#define ERROR_API_NOT_INITIALIZED 2101

// ***** E N D  O F  E R R O R   C O D E S ******

//Version of the API 5.01.01
#define COMMAND_LAYER_VERSION 50101

// ***** API'S FUNCTIONAL CORE *****

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetDevices(std::vector<KinovaDevice> &devices, int &result);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetActiveDevice(KinovaDevice device);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int InitAPI(void);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int CloseAPI(void);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetCodeVersion(std::vector<int> &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAPIVersion(std::vector<int> &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianPosition(CartesianPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularPosition(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianForce(CartesianPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularForce(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCurrent(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetActualTrajectoryInfo(TrajectoryPoint &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetGlobalTrajectoryInfo(TrajectoryFIFO &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetSensorsInfo(SensorsInfo &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetSingularityVector(SingularityVector &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetAngularControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int StartControlAPI();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int StopControlAPI();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int RestoreFactoryDefault();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SendJoystickCommand(JoystickCommand joystickCommand);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SendAdvanceTrajectory(TrajectoryPoint trajectory);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SendBasicTrajectory(TrajectoryPoint trajectory);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetClientConfigurations(ClientConfigurations &config);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetClientConfigurations(ClientConfigurations config);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int EraseAllTrajectories();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetPositionCurrentActuators(std::vector<float> &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorPID(unsigned int address, float P, float I, float D);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCommand(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianCommand(CartesianPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCurrentMotor(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularVelocity(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetControlType(int &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int StartForceControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int StopForceControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int StartCurrentLimitation();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int StopCurrentLimitation();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetSystemErrorCount(unsigned int &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetSystemError(unsigned int indexError, SystemError &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int ClearErrorLog();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int EraseAllProtectionZones();

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetSerialNumber(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetControlMapping(ControlMappingCharts &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetProtectionZone(ZoneList &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetProtectionZone(ZoneList Command);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetGripperStatus(Gripper &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetQuickStatus(QuickStatus &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetForcesInfo(ForcesInfo &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetControlMapping(ControlMappingCharts Command);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int ProgramFlash(char * filename);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetJointZero(int ActuatorAdress);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueZero(int ActuatorAdress);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueGain(int ActuatorAdress, int Gain);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorPIDFilter(int ActuatorAdress, float filterP, float filterI, float filterD);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorAddress(int ActuatorAdress, int newAddress);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetGeneralInformations(GeneralInformations &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetFrameType(int frameType);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianForceMinMax(CartesianInfo min, CartesianInfo max);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianInertiaDamping(CartesianInfo inertia, CartesianInfo damping);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetAngularTorqueMinMax(AngularInfo min, AngularInfo max);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetAngularInertiaDamping(AngularInfo inertia, AngularInfo damping);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetDevValue(std::vector<float> command);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetDevValue(std::vector<float> &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetSpasmFilterValues(std::vector<float> Response, int activationStatus);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetSpasmFilterValues(std::vector<float> &Response, int &activationStatus);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int MoveHome();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetAngularForceGravityFree(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetActuatorAcceleration(AngularAcceleration &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int InitFingers();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int GetPeripheralInventory(std::vector<PeripheralInfo> &);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int SetModel(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);

