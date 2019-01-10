////////////////////////////////////////////////////////////////////////////
//
//    File: messages.h
//
//    Usage:
//        See lidarserver.h
//
//    Description:
//        See lidarserver.h
//        See messages.cc
//
//        This file defines all the message types which the laser
//		  range finder supports and we also support (or at least plan to).
//
//    Written By:
//        Eric Seidel
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MESSAGES_H
#define MESSAGES_H

const unsigned char kLMSTelegramSTX					= 0x02;
const unsigned char kLMSTelegramAllDevicesAddress   = 0x00;
const unsigned char kLMSTelegramHostComputerAddress	= 0x80;
const ulong_t		kLMSMaximumTelegramLength		= 812;

enum LMSIncomingTelegrams {
	kLMSNoTelegram								= 0x00,
	kLMSPowerOnTelegram							= 0x90,
	kLMSConfirmSoftwareResetTelegram			= 0x91,
	kLMSNotAcknowledgedTelegram					= 0x92,
	kLMSRespondChangeOperatingModeTelegram		= 0xA0,
	kLMSRespondValuesTelegram					= 0xB0,
	kLMSRespondSensorStatusTelgram				= 0xB1,
	kLMSRespondErrorOrTestTelegram				= 0xB2,
	//kConfirmNewPasswordTelegram				= 0xC2,
	kLMSRespondCurrentLMSConfiguration			= 0xF4,
	kLMSRespondValuesPlusReflectivityTelegram   = 0xF5,
	kLMSRespondDefineLMSConfiguration 			= 0xF7
};

enum LMSOutgoingTelegrams {
	kLMSInitializationTelegram = 0x10,
	kLMSChangeOperatingModeTelegram				= 0x20,
	kLMSRequestValuesTelegram					= 0x30,
	kLMSRequestStatusTelegram					= 0x31,
	kLMSRequestErrorTelegram					= 0x32,
	//kLMSRequestTestTelegram					= 0x33,
	//kLMSChangePasswordTelegram					= 0x42,
	//kLMSStartCalibrationTelegram				= 0x50,
	kLMSSetPermenantBaudRateTelegram			= 0x66,
	kLMSReadLMSConfiguration					= 0x74,
	kLMSRequestValuesPlusReflectivityTelegram   = 0x75,
	kLMSRequestCartesianCoordinatesTelegram		= 0x76,
	kLMSDefineLMSConfiguration					= 0x77
};

enum LMSOperatingModes {
	kLMSInstallationMode				= 0x00,
	kLMSCalibrationMode					= 0x01,
	kLMSResetPassword					= 0x02, // only possible from diagnostic mode.
	KLMSDiagnosticMode					= 0x03,
	kLMSContinuousMinimumValuesMode		= 0x20,	// 0.5 degree apart, 37.5Hz.
	kLMSContinuousAllValuesMode			= 0x24,
	kLMSContinuousPartialValuesMode = 0x2a,				// 1 degree apart, 75Hz.
	// many others.
	kLMSConfigTo38400Baud				= 0x40,
	kLMSConfigTo19200Baud				= 0x41,
	kLMSConfigTo9600Baud				= 0x42,
	kLMSConfigTo500000Baud				= 0x43
};

// FIX -- THESE ALL SHOULD BE DOUBLE CHECKED!!!

enum LMSStatusByte {
	kLMSStatusErrorMask				= 0x07,
	kLMSDataSourceMask				= 0x18,
	kLMSDataSourceLMS6Mask			= 0x10,
	kLMSRestartStateMask			= 0x20,
	kLMSImplausableValueMask		= 0x40,
	kLMSPolutionMask				= 0x80
};

// FIX -- THESE ALL SHOULD BE DOUBLE CHECKED!!!
enum LMSMeasuredValues {
	kLMSMeasuredValuesCountMask		= 0x03FF,
	kLMSPartialScanRegionMask		= 0x0C00,
	kLMSPartialScanRegionXOOMask	= 0x0000,
	kLMSPartialScanRegionX25Mask	= 0x0400,
	kLMSPartialScanRegionX50Mask	= 0x0800,
	kLMSPartialScanRegionX75Mask	= 0x0C00,
	kLMSPartialScanMask				= 0x2000,
	kLMSMeasuredValueUnitMask		= 0xC000,
	kLMSMeasuredValueUnitCM			= 0x0000,
	kLMSMeasuredValueUnitMM			= 0x4000
};

// FIX -- THESE ALL SHOULD BE DOUBLE CHECKED!!!
enum LMSMeasuredValueData   {
	kLMSMeasuredDistanceMask		= 0x1FFF,
	kLMSDazzleFlagMask				= 0x2000,
	kLMSFieldBFlagMask				= 0x4000,
	kLMSFieldAFlagMask				= 0x8000
};

enum LMSConfigurationValues {
	kLMSDefaultBlanking				= 0x00,
	kLMSDefaultPeakThreshold		= (70),
	kLMSDefaultStopThreshold		= 0x00,
	kLMSDefaultAvailablity			= 0x00,
	kLMSHighAvailability				= 0x01,			// disables shutdown on faults
	kLMSSendRealtimeIndicies		= 0x02,
	kLMSDefaultMeasurementMode		= 0x00,
	kLMSDefaultUnits				= (1),
	kLMSUseCM						= (0),
	kLMSUseMM						= (1),
	kLMSDefaultTemporaryField		= 0x00,
	kLMSDefaultSubtractiveFields	= 0x00,
	kLMSDefaultMultipleEvaluation   = (2)
};

#endif