/*! \file
	\brief	Provides Standard return values for Xsens functions

	\section FileCopyright Copyright Notice
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/

/*! \cond NODOXYGEN */

#ifndef XSENS_STD_H
#define XSENS_STD_H

//////////////////////////////////////////////////////////////////////////////////////////

#if defined(_WIN32) && !defined(_WIN32_WINNT)
#	define _WIN32_WINNT 0x0502
#endif

#ifdef _WIN32
#	include <windows.h>
#endif

/*! \endcond */

//////////////////////////////////////////////////////////////////////////////////////////
// NOTE!!! When adding a value to this list, also add its description in xsens_std.cpp
/*!	\defgroup enums Global enumerations
	\addtogroup enums
	@{
*/
/*!	\brief Xsens return values
	\details These values are used to signal success or specific failures of functions
*/
enum XsensResultValue {
	// general OK
	 XRV_OK					= 0		//!< 0: Operation was performed successfully

	// communication protocol
	,XRV_NOBUS				= 1		//!< 1: No bus communication possible
	,XRV_BUSNOTREADY		= 2		//!< 2: InitBus and/or SetBID are not issued
	,XRV_INVALIDPERIOD		= 3		//!< 3: Period sent is invalid
	,XRV_INVALIDMSG			= 4		//!< 4: The message is invalid or not implemented
	,XRV_INITBUSFAIL1		= 16	//!< 16: A slave did not respond to WaitForSetBID
	,XRV_INITBUSFAIL2		= 17	//!< 17: An incorrect answer received after WaitForSetBID
	,XRV_INITBUSFAIL3		= 18	//!< 18: After four bus-scans still undetected Motion Trackers
	,XRV_SETBIDFAIL1		= 20	//!< 20: No reply to SetBID message during SetBID procedure
	,XRV_SETBIDFAIL2		= 21	//!< 21: Other than SetBIDAck received
	,XRV_MEASUREMENTFAIL1	= 24	//!< 24: Timer overflow - period too short to collect all data from Motion Trackers
	,XRV_MEASUREMENTFAIL2	= 25	//!< 25: Motion Tracker responds with other than SlaveData message
	,XRV_MEASUREMENTFAIL3	= 26	//!< 26: Total bytes of data of Motion Trackers including sample counter exceeds 255 bytes
	,XRV_MEASUREMENTFAIL4	= 27	//!< 27: Timer overflows during measurement
	,XRV_MEASUREMENTFAIL5	= 28	//!< 28: Timer overflows during measurement
	,XRV_MEASUREMENTFAIL6	= 29	//!< 29: No correct response from Motion Tracker during measurement
	,XRV_TIMEROVERFLOW		= 30	//!< 30: Timer overflows during measurement
	,XRV_BAUDRATEINVALID	= 32	//!< 32: Baud rate does not comply with valid range
	//,XRV_PARAMINVALID		= 33	//!< 33: An invalid parameter is supplied
	,XRV_INVALIDPARAM		= 33	//!< 33: An invalid parameter is supplied
	,XRV_MEASUREMENTFAIL7	= 35	//!< 35: TX PC Buffer is full
	,XRV_MEASUREMENTFAIL8	= 36	//!< 36: TX PC Buffer overflow, cannot fit full message
	,XRV_DEVICEERROR		= 40	//!< 40: The device generated an error, try updating the firmware

	// CMT / XME / etc
	,XRV_ERROR					= 256	//!< 256: A generic error occurred
	,XRV_NOTIMPLEMENTED			= 257	//!< 257: Operation not implemented in this version (yet)
	,XRV_TIMEOUT				= 258	//!< 258: A timeout occurred
	,XRV_TIMEOUTNODATA			= 259	//!< 259: Operation aborted because of no data read
	,XRV_CHECKSUMFAULT			= 260	//!< 260: Checksum fault occurred
	,XRV_OUTOFMEMORY			= 261	//!< 261: No internal memory available
	,XRV_NOTFOUND				= 262	//!< 262: The requested item was not found
	,XRV_UNEXPECTEDMSG			= 263	//!< 263: Unexpected message received (e.g. no acknowledge message received)
	,XRV_INVALIDID				= 264	//!< 264: Invalid id supplied
	,XRV_INVALIDOPERATION		= 265		//!< 265: Operation is invalid at this point
	,XRV_INSUFFICIENTSPACE		= 266	//!< 266: Insufficient buffer space available
	,XRV_INPUTCANNOTBEOPENED	= 267	//!< 267: The specified i/o device can not be opened
	,XRV_OUTPUTCANNOTBEOPENED	= 268	//!< 268: The specified i/o device can not be opened
	,XRV_ALREADYOPEN			= 269	//!< 269: An I/O device is already opened with this object
	,XRV_ENDOFFILE				= 270	//!< 270: End of file is reached
	,XRV_COULDNOTREADSETTINGS	= 271	//!< 271: A required settings file could not be opened or is missing some data
	,XRV_NODATA					= 272	//!< 272: No data is available
	,XRV_READONLY				= 273	//!< 273: Tried to change a read-only value
	,XRV_NULLPTR				= 274	//!< 274: Tried to supply a NULL value where it is not allowed
	,XRV_INSUFFICIENTDATA		= 275	//!< 275: Insufficient data was supplied to a function
	,XRV_BUSY					= 276	//!< 276: Busy processing, try again later
	,XRV_INVALIDINSTANCE		= 277	//!< 277: Invalid instance called
	,XRV_DATACORRUPT			= 278	//!< 278: A trusted data stream proves to contain corrupted data

	,XRV_READINITFAILED			= 279	//!< 279: Failure during read of settings
	,XRV_NOXMFOUND				= 280	//!< 280: Could not find any MVN-compatible hardware
	,XRV_ONLYONEXMFOUND			= 281	//!< 281: Found only one responding Xbus Master
	,XRV_MTCOUNTZERO			= 282	//!< 282: No sensors found
	,XRV_MTLOCATIONINVALID		= 283	//!< 283: One or more sensors are not where they were expected
	,XRV_INSUFFICIENTMTS		= 284	//!< 284: Not enough sensors were found
	,XRV_INITFUSIONFAILED		= 285	//!< 285: Failure during initialization of Fusion Engine
	,XRV_OTHER					= 286	//!< 286: Something else was received than was requested

	,XRV_NOFILEOPEN				= 287	//!< 287: No file opened for reading/writing
	,XRV_NOPORTOPEN				= 288	//!< 288: No serial port opened for reading/writing
	,XRV_NOFILEORPORTOPEN		= 289	//!< 289: No file or serial port opened for reading/writing
	,XRV_PORTNOTFOUND			= 290	//!< 290: A required port could not be found
	,XRV_INITPORTFAILED			= 291	//!< 291: The low-level port handler failed to initialize
	,XRV_CALIBRATIONFAILED		= 292	//!< 292: A calibration routine failed

	,XRV_CONFIGCHECKFAIL		= 293	//!< 293: The in-config check of the device failed
	,XRV_ALREADYDONE			= 294	//!< 294: The operation is once only and has already been performed

	,XRV_SYNC_SINGLE_SLAVE		= 295	//!< 295: The single connected device is configured as a slave
	,XRV_SYNC_SECOND_MASTER		= 296	//!< 296: More than one master was detected
	,XRV_SYNC_NO_SYNC			= 297	//!< 297: A device was detected that was neither master nor slave
	,XRV_SYNC_NO_MASTER			= 298	//!< 298: No master detected
	,XRV_SYNC_DATA_MISSING		= 299	//!< 299: A device is not sending enough data

	,XRV_VERSION_TOO_LOW		= 300	//!< 300: The version of the object is too low for the requested operation
	,XRV_VERSION_PROBLEM		= 301	//!< 301: The object has an unrecognized version, so it's not safe to perform the operation

	,XRV_ABORTED				= 302	//!< 302: The process was aborted by an external event, usually a user action or process termination
};
//! @}

//////////////////////////////////////////////////////////////////////////////////////////

/*! \cond NODOXYGEN */
#ifndef DELNUL
//! This macro deletes a pointer and sets it to NULL
#define DELNUL(ptr)		do { delete ptr; ptr = NULL; } while (0);
//! This macro deletes a pointer if it is not NULL and then sets it to NULL
#define CHKDELNUL(ptr)	do { if (ptr != NULL) { delete ptr; ptr = NULL; } } while (0);
//! This macro deletes a multi-object pointer and sets it to NULL
#define LSTDELNUL(ptr)		do { delete[] ptr; ptr = NULL; } while (0);
//! This macro deletes a multi-object pointer if it is not NULL and then sets it to NULL
#define LSTCHKDELNUL(ptr)	do { if (ptr != NULL) { delete[] ptr; ptr = NULL; } } while (0);

//! This macro frees a pointer and then sets it to NULL
#define FREENUL(ptr)	do { free(ptr); ptr = NULL; } while (0);
//! This macro frees a pointer if it is not NULL and then sets it to NULL
#define CHKFREENUL(ptr)	do { if (ptr != NULL) { free(ptr); ptr = NULL; } } while (0);
#endif
/*! \endcond */

//////////////////////////////////////////////////////////////////////////////////////////

/* \brief Retrieve a string corresponding to the given result code.

	This function uses the CmtResultText list to return a string with the relevant result 
	code in a textual format. If the supplied result code is invalid the
	"!!Invalid result code!!" string is returned.
*/
#if !defined(_CMT_DLL_IMPORT) && !defined(_XME_DLL_IMPORT)
const char* xsensResultText(const XsensResultValue result);
#endif

//////////////////////////////////////////////////////////////////////////////////////////

#define XSENS_LABEL_SIZE			256		//!< The maximum length of a label
#define XSENS_SHORT_STRING_SIZE		256		//!< The maximum length of a short string
#define XSENS_LONG_STRING_SIZE		16384	//!< The maximum length of a long string

//////////////////////////////////////////////////////////////////////////////////////////

/*! \cond NODOXYGEN */
#endif	// XSENS_STD_H
/*! \endcond */

