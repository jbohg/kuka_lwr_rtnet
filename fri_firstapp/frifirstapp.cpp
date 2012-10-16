/*{{[PH]
****************************************************************************
Project:  FRI

This material is the exclusive property of KUKA Roboter GmbH 
and must be returned to KUKA Roboter GmbH immediately upon 
request.  This material and the information illustrated or 
contained herein may not be used, reproduced, stored in a 
retrieval system, or transmitted in whole or in part in any 
way - electronic, mechanical, photocopying, recording, or 
otherwise, without the prior written consent of KUKA Roboter GmbH.  
  
All Rights Reserved
Copyright (C)  2009
KUKA Roboter GmbH
Augsburg, Germany
  
[PH]}}
*/

 /*
   {{[FH]
   ****************************************************************************
   friFirstApp.cpp
       
   NOTE: This sample, as the corresponding FRI (Fast Research inteface) is subject to radical change
	  
      
   [FH]}}
 */ 
 /**
    \defgroup friFirstApp
    The Sample application just interacts to FRI w.r.t. joint position commands
    The major goal is to understand FRI concepts and mechanism 
 */
 /* @{ */

 /** 
     \author (Guenter Schreiber)
     \file
     \brief First Example for KUKA.FastResearchInterface

     Get this one running on your favorite system as proof of concept
     NOTE: This sample, as the corresponding FRI (Fast Research inteface) is subject to radical change

 */


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifdef __WIN32
#include "stdafx.h"
#endif



#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include "friudp.h"
#include "friremote.h"

#ifndef M_PI 
#define M_PI 3.14159
#endif

using namespace std;

int 
#ifdef __WIN32

_tmain
#else
#ifdef _WRS_KERNEL
friFirstApp
#else
main
#endif
#endif
(int argc, char *argv[])
{

  cout << "Opening FRI Version " 
       << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
       << " Interface for First Sample" << endl;
  {
    // do checks, whether the interface - and the host meets the requirements
    // Note:: This Check remains in friRempte.cpp -- should go to your code ...
    FRI_PREPARE_CHECK_BYTE_ORDER;
    if (!FRI_CHECK_BYTE_ORDER_OK) 
      {
	cerr << "Byte order on your system is not appropriate - expect deep trouble" <<endl;
      }
    if (!FRI_CHECK_SIZES_OK)
      {
	cout << "Sizes of datastructures not appropriate - expect even deeper trouble" << endl;

      }
  }
  {
    friRemote friInst;
    FRI_QUALITY lastQuality = FRI_QUALITY_BAD;
    double timeCounter=0;
    /* enter main loop - wait until we enter stable command mode */
    for(;;)
      {
	/// perform some arbitrary handshake to KRL -- possible in monitor mode already
	// send to krl int a value
	friInst.setToKRLInt(0,1);
	if ( friInst.getQuality() >= FRI_QUALITY_OK)
	  {
	    // send a second marker
	    friInst.setToKRLInt(0,10);
	  } else 
	  {
			    
	    std::cout << "Quality not ok!" << std::endl;
	  }

	//
	// just mirror the real value..
	//
	friInst.setToKRLReal(0,friInst.getFrmKRLReal(1));

	// Prepare a new position command - if we are in command mode
	float newJntVals[LBR_MNJ];
	for (int i = 0; i < LBR_MNJ; i++)
	  {
	    newJntVals[i] = friInst.getMsrCmdJntPosition()[i];
	  }

	/** Sample - if in command mode - and motor on - 
	    perform some sort of sinewave motion */
	if ( friInst.getState() == FRI_STATE_CMD)
	  {
	    //	    cout << "We are in CMD mode" << endl;
	    if ( friInst.isPowerOn() )
	      {
		timeCounter+=friInst.getSampleTime();
		for (int i = 0; i < LBR_MNJ; i++)
		  {
		    // perform some sort of sine wave motion

		    newJntVals[i]+=(float)sin( timeCounter * M_PI * 0.1) * (float)(10./180.*M_PI);
		  }
	      }
	    else
	      {
		timeCounter=0.;
	      }
	  }
	else
	  {
	    //	    cout << "We are NOT in CMD mode" << endl;
	    timeCounter=0.;
	  }
	// Call to data exchange - and the like 
	friInst.doPositionControl(newJntVals);

	// have some debug information every n.th. step
	int divider = (int)( (1./friInst.getSampleTime()) *2.0);
			
	if ( friInst.getSequenceCount() % divider == 0)
	  {
	    cout << "krl interaction \n";
	    cout << friInst.getMsrBuf().krl;
	    cout << "intf stat interaction \n";
	    cout << friInst.getMsrBuf().intf.stat;
	    cout << "smpl " << friInst.getSampleTime();

	    cout << endl;
	  }

	// Stop request is issued from the other side
	if ( friInst.getFrmKRLInt(0) == -1) 
	  {
	    cout << "leaving \n";
	    break;	  
	  }
	//
	// Quality change leads to output of statistics
	// for informational reasons
	//
	if ( friInst.getQuality() != lastQuality)
	  {
	    cout << "quality change detected "<< friInst.getQuality()<< " \n";
	    cout << friInst.getMsrBuf().intf;
	    cout << endl;
	    lastQuality=friInst.getQuality();
	  }
      }

    /* and leave it on */
  }

  return EXIT_SUCCESS;
}
/* @} */
