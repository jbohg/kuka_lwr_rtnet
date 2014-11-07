/*********************************************************************
 *
 *  Copyright (c) 2012, Jeannette Bohg - MPI for Intelligent Systems
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/********************************************************************* 
 FRI_SecondApp_rt.cpp

 The second -more versatile- sample of FRI Usage using an impedance 
 controller to perform the sine wave.
*******************************************************************/




#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifdef __WIN32
#include "stdafx.h"
#endif

#include <native/task.h>
#include <native/pipe.h>
#include <native/timer.h>
#include <sys/mman.h>

#include <signal.h>

#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include <fcntl.h>
#include <boost/thread.hpp>
#include "friudp_rt.h"
#include "friremote_rt.h"

#ifndef M_PI 
#define M_PI 3.14159
#endif

using namespace std;

bool going = true;
int frequency = 500; //in Hz
double T_s = 1.0/double(500);

RT_PIPE log_pipe;
RT_TASK task;

typedef struct{
  long time;
  int num_received_messages;
  float cur_jnt_vals[LBR_MNJ];
  int quality;
  int mode;
  string message;
} loop_monitoring;

void waitForEnter()
{
  std::string line;
  std::getline(std::cin, line);
  std::cout << line << std::endl;
}

void warnOnSwitchToSecondaryMode(int)
{
  std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

void mainControlLoop(void* cookie)
{
  signal(SIGXCPU, warnOnSwitchToSecondaryMode);
  
  //  rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);
  rt_task_set_mode(0, T_WARNSW, NULL);

  friRemote friInst(49938, "192.168.0.20");
  //  friRemote friInst;
  FRI_QUALITY lastQuality = FRI_QUALITY_BAD;
  FRI_CTRL lastCtrlScheme = FRI_CTRL_OTHER;
  double timeCounter=0;
  
  /* enter main loop - wait until we enter stable command mode */
  while(going)
    {
      friInst.doReceiveData();
      
      /// perform some arbitrary handshake to KRL -- possible in monitor mode already
      // send to krl int a value
      friInst.setToKRLInt(0,1);
      lastQuality = friInst.getQuality();
      if ( lastQuality >= FRI_QUALITY_OK)
	{
	  // send a second marker
	  friInst.setToKRLInt(0,10);
	}
      
      //
      // just mirror the real value..
      //
      friInst.setToKRLReal(0,friInst.getFrmKRLReal(1));

      if ( lastCtrlScheme != friInst.getCurrentControlScheme())
	{
	  //cout << "switching control scheme " << lastCtrlScheme;
	  lastCtrlScheme = friInst.getCurrentControlScheme();
	  //	  cout << " to " << lastCtrlScheme;
	}
      
      // Prepare a new position command - if we are in command mode
      switch ( friInst.getCurrentControlScheme())
	{
	case   FRI_CTRL_POSITION:
	case   FRI_CTRL_JNT_IMP:
	  {
	    
	    float newJntVals[LBR_MNJ];
	    for (int i = 0; i < LBR_MNJ; i++)
	      {
		newJntVals[i] = friInst.getMsrCmdJntPosition()[i];
	      }

	    /* Sample - if in command mode - and motor on - 
	       perform some sort of sinewave motion */
	    if ( friInst.getState() == FRI_STATE_CMD)
	      {
		if ( friInst.isPowerOn() )
		  {
		    timeCounter+=friInst.getSampleTime();
		    for (int i = 0; i < LBR_MNJ; i++)
		      {
			// perform some sort of sine wave motion
			
			newJntVals[i]+=(float)sin( timeCounter * M_PI * 0.3) 
			  * (float)(10./180.*M_PI);
		      }
		  }
		else
		  {
		    timeCounter=0.;
		  }
	      }
	    else
	      {
		timeCounter=0.;
	      }
	    // Call to data exchange - and the like 
	    friInst.doPositionControl(newJntVals, false);
	  }
	  break;
	case FRI_CTRL_CART_IMP:
	  /** joint/cart positions, joint/cart stiffness, joint/cart damping
	      and additional TCP F/T can be commanded */
	  {
	    float newCartVals[FRI_CART_FRM_DIM];
	    float newForceTorqueAdd[FRI_CART_VEC];
	    
	    for (int i = 0; i < FRI_CART_FRM_DIM; i++)
	      {
		newCartVals[i] = friInst.getMsrCmdCartPosition()[i];
	      }
	    for (int i = 0; i < FRI_CART_VEC; i++)
	      {
		newForceTorqueAdd[i] = 0.0;
	      }

	    
	    if ( friInst.getState() == FRI_STATE_CMD)
	      {
		if ( friInst.isPowerOn() )
		  {
		    if ( friInst.getFrmKRLInt(1)  >= 1 )
		      {
			/// do force superposition...
			/// if the KRL Side has set $FRI_TO_INT[2] >= 1
			for (int i = 0; i < 3; i++)
			  newForceTorqueAdd[i]+=(float)sin( timeCounter * M_PI * 0.03) * (10.);
		      }
		    if ( friInst.getFrmKRLInt(1) <= 2)
		      {
			/// do Cartesian position superposition
			/// if the KRL Side has set $FRI_TO_INT[2] <= 2
			timeCounter+=friInst.getSampleTime();
			for (int i = 1; i <= 3; i++)
			  {
			    // perform some sort of sine wave motion
			    newCartVals[(i*4)-1]+=(float)sin( timeCounter * M_PI * 0.3) * (0.1f);
			  }
		      }
		  }
		else
		  {
		    timeCounter=0.;
		  }
	      }
	    else
	      {
		timeCounter=0.;
	      }
	    // Compute new values. Data exchange triggered from the top level
	    friInst.doCartesianImpedanceControl(newCartVals,NULL,NULL,newForceTorqueAdd, NULL, false);
	  }
	  break;
	default:
	  /* do nothing - just data exchange for waiting */
	  friInst.doTest();
	}

      // Send packages 
      friInst.doSendData();
     
      // Stop request is issued from the other side
      if ( friInst.getFrmKRLInt(0) == -1) 
	{
	  cout << "leaving \n";
	  break;	  
	}
    }
}



int main (int argc, char *argv[])
{

 mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "fri_second_rt", 50, 0);


  cout << "Opening FRI Version " 
       << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION 
       << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
       << " Interface for Second Sample" << endl;
  {
    // do checks, whether the interface - and the host meets the requirements
    // Note:: This Check remains in friRemote.cpp -- should go to your code ...
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
  

  rt_task_create(&task, "Real time loop", 0, 50, T_JOINABLE | T_FPU);
  rt_task_start(&task, &mainControlLoop, NULL);
  rt_task_sleep(1e6);  
  
  std::cout << "Press [Enter] to exit.\n";
  waitForEnter();
  std::cout << "exiting\n";
  
  going = false;
  rt_task_join(&task);

  return EXIT_SUCCESS;
}  

/* @} */
