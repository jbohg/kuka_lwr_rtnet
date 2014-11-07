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

static const string ip_left = "192.168.0.20";
static const string ip_right = "192.168.1.20";

string ip;

typedef struct{
  long time;
  int num_received_messages;
  float cur_jnt_vals[LBR_MNJ];
  int quality;
  int mode;
  int control_mode;
  string message;
  string cmd;
  string msr;
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



void logTask()
{
  int fd;
  FILE *log_file = fopen("log_file.dat","w");
  
  loop_monitoring log;
  
  fd = open("/proc/xenomai/registry/native/pipes/log_pipe", O_RDONLY);
  
  if (fd < 0) {
    printf("cannot open log pipe\n");
    return;
  }


  size_t size = 0;
  bool reading = true;
  
  if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
    reading = false;
  

  while ( reading || going )
    {
      if(reading)
	{
	  fprintf(log_file, "%ld ", log.time);
	  fprintf(log_file, "%d ", log.num_received_messages);
	  fprintf(log_file, "%s", log.message.c_str());
	  for (int i = 0; i < LBR_MNJ; i++)
	    fprintf(log_file, "%f ", log.cur_jnt_vals[i]);
	  
	  fprintf(log_file, "%d ", log.quality);
	  fprintf(log_file, "%d ", log.control_mode);
	  fprintf(log_file, "%d ", log.mode);
	  fprintf(log_file, "\n");
	  fprintf(log_file, "MSR...............\n");
	  fprintf(log_file, "%s\n", log.msr.c_str());
	  fprintf(log_file, "CMD...............\n");
	  fprintf(log_file, "%s\n", log.cmd.c_str());
	  fprintf(log_file, "\n");
	}
      reading = true;
      if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
	reading = false;
    }
  
  close(fd);
  fclose(log_file);
  
  return;
}

void mainControlLoop(void* cookie)
{
  signal(SIGXCPU, warnOnSwitchToSecondaryMode);
  
  //  rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);
  rt_task_set_mode(0, T_WARNSW, NULL);

  
  //memory allocation
  loop_monitoring log;
  long t_1 = long(rt_timer_ticks2ns(rt_timer_read()));
  

  friRemote friInst(49938, ip.c_str());
  //  friRemote friInst;
  FRI_QUALITY lastQuality = FRI_QUALITY_BAD;
  FRI_CTRL lastCtrlScheme = FRI_CTRL_OTHER;
  
  /* enter main loop - wait until we enter stable command mode */
  while(going)
    {
      friInst.doReceiveData();
      log.msr = friInst.getCurrentRecvMsr();
      
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

      /*
      int divider = (int)( (1./friInst.getSampleTime()));
      if ( friInst.getSequenceCount() % divider == 0)
       	{
       	  std::stringstream buffer;
       	  buffer << "krl interaction \n" << friInst.getMsrBuf().krl 
       		 << "intf stat interaction \n" << friInst.getMsrBuf().intf.stat 
       		 << "smpl " << friInst.getSampleTime() << endl;
	  
       	  log.message = buffer.str();
	  
	  
       	} else {
       	log.message = "";
      }
      */

      if ( lastCtrlScheme != friInst.getCurrentControlScheme())
	{
	  lastCtrlScheme = friInst.getCurrentControlScheme();
	  log.message += "control scheme changed\n";
	  log.control_mode = lastCtrlScheme;
	} 
      else 
	{
	  log.message = "";
	}
      

      switch ( friInst.getCurrentControlScheme())
	{
	case   FRI_CTRL_JNT_IMP:
	  {
	    log.message += "Control scheme is JNT impedance control\n";
	    float newJntVals[LBR_MNJ];
	    float newJntStiff[LBR_MNJ];
	    float newJntDamp[LBR_MNJ];
	    float newJntAddTorque[LBR_MNJ];
	    for (int i = 0; i < LBR_MNJ; i++)
	      {
		newJntVals[i] = friInst.getMsrCmdJntPosition()[i];
		newJntStiff[i] = 0.0;
		newJntDamp[i] = 0.0;
		newJntAddTorque[i] = 0.0;
	      }	    
	    
	    for (int i = 0; i < LBR_MNJ; i++)
	      {
		log.cur_jnt_vals[i] = newJntVals[i];
	      }
	    
	    // Call to data exchange - and the like 
	    friInst.doJntImpedanceControl(NULL,//newJntVals, 
					  newJntStiff, 
					  newJntDamp, 
					  newJntAddTorque, 
					  false);
	    
	    log.message+=friInst.getCurrentCommandFlags();
	  }
	  break;
	default:
	  /* do nothing - just data exchange for waiting */
	  friInst.doTest();
	}

      // Send packages 
      friInst.doSendData();
      log.cmd = friInst.getCurrentSentCmd();

      // Stop request is issued from the other side
      if ( friInst.getFrmKRLInt(0) == -1) 
	{
	  cout << "leaving \n";
	  break;	  
	}

      
      if ( friInst.getQuality() != lastQuality)
       	{
       	  log.message += "quality change detected\n";
	} 
      
      
      // log content of message 
      log.num_received_messages++;
      

      
      if(lastQuality >= FRI_QUALITY_OK)
       	log.quality = 1;
      else 
       	log.quality = 0;
      
      if( friInst.getState() == FRI_STATE_CMD)
       	log.mode = 1;
      else 
       	log.mode = 0;

      rt_pipe_write(&log_pipe,&log,sizeof(log), P_NORMAL);
      log.time = long(rt_timer_ticks2ns(rt_timer_read())) - t_1;
      
    }
}



int 
#ifdef __WIN32

_tmain
#else
#ifdef _WRS_KERNEL
friSecondApp
#else
main
#endif
#endif
(int argc, char *argv[])
{

  if(argc>2) {
    cout << "Wrong number of arguments" << endl << "Usage: " << argv[0] << " [left|right]" << endl;
    exit(-1);
  }

  if(argc==2){
    if(strcmp (argv[1], "left") == 0)
      ip = ip_left;
    else if(strcmp (argv[1], "right") == 0) {
      ip = ip_right;
    } else {
      cout << "Wrong option." << endl << "Usage: " << argv[0] << " [left|right]" << endl;
      exit(-1);
    }
  } else if(argc==1) {
    ip = ip_left;
  }

  cout << "Using IP " << ip << " for " << argv[1] << " arm." << endl;
  
  int tmp = 0;
  
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

  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "fri_gravcomp_rt", 50, 0);
  
  
  if((tmp = rt_pipe_create(&log_pipe, "log_pipe", P_MINOR_AUTO, 0)))
    {
      std::cout << "cannot create print pipe, error " << tmp << std::endl;
      return 1;
    }

  boost::thread log_thread(logTask);
  

  rt_task_create(&task, "Real time loop", 0, 50, T_JOINABLE | T_FPU);
  rt_task_start(&task, &mainControlLoop, NULL);
  rt_task_sleep(1e6);  
  
  std::cout << "Press [Enter] to exit.\n";
  waitForEnter();
  std::cout << "exiting\n";
  
  going = false;
  rt_task_join(&task);

  rt_pipe_delete(&log_pipe);
  
  log_thread.join();

  return EXIT_SUCCESS;
}  

/* @} */
