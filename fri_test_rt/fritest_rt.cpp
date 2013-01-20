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

 /********************************************************************
   fritest_rt.cpp
    Based on KUKA version of FRI examples.
   The Sample application just interacts to FRI w.r.t. 
   joint position commands and copies back whatever it receives into 
   command structure. 
   The major goal is to understand FRI concepts and mechanism and to test
   the data rate of the RTNet connection.
 *******************************************************************/


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
int mainLoopCounter = 0;

// int frequency = 500; //in Hz
// double T_s = 1.0/double(500);

RT_PIPE log_pipe;
RT_TASK task;

typedef struct{
  long time;
  int num_received_messages;
  float cur_jnt_vals[LBR_MNJ];
  int quality;
  int mode;
  char message[1024];
  int seqCount;
  int reflSeqCount;
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

      //      std::cout << "in log loop" << std::endl;
      if(reading)
	{
	  if(log.message[0]!=0){
	    fprintf(log_file, "Time %ld\n", log.time);
	    // fprintf(log_file, "%d ", log.num_received_messages);
	    // for (int i = 0; i < LBR_MNJ; i++)
	    //   fprintf(log_file, "%f ", log.cur_jnt_vals[i]);
	    
	    fprintf(log_file, "Quality %d\n", log.quality);
	    fprintf(log_file, "Mode %d ", log.mode);
	    // fprintf(log_file, "\n");
	    fprintf(log_file, "%s \n\n", log.message);
	  }
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
  //rt_task_wait_period(NULL);
  

  friRemote friInst(49938, "192.168.0.20");
  //  friRemote friInst;
  FRI_QUALITY lastQuality = FRI_QUALITY_BAD;
  /* enter main loop - wait until we enter stable command mode */
  while(going)
    {
      mainLoopCounter++;
      //      std::cout << "entering main loop " << std::endl;

      //friInst.doReceiveData();
       if(friInst.doReceiveData()==0) {

	 //  std::cout << "entering main loop " << std::endl;

	 // have some debug information every n.th. step
	 int divider = (int)( (1./friInst.getSampleTime()) *0.5);
	 std::stringstream buffer;
	 if ( friInst.getSequenceCount() % divider == 0)
	   {
	     buffer << "krl interaction \n" << friInst.getMsrBuf().krl 
		    << "intf stat interaction \n" << friInst.getMsrBuf().intf.stat 
		    << "smpl " << friInst.getSampleTime() << std::endl
		    << "sequence count " << friInst.getSequenceCount() << std::endl
		    << "refl sequence count " << friInst.getReflSequenceCount() << std::endl;
	     
	   } 
	 else 
	   {
	     buffer << "";
	   }
	 
	 
	 if ( friInst.getQuality() != lastQuality)
	   {
	     buffer<< "QQQQuality change detected at" 
		   << "sequence count " << friInst.getSequenceCount() << std::endl
		   << "refl sequence count " << friInst.getReflSequenceCount() << std::endl;;
	   } 
	 
	 sprintf(log.message, "%s\n", buffer.str().c_str());
	 
	 
	 if(lastQuality >= FRI_QUALITY_OK)
	   log.quality = 1;
	 else 
	   log.quality = 0;
	   
	 
	 if( friInst.getState() == FRI_STATE_CMD)
	   log.mode = 1;
	 else 
	   log.mode = 0;
	 
       }

       rt_pipe_write(&log_pipe,&log,sizeof(log), P_NORMAL);
       log.time = long(rt_timer_ticks2ns(rt_timer_read())) - t_1;

    /* and leave it on */

      
      /// perform some arbitrary handshake to KRL -- possible in monitor mode already
      // send to krl int a value
      friInst.setToKRLInt(0,1);
      lastQuality = friInst.getQuality();
      if ( lastQuality >= FRI_QUALITY_OK)
	{
	  // send a second marker
	  friInst.setToKRLInt(0,10);
	}  /* else 
	{
	  
	   std::cout << "Quality not ok!" << std::endl;
	   }
	   */
      //
      // just mirror the real value..
      //
      friInst.setToKRLReal(0,friInst.getFrmKRLReal(1));
	

      // Mirror old joint values 
      friInst.doTest();
      
      if(mainLoopCounter>=3000 && mainLoopCounter<=3200)
	continue;
      
      // Send packages 
      friInst.doSendData();

      //rt_task_wait_period(NULL);
    }

  return;
}

int main (int argc, char *argv[])
{
  std::string ans;
  int tmp = 0;

  //the argument sets the frequency
  // printf("please enter frequency of operation [%d]: ", frequency);
  // getline(std::cin,ans);
  // if ( (tmp = atoi(ans.c_str())))
  //   {
  //   if( (tmp>10) && (tmp < 1200) )
  //   {
  //     frequency = tmp;
  //     T_s = (double) (1 / (double) frequency);
  //   }
  // }
  // printf("frequency set to %d, period is %f\n", frequency, T_s);
  
  cout << "Opening FRI Version " 
       << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION 
       << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
       << " Interface for Communication Test" << endl;
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
  rt_task_shadow(NULL, "fri_test_rt", 50, 0);
  
  
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
