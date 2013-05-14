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
  frifirstapp_rt.cpp
       
  Based on Kuka FRI examples.
  The Sample application just interacts to FRI w.r.t. joint position 
  commands and performs a sine curve using position control.
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
int frequency = 1000; //in Hz
double T_s = 1.0/double(frequency);

static const string ip_left = "192.168.0.20";
static const string ip_right = "192.168.1.20";

RT_PIPE log_pipe;
RT_TASK task;

long previous_read, before_read, before_write;
       

typedef struct{
  //  long time;
  //  int num_received_messages;
  //  float cur_jnt_vals[LBR_MNJ];
  //  int quality;
  //  int mode;
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
	  /*	  fprintf(log_file, "%ld ", log.time);
	  fprintf(log_file, "%d ", log.num_received_messages);
	  for (int i = 0; i < LBR_MNJ; i++)
	    fprintf(log_file, "%f ", log.cur_jnt_vals[i]);
	  
	  fprintf(log_file, "%d ", log.quality);
	  fprintf(log_file, "%d ", log.mode);
	  fprintf(log_file, "\n");
	  */
	  fprintf(log_file, "%s ", log.message.c_str());
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
  
  rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);
  rt_task_set_mode(0, T_WARNSW, NULL);  
  rt_task_wait_period(NULL);

  friRemote friInst(49938, ip_right.c_str());

  int ret;
  unsigned long overrun;
  
  //memory allocation
  loop_monitoring log;
  
  FRI_QUALITY lastQuality = FRI_QUALITY_BAD;
  int res;
  before_read = long(rt_timer_ticks2ns(rt_timer_read()));
   
  string cmd;
  
  /* enter main loop - wait until we enter stable command mode */
  while(going)
    {
      ret = rt_task_wait_period(&overrun);
      before_read = long(rt_timer_ticks2ns(rt_timer_read()));

      std::stringstream buffer;
      if(ret !=0 )
	buffer << "Timing error\n";
      if (ret == -EWOULDBLOCK) {
	buffer << "EWOULBLOCK while rt_task_wait_period. Overrun: " << overrun << "\n";
      } else if(ret == -EINTR){
	buffer << "EINTR while rt_task_wait_period. Overrun: " << overrun << "\n";
      } else if(ret == -ETIMEDOUT){
	buffer << "ETIMEDOUT while rt_task_wait_period. Overrun: " << overrun << "\n";
      } else if(ret == -EPERM){
	buffer << "EPERM while rt_task_wait_period\n";
      }      
      
      res = friInst.doReceiveData();
      
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
      
      // Mirror old joint values 
      friInst.doTest();
      
      before_write = long(rt_timer_ticks2ns(rt_timer_read()));
      // Send packages if a package has been received 
      // (don't flood the socket with CMDs -> this will lead to a can bus error on the robot)
      if(res ==0 ){
	friInst.doSendData();
	cmd = friInst.getCurrentSentCmd ();

	
	buffer << "Time since last valid read: " <<  (before_read-previous_read) / 1000000 
	       << "." << setfill('0') << setw(6) <<(before_read-previous_read) % 1000000 << "ms" << endl;
	buffer << cmd;
	log.message = buffer.str();
	rt_pipe_write(&log_pipe,&log,sizeof(log), P_NORMAL);
	previous_read = before_read;
      }
    }

  return;

}


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

  // std::string ans;
  int tmp = 0;

  // //the argument sets the frequency
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

  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "fri_first_rt", 50, 0);


  cout << "Opening FRI Version " 
       << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION 
       << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
       << " Interface for First Sample" << endl;
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
