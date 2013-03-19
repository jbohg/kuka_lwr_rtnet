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
   friRemote.cpp
  
   Based on KUKA version of FRI interface.
   FRI Remote class encapsulating UDP package handshakes. Adapted for 
   RTNet use.

 ********************************************************************/
#include "friremote_rt.h"

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <native/task.h>

std::ostringstream stream;

friRemote::friRemote(int port, const char *hintToRemoteHost) 
  : remote(port,hintToRemoteHost)
  , seqCount(0)
  , inCount(0)
  , outCount(0)
{
  std::cout << __FUNCTION__ << " " <<port <<std::endl;
  std::cout << "FRI Version " << FRI_MAJOR_VERSION 
	    << "." << FRI_SUB_VERSION << "." 
	    <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR << std::endl;
  memset((void*)(& msr),0x0,sizeof(msr));
  memset((void*)(& cmd), 0x0,sizeof(cmd));
  {
    // do checks, whether the interface - and the host meets the requirements
    FRI_PREPARE_CHECK_BYTE_ORDER;
    if (!FRI_CHECK_BYTE_ORDER_OK) 
      {
	std::cerr << __FUNCTION__
		  <<"Byte order on your system is not appropriate - expect deep trouble" 
		  <<std::endl;
      }
    if (!FRI_CHECK_SIZES_OK)
      {
	std::cout << __FUNCTION__
		  <<"Sizes of datastructures not appropriate - expect even deeper trouble" 
		  << std::endl;

      }
  }
}

friRemote::~friRemote()
{
  std::cout << __FUNCTION__ << " bye for now "  <<std::endl;
}

int friRemote::doReceiveData()
{
  inCount = msr.head.sendSeqCount;
  int rc = remote.Recv(&msr);
  if(inCount == msr.head.sendSeqCount){
    std::ostringstream stream;
    stream << "RECEIVED old package twice\n";
    sent_cmd = stream.str();
  }

  /*
  std::ostringstream stream;
  stream << msr;
  recv_msr = stream.str();
  */
  return rc;
}


/** Data Exchanger -- normally update within access routine implicitely ... */
int friRemote::doSendData()
{
  
  std::ostringstream stream;
  stream << "Old seqCount in: " << cmd.head.reflSeqCount << std::endl;
  stream << "New seqCount in: " << msr.head.sendSeqCount << std::endl;

  stream << "Old seqCount out: " << msr.head.reflSeqCount << std::endl;
  stream << "New seqCount out: " << seqCount << std::endl;

  sent_cmd = stream.str();

  // received at least something 
  seqCount++;
  cmd.head.sendSeqCount = seqCount;
  cmd.head.reflSeqCount = msr.head.sendSeqCount;
  cmd.head.datagramId = FRI_DATAGRAM_ID_CMD;
  cmd.head.packetSize = sizeof(tFriCmdData);
  int rc=remote.Send(&cmd);

  /*
  std::ostringstream stream;
  stream << cmd;
  sent_cmd = stream.str();
  */

  return rc;
}

/** send commands based on last msr datagram and 
    receives a new one
*/

int friRemote::doDataExchange()
{
  doSendData();
  rt_task_wait_period(NULL);
  return doReceiveData();
}

/** just copy the old data */
void friRemote::doTest()

{
  // just mirror the old values
  cmd.cmd.cmdFlags=FRI_CMD_JNTPOS;
  flags="FRI_CMD_JNTPOS\n";
  for (int i = 0; i < LBR_MNJ; i++)
      cmd.cmd.jntPos[i]=msr.data.cmdJntPos[i]+msr.data.cmdJntPosFriOffset[i];
}

/** automatically do data exchange, if not otherwise specified */
int friRemote::doPositionControl(float newJntPosition[LBR_MNJ], bool flagDataExchange)

{
  // Helper, if not properly initialized or the like...
  
  cmd.cmd.cmdFlags=FRI_CMD_JNTPOS;
  // Note:: If the interface is not in Command mode,
  // The commands have to be "mirrored" to get in sync
  if ((getState() != FRI_STATE_CMD) || (!isPowerOn()))
    {
      for (int i = 0; i < LBR_MNJ; i++)
	{
	  cmd.cmd.jntPos[i]=msr.data.cmdJntPos[i]+msr.data.cmdJntPosFriOffset[i];
	}
    }
  else
    {
      // compute new values here ...
      for (int i = 0; i < LBR_MNJ; i++)
	cmd.cmd.jntPos[i]=newJntPosition[i];
    }

  if (flagDataExchange)
    {
      return doDataExchange();
    }
  return 1;
}


/** automatically do data exchange, if not otherwise specified 
    if flagDataExchange is set to false, call doDataExchange() 
    or doReceiveData()/doSendData() on your own
    IN: newJntPosition   - joint positions
    newJntStiff      - joint stiffness (Spring factor)
    newJntDamp       - joint damping   (Damping factor)
    newJntAddTorque  - additional torque 
       
    Note: If any of the pointers (newJntPosition, newJntStiff, newJntDamp, newJntAddTorque) is NULL, the 
    value is ignored -> the respective  cmd.cmd.cmdFlags field is set properly
    Note: It is possible to change cmd.cmd.cmdFlags in monitor mode only !!
*/
int friRemote::doJntImpedanceControl(const float newJntPosition[LBR_MNJ], 
				     const float newJntStiff[LBR_MNJ], 
				     const float newJntDamp[LBR_MNJ], 
				     const float newJntAddTorque[LBR_MNJ],
				     bool flagDataExchange)

{
  // Helper, if not properly initialized or the like...
  cmd.cmd.cmdFlags=0;
  flags = "";
  if (newJntPosition)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_JNTPOS;
      flags+="FRI_CMD_JNTPOS";
      // Note:: If the interface is not in Command mode,
      // The commands have to be "mirrored" to get in sync
      if ((getState() != FRI_STATE_CMD) || (!isPowerOn()))
	{
	  for (int i = 0; i < LBR_MNJ; i++)
	    {
	      cmd.cmd.jntPos[i]=msr.data.cmdJntPos[i]+msr.data.cmdJntPosFriOffset[i];
	    }
	}
      else
	{
	  // compute new values here ...
	  for (int i = 0; i < LBR_MNJ; i++)
	    cmd.cmd.jntPos[i]=newJntPosition[i];
	}
    }

  if (newJntStiff)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_JNTSTIFF;
      flags+="|FRI_CMD_JNTSTIFF";
      for (int i = 0; i < LBR_MNJ; i++)
	cmd.cmd.jntStiffness[i]=newJntStiff[i];
    }
  if (newJntDamp)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_JNTDAMP;
      flags+="|FRI_CMD_JNTDAMP";
      for (int i = 0; i < LBR_MNJ; i++)
	cmd.cmd.jntDamping[i]=newJntDamp[i];
    }
  if (newJntAddTorque)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_JNTTRQ;
      flags+="|FRI_CMD_JNTTRQ\n";
      for (int i = 0; i < LBR_MNJ; i++)
	cmd.cmd.addJntTrq[i]=newJntAddTorque[i];
    }

  if (flagDataExchange)
    {
      return doDataExchange();
    }
  return 1;
}

/** automatically do data exchange, if not otherwise specified 
    if flagDataExchange is set to false, call doDataExchange() 
    or doReceiveData()/doSendData() on your own
    IN: newJntPosition   - joint positions
    newJntStiff      - joint stiffness (Spring factor)
    newJntDamp       - joint damping   (Damping factor)
    newJntAddTorque  - additional torque 

    Note: If any of the pointers (newJntPosition, newJntStiff, newJntDamp, newJntAddTorque) is NULL, the 
    value is ignored -> the respective  cmd.cmd.cmdFlags field is set properly
    Note: It is possible to change cmd.cmd.cmdFlags in monitor mode only !!
*/
int friRemote::doCartesianImpedanceControl(const float newCartPosition[FRI_CART_FRM_DIM], 
					   const float newCartStiff[FRI_CART_VEC], 
					   const float newCartDamp[FRI_CART_VEC], 
					   const float newAddTcpFT[FRI_CART_VEC],
					   const float newJntNullspace[LBR_MNJ],  
					   bool flagDataExchange)
{

  // Helper, if not properly initialized or the like...
  cmd.cmd.cmdFlags=0;
  if ( newCartPosition )
    {
      cmd.cmd.cmdFlags|=FRI_CMD_CARTPOS;
      for ( int i = 0; i < FRI_CART_FRM_DIM; i++)
	{
	  cmd.cmd.cartPos[i]=newCartPosition[i];

	}
    }
  if ( newCartStiff)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_CARTSTIFF;
      for ( int i = 0; i < FRI_CART_VEC; i++)
	{
	  cmd.cmd.cartStiffness[i]=newCartStiff[i];

	}

    }
  if ( newCartDamp)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_CARTDAMP;
      ;
      for ( int i = 0; i < FRI_CART_VEC; i++)
	{
	  cmd.cmd.cartDamping[i]=newCartDamp[i];

	}
    }
  if ( newAddTcpFT)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_TCPFT;
      ;
      for ( int i = 0; i < FRI_CART_VEC; i++)
	{
	  cmd.cmd.addTcpFT[i]=newAddTcpFT[i];

	}
    }

  if (newJntNullspace)
    {
      cmd.cmd.cmdFlags|=FRI_CMD_JNTPOS;
      // Note:: If the interface is not in Command mode,
      // The commands have to be "mirrored" to get in sync
		
      // compute new values here ...
      for (int i = 0; i < LBR_MNJ; i++)
	cmd.cmd.jntPos[i]=newJntNullspace[i];		
    }


  if (flagDataExchange)
    {
      return doDataExchange();
    }
  return 1;
}






/* @} */

