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
  friremote_rt.cpp

   Based on KUKA version of FRI interface.
   Remote class for handshaking/dealing with udp datastructures
   This code is most important to understand the concepts behind 
   data handshake. Extended for RTNet use.
*/

#ifndef FRIFRIREMOTE_H
#define FRIFRIREMOTE_H


#include "friudp_rt.h"

class friRemote
{
 public:
  friRemote(int port = FRI_DEFAULT_SERVER_PORT, const char * hintToRemoteHost=NULL);
  ~friRemote();

  /** Data Exchanger -- normally update within access routine implicitely 
      send commands based on last datagram and after waits on new measurement
      calls doSendData() and doReceiveData();
      ... */
  int doDataExchange();
  /** Receives data while calling friUdp::Recv()
      The call will block..
  */
  int doReceiveData();
  /** Sends the data */
  int doSendData();

  /** Mirrors the current joint positions **/
  void doTest();

  /* @{ */
  /** automatically do data exchange, if not otherwise specified 
      if flagDataExchange is set to false, call doDataExchange() 
      or doReceiveData()/doSendData() on your own
  */
  int doPositionControl(float newJntPosition[LBR_MNJ], bool flagDataExchange=true);

  /** automatically do data exchange, if not otherwise specified 
      if flagDataExchange is set to false, call doDataExchange() 
      or doReceiveData()/doSendData() on your own
      IN: newJntPosition   - joint positions
      newJntStiff      - joint stiffness (Spring factor)
      newJntDamp       - joint damping   (Damping factor)
      newJntAddTorque  - additional torque 

      Note: If any of the pointers (newJntPosition, newJntStiff, newJntDamp, newJntAddTorque) 
      is NULL, the value is ignored -> the respective  cmd.cmd.cmdFlags field is set properly
      Note: It is possible to change cmd.cmd.cmdFlags in monitor mode only !!
  */
  int doJntImpedanceControl(const float newJntPosition[LBR_MNJ], 
			    const float newJntStiff[LBR_MNJ] = NULL, 
			    const float newJntDamp[LBR_MNJ]=NULL, 
			    const float newJntAddTorque[LBR_MNJ]=NULL,
			    bool flagDataExchange=true);


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
  int doCartesianImpedanceControl(const float newCartPosition[FRI_CART_FRM_DIM], 
				  const float newCartStiff[FRI_CART_VEC]=NULL, 
				  const float newCartDamp[FRI_CART_VEC]=NULL, 
				  const float newAddTcpFT[FRI_CART_VEC]=NULL,
				  const float newJntNullspace[LBR_MNJ]=NULL,  
				  bool flagDataExchange=true);
  /* @} */

  /** measured Cartesian frame (in m)
      KRL: $POS_ACT_MSR
      Reference: Base and tool are specified by $stiffness.base, $stiffness.tool 
  */
  float * getMsrCartPosition() { return msr.data.msrCartPos; }
  /** commanded Cartesian frame (in m, before FRI) 
      KRL: $POS_ACT_CMD
      Reference: Base and tool are specified by $stiffness.base, $stiffness.tool 
  */
  float * getMsrCmdCartPosition() { return msr.data.cmdCartPos; }

  /** commanded Cartesian frame (in m, due to FRI) */
  float * getMsrCmdCartPosFriOffset() { return msr.data.cmdCartPosFriOffset; }

  /** Access to inner buffers for further manipulation */
  tFriMsrData & getMsrBuf() { return msr;}
  tFriCmdData & getCmdBuf() { return cmd;}
  /* @{ */
  /** interpretational access routines */
  FRI_STATE getState()      { return (FRI_STATE)msr.intf.state;   }
  FRI_QUALITY getQuality()  { return (FRI_QUALITY)msr.intf.quality; }
  FRI_CTRL getCurrentControlScheme (){ return (FRI_CTRL)msr.robot.control; }

  bool isPowerOn() { return msr.robot.power!=0; }
  /* @} */

  /* @{ */
  /** Important value for superposition - and during poweroff stages, to become command mode */
  float * getMsrCmdJntPosition() { return msr.data.cmdJntPos; }
  /** returns the offset, which is commanded by FRI Remote side 
   *  Complete desired position inside LBR Kernel is the sum of cmdJntPos and cmdJntPosFriOffset */
  float * getMsrCmdJntPositionOffset() { return msr.data.cmdJntPosFriOffset; }

  void getCurrentCmdJntPosition( float jntVec[LBR_MNJ] ) { for ( int i = 0; i < LBR_MNJ; i++) jntVec[i]= msr.data.cmdJntPos[i]+msr.data.cmdJntPosFriOffset[i];}
  /** Current measured jnt position of the robot */
  float * getMsrMsrJntPosition() { return msr.data.msrJntPos; }
  float * getMsrEstExtJntTrq() { return msr.data.estExtJntTrq; }
  float * getMsrJntTrq() { return msr.data.msrJntTrq; }
  /* @} */

  float getSampleTime() { return msr.intf.desiredCmdSampleTime; }
  int getSequenceCount() { return seqCount; }
  int getReflSequenceCount() { return msr.head.sendSeqCount; }

  /* @{ */
  /** KRL Interaction -- Corresponds to $FRI_TO_REA */
  float getFrmKRLReal(int index) { return msr.krl.realData[index]; }
  /** KRL Interaction -- Corresponds to $FRI_FRM_REA */
  void  setToKRLReal(int index, float val) { cmd.krl.realData[index]=val; }
  /** KRL Interaction -- Corresponds to $FRI_TO_INT */
  int   getFrmKRLInt(int index) { return msr.krl.intData[index]; }
  /** KRL Interaction -- Corresponds to $FRI_FRM_INT */
  void  setToKRLInt(int index, int val) { cmd.krl.intData[index]=val; }
  /** KRL Interaction -- Corresponds to $FRI_TO_BOOL */
  bool  getFrmKRLBool(int index) { return ((msr.krl.boolData & (1<<index)) != 0);}
  /** KRL Interaction -- Corresponds to $FRI_FRM_BOOL */
  fri_uint16_t getFrmKRLBool() { return msr.krl.boolData; }
  /** KRL Interaction -- Corresponds to $FRI_FRM_BOOL */
  void  setToKRLBool(int index, bool val) 
  { if ( val) { cmd.krl.boolData |= (1<<index);} 
    else {cmd.krl.boolData &= (~(1<<index));}}
  void setToKRLBool(fri_uint16_t val) {   cmd.krl.boolData = val; }
  /* @} */
 protected:
  tFriMsrData msr;
  tFriCmdData cmd;
 private:
  friUdp remote;
  int seqCount;
};




#endif
/* @} */
