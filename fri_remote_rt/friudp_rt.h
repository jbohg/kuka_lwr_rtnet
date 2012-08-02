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
   friUdp_rt.h

    Based on KUKA version of FRI interface.
    Header for udp Communications. Adapted for RTNet use.
 *******************************************************************/
#ifndef FRIFRIUDPRT_H
#define FRIFRIUDPRT_H

#ifdef VXWORKS
#else
#define HAVE_GETHOSTNAME
#endif

#include <stdio.h>
#include <stdlib.h>

#ifdef _MSC_VER
#include <winsock2.h>
#else
 //#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#ifndef _WRS_KERNEL
#include <sys/time.h>
#endif
#include <time.h>
#endif
#include "fricomm_rt.h"

#include <iostream>

#ifdef VXWORKS // VxWorks Kernel

#include <sockLib.h>
#endif


#define FRI_DEFAULT_SERVER_PORT 49938

#ifdef QNX
#define HAVE_TIME_STAMP_RECEIVE
#endif


#ifdef HAVE_TIME_STAMP_RECEIVE
 /** Receive Timestamp -- mechanism works under 
     QNX, Linux?

     Not under Windows - and under VxWorks
 */

typedef struct
{
  tFriMsrData load;
  /* add a received-timestamp */
  double timestamp;
} tFriMsrPacket;

#endif

/**
   FRI Remote Sample Implementation

   @author Günter Schreiber <guenter@jacobus>
*/

enum NET_return_codes {

  SUCCESS = 0, //!< SUCCESS
  UDP_BUFFER_OVERFLOW, //!< UDP_BUFFER_OVERFLOW
  UDP_SEND_SYSCALL_ERROR,//!< UDP_SEND_SYSCALL_ERROR
  UDP_SEND_SIZE_ERROR, //!< UDP_SEND_SIZE_ERROR
  UDP_SOCKET_UNDEFINED, //!< UDP_SOCKET_UNDEFINED
  UDP_MSG_LENGTH_ERROR, //!< UDP_MSG_LENGHT_ERROR
  UDP_PAYLOAD_ERROR, //!< UDP_PAYLOAD_ERROR
  SDC_MESSAGE_UNDEFINED, //!< SDC_MESSAGE_UNDEFINED
  SDC_ANSWER_MSG_ERROR, //!< SDC_ANSWER_MSG_ERROR
  UDP_RECEIVE_SYSCALL_ERROR, //!< UDP_RECEIVE_SYSCALL_ERROR
  UDP_TIMEOUT_ERROR, //!< UDP_TIMEOUT_ERROR
  SDC_CARD_UNDEFINED, //!< SDC_CARD_UNDEFINED
  SDC_NETWORK_DOWN, //!< SDC_NETWORK_DOWN
  UDP_SEND_TIMEOUT,
  SDC_INVALID_COMMAND,
  SDC_SEMAPHORE_ERROR,
  UDP_INVALID_RETURN_ADDRESS,
};


class friUdp{
 public:
  friUdp(int port=FRI_DEFAULT_SERVER_PORT, const char *remoteHost = NULL);

  ~friUdp();

 protected:
  /// Note: Remote host need not to be specified - if NULL, wait for the 
  /// incoming packages
  void Init(const char * remoteHost=NULL);
  void Close(void);
#ifdef WIN32
  int StartWinsock(void);
#endif
 public:
  int Send(tFriCmdData *data);
  int Recv(tFriMsrData *packet);
 private:
  int RecvPacket(int fd, tFriMsrData* p, struct timeval* ts, struct sockaddr_in* client);
  /// socket
  int udp_socket_;
  int serverPort;
  struct sockaddr_in krcAddr;
  /// if timestamp on receive is available, last received value can be inquired here
  double m_timestamp;
 public:
  /// This feature will be available only for systems, which support 
  /// SO_TIMESTAMP in the socket options, e.g. qnx
  double	getLastTimestamp() { return m_timestamp; }
};




inline std::ostream & operator<<(std::ostream &out , tFriHeader & head)
{
  out << "sendSeqCount " << head.sendSeqCount << "\n";
  out << "reflSeqCount " << head.reflSeqCount << "\n";
  out << "packetSize   " << head.packetSize   << "\n";
  out << "datagramId   " << std::hex  << head.datagramId << std::dec  ;
  switch (head.datagramId )
    {
    case FRI_DATAGRAM_ID_CMD:
      out << " FRI_DATAGRAM_ID_CMD \n" ;
      break;
    case FRI_DATAGRAM_ID_MSR:
      out << " FRI_DATAGRAM_ID_MSR \n" ;
      break;
    default:
      out <<" Unkown \n";
    }
  return out;
}
inline std::ostream & operator<<(std::ostream &out , tFriKrlData& krl)
{
  out << "krl_real ";
  for ( int i = 0; i < FRI_USER_SIZE; i++)
    out << " " << krl.realData[i];
  out << "\n";
  out << "krl_int ";
  for ( int i = 0; i < FRI_USER_SIZE; i++)
    out << " " << krl.intData[i];
  out << "\n";
  out << "krl_bool ";
  out << std::hex << krl.boolData << std::dec << "\n";
  return out;
}

inline std::ostream & operator<<(std::ostream &out , tFriIntfStatistics & stat)
{
  out << "answerRate  " << stat.answerRate << "\n";
  out << "latency     " << stat.latency << "\n";
  out << "jitter      " << stat.jitter << "\n";
  out << "missRate    " << stat.missRate << "\n";
  out << "missCounter " << stat.missCounter << "\n";
  return out;
}

inline std::ostream & operator<<(std::ostream &out , tFriIntfState & state)
{
  out << "timestamp " << state.timestamp<< "\n";
  out << "state     " << state.state<< "\n";
  out << "quality   " << state.quality << "\n";
  out << "desiredMsrSampleTime " << state.desiredMsrSampleTime << "\n";
  out << "desiredCmdSampleTiintfme " << state.desiredCmdSampleTime  << "\n";
  out << "safetyLimits " << state.safetyLimits << "\n";
  out << "statistics " << state.stat << "\n";
  return out;
}

inline std::ostream & operator<< (std::ostream & out, tFriRobotState & robot)
{
  out << "power   " << robot.power<< "\n";
  out << "control " << robot.control << "\n";
  out << "error   " << robot.error << "\n";
  out << "warning " << robot.warning << "\n";
  out << "temperature " ;
  for (int i = 0; i < LBR_MNJ; i++)
    out << robot.temperature[i] << " " ;
  out << "\n";
  return out;

}

#define WRITE_JNT_VEC(field)			\
  out << ""#field;				\
  for ( int i  = 0; i < LBR_MNJ; i++)		\
    out << " " << robot.field[i];		\
  out << "\n";
inline std::ostream & operator<<(std::ostream &out, tFriRobotData & robot)
{
  WRITE_JNT_VEC(msrJntPos);
  WRITE_JNT_VEC(cmdJntPos);
  WRITE_JNT_VEC(msrJntTrq);
  WRITE_JNT_VEC(estExtJntTrq);
  return out;

}

inline std::ostream & operator<<(std::ostream &out, tFriRobotCommand & robot)
{
  out << std::hex << robot.cmdFlags << std::dec << "\n";
  WRITE_JNT_VEC(jntPos);
  return out;

}


inline std::ostream & operator<<(std::ostream &out, tFriMsrData & msr)
{
  out << "head " << msr.head;
  out << "krl " << msr.krl;
  out << "intf " << msr.intf;
  out << "robot " << msr.robot;
  out << "data " << msr.data;
  return out;
}




inline std::ostream & operator<<(std::ostream &out, tFriCmdData & cmd)
{
  out << "head " << cmd.head;
  out << "krl " << cmd.krl;
  out << "cmd " << cmd.cmd;
  return out;
}




#endif
/* @} */
