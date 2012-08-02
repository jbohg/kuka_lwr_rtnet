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
FRI_SecondApp.cpp



[FH]}}
*/ 
/**
\defgroup friSecondApp
*/
/* @{ */
/**
\author (Guenter Schreiber)
\file FRI_SecondApp.cpp
\brief The second -more versatile- sample of FRI Usage

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
friSecondApp
#else
main
#endif
#endif
(int argc, char *argv[])
{

	cout << "Opening FRI Interface for Second Sample" << endl;
	{
		friRemote friInst;
		FRI_QUALITY lastQuality = FRI_QUALITY_BAD;
		FRI_CTRL lastCtrlScheme = FRI_CTRL_OTHER;

		double timeCounter=0;
		// do one handshake before the endless loop
		friInst.doDataExchange();
		/** enter main loop - wait until we enter stable command mode */
		for(;;)
		{
			// do some handshake to KRL
			// send to krl int a value
			friInst.setToKRLInt(0,1);
			if ( friInst.getQuality() >= FRI_QUALITY_OK)
			{
				// send a second marker
				friInst.setToKRLInt(0,10);
			}

			//
			// just mirror the real value..
			//
			friInst.setToKRLReal(0,friInst.getFrmKRLReal(1));

			//
			if ( lastCtrlScheme != friInst.getCurrentControlScheme())
			{
				cout << "switching control scheme " << lastCtrlScheme;
				lastCtrlScheme = friInst.getCurrentControlScheme();
				cout << " to " << lastCtrlScheme;
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
                            cout << "preparing sine command" << endl;
							timeCounter+=friInst.getSampleTime();
							for (int i = 0; i < LBR_MNJ; i++)
							{
								// perform some sort of sine wave motion

								newJntVals[i]+=(float)sin( timeCounter * M_PI * 0.02) * (float)(10./180.*M_PI);
							}
						}
						else
						{
							timeCounter=0.;
						}
					}
					else
					{
                        cout << "NOT in CMD mode" << endl;
						timeCounter=0.;
					}
					// Call to data exchange - and the like 
					friInst.doPositionControl(newJntVals);
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
                            cout << "preparing sine command" << endl;
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
									newCartVals[(i*4)-1]+=(float)sin( timeCounter * M_PI * 0.02) * (0.1f);
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
                        cout << "NOT in CMD mode" << endl;
						timeCounter=0.;
					}
					// Call to data exchange - and the like 
					friInst.doCartesianImpedanceControl(newCartVals,NULL,NULL,newForceTorqueAdd);
				}
				break;
			default:
				/* do nothing - just data exchange for waiting */
				friInst.doDataExchange();
			}
			// have some debug information every n.th. step
			int divider = (int)( (1./friInst.getSampleTime()) *5.0);

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
			/// Quality change leads to output of statistics
			/// for informational reasons
			/// \sa FRI_QUALITY
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
