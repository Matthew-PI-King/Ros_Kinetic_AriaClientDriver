/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "Aria.h"
#include "ArNetworking.h"

void test(ArNetPacket *packet)
{
  //char buf[packet->getDataLength()];
 
 printf("datalength recieved:%i\n",packet->getDataLength()); 
  int datLength=packet->bufToByte4();
  double bufnum[datLength];
  //packet->bufToStr(buf,packet->getDataLength());
  printf("\n");
  for(int i=0;i<datLength;i++){
    bufnum[i]=packet->bufToByte4();
    printf("%1.0f,", bufnum[i]);
  }
  printf("\n");
}

void tcpSetVel(double vel,ArClientBase *client){
  ArNetPacket request;
  request.empty();
  printf("Requesting set vel %lf\n", vel);
  request.byte2ToBuf((ArTypes::Byte2)(vel));
  client->requestOnce("SetVelRequest",&request);
}

void tcpSetRotVel(double vel,ArClientBase *client){
  ArNetPacket request;
  request.empty();
  printf("Requesting set vel %lf\n", vel);
  request.byte2ToBuf((ArTypes::Byte2)(vel));
  client->requestOnce("SetRotVelRequest",&request);
}

int main(int argc, char **argv)
{
  ArClientBase client;
  ArGlobalFunctor1<ArNetPacket *> testCB(&test);

  Aria::init();
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);
  ArTime startTime;
  startTime.setToNow();
  if (!client.blockingConnect("localhost", 7272))
  {
    printf("Could not connect to server, exiting\n");
    exit(1);
  }    
  printf("Took %ld msec to connect\n", startTime.mSecSince());
  
  client.runAsync();
  
  client.lock();
  client.logDataList(); //very useful command to see what we can request
  
  //tcpSetVel(10.0,&client);
  //tcpSetRotVel(10.0,&client);
  ArNetPacket request;
  request.empty();
  printf("Requesting step back %lf\n", 1000.0);
  request.byte2ToBuf((ArTypes::Byte2)(1000.0));
  client.requestOnce("MoveStepRequest",&request);



// client.requestOnce("SetVel");
  // client.addHandler("LaserRequest", &testCB);
   
  //client.requestOnce("LaserRequest");
  //client.logDataList();
  //client.request("LaserRequest", 100);
  //client.unlock();
  //ArUtil::sleep(5000);
  //client.lock();
  //client.disconnect();
  //client.unlock();
  ArUtil::sleep(50);
  exit(0);
}
