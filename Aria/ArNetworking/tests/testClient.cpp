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
  char buf[packet->getDataLength()];
  printf("datalength recieved:%i\n",packet->getDataLength()); 
  double bufnum[3];
  //packet->bufToStr(buf,packet->getDataLength());
  bufnum[0]=packet->bufToByte2();
  bufnum[1]=packet->bufToByte2();
  bufnum[2]=packet->bufToByte2();
  printf("command %f,%f,%f\n", bufnum[0], bufnum[1], bufnum[2]);
}

int main(int argc, char **argv)
{
  ArClientBase client;
  ArGlobalFunctor1<ArNetPacket *> testCB(&test);

  Aria::init();
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);
  ArTime startTime;
  startTime.setToNow();
  if (!client.blockingConnect("localhost", 7273))
  {
    printf("Could not connect to server, exiting\n");
    exit(1);
  }    
  printf("Took %ld msec to connect\n", startTime.mSecSince());
  
  client.runAsync();
  
  client.lock();
  client.addHandler("test", &testCB);
  client.addHandler("test2", &testCB);
  client.addHandler("test3", &testCB);
  client.logDataList();
  
  client.requestOnce("test");
  client.request("test2", 100);
  client.request("test3", -1);
  client.unlock();
  ArUtil::sleep(1000);
  printf("Changing speed\n");
  client.lock();
  client.request("test2", 300);
  client.unlock();
  ArUtil::sleep(1000);
  client.lock();
  client.requestStop("test2");
  client.unlock();
  
  ArUtil::sleep(1000);
  client.lock();
  client.disconnect();
  client.unlock();
  ArUtil::sleep(50);
  exit(0);
}
