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

ArClientBase client;

void drawingData(ArNetPacket *packet)
{
  int x, y;
  int numReadings;
  int i;

  numReadings = packet->bufToByte4();

  if (numReadings == 0)
  {
    printf("No readings for sensor %s\n", client.getName(packet));
  }
  printf("Readings for %s:", client.getName(packet));
  for (i = 0; i < numReadings; i++)
  {
    x = packet->bufToByte4();
    y = packet->bufToByte4();
    printf(" (%d %d)", x, y);
  }
  printf("\n\n");
}

ArGlobalFunctor1<ArNetPacket *> drawingDataCB(&drawingData);

void drawing(ArNetPacket *packet)
{
  int numDrawings;
  int i;
  char name[512];
  char shape[512];
  long primary, size, layer, secondary;
  unsigned long refresh;
  numDrawings = packet->bufToByte4();
  ArLog::log(ArLog::Normal, "There are %d drawings", numDrawings);
  for (i = 0; i < numDrawings; i++)
  {
    packet->bufToStr(name, sizeof(name));
    packet->bufToStr(shape, sizeof(shape));
    primary = packet->bufToByte4();
    size = packet->bufToByte4();
    layer = packet->bufToByte4();
    refresh = packet->bufToByte4();
    secondary = packet->bufToByte4();
    ArLog::log(ArLog::Normal, "name %-20s shape %-20s", name, shape);
    ArLog::log(ArLog::Normal, 
	       "\tprimary %08x size %2d layer %2d refresh %4u secondary %08x",
	       primary, size, layer, refresh, secondary);
    client.addHandler(name, &drawingDataCB);
    client.request(name, refresh);
  }
  
}

int main(int argc, char **argv)
{

  std::string hostname;
  ArGlobalFunctor1<ArNetPacket *> drawingCB(&drawing);
  Aria::init();
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);



  ArArgumentParser parser(&argc, argv);

  ArClientSimpleConnector clientConnector(&parser);
  
  parser.loadDefaultArguments();

  /* Check for -help, and unhandled arguments: */
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    exit(0);
  }

  /* Connect our client object to the remote server: */
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      printf("Server '%s' rejected connection, exiting\n", client.getHost());
    else
      printf("Could not connect to server '%s', exiting\n", client.getHost());
    exit(1);
  } 

  printf("Connected to server.\n");

  client.addHandler("listDrawings", &drawingCB);
  client.requestOnce("listDrawings");
  
  client.runAsync();
  while (client.getRunningWithLock())
  {
    ArUtil::sleep(1);
    //printf("%d ms since last data\n", client.getLastPacketReceived().mSecSince());
  }
  Aria::shutdown();
  return 0;

}
