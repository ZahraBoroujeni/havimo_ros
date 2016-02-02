/*
HaViMo.cpp 

by Dr. Hamid Mobalegh, HaViSys UG (haftungsbeschraenkt)

This library is a free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://www.havisys.com
*/

#include <havimo.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


HaViMo::HaViMo():link("/dev/ttyUSB0",115200, serial::Timeout::simpleTimeout(10))
{
	result="";

  	priv_nh_.param<std::string>("havimo_serial_port", serial_port_, "/dev/ttyUSB0");
  	priv_nh_.param("havimo_baud_rate", baud_rate_,115200);
  	priv_nh_.param("havimo_debug",debug,1);
  	link.close();
  	link.setPort(serial_port_);
  	link.setBaudrate(baud_rate_);
  	link.open();
  	try
    {
      ROS_INFO("Is the serial port open?");
      //cout << my_serial.getBaudrate() << endl;
	  if(link.isOpen())
	  	ROS_INFO(" Yes.");
	  else
	  	ROS_INFO(" No.");
    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("could not find serial port");
    }
}


HaViMo::~HaViMo(){
}


void HaViMo::invokeRegion(){
	//send a cap_grid ccommand for the next run
	uint8_t s[] = "\xFF\xFE\x0E\x00\x00\x0E";// 15
	link.write(s,6);
}
void HaViMo::invokeGridding(){
	//send a cap_grid ccommand for the next run
	uint8_t s[] = "\xFF\xFE\x15\x00\x00\x15";// 15
	link.write(s,6);
}

bool HaViMo::getObject(int index, Cat* result){
	uint8_t s[] = "\xFF\xFE\x02\x00\x10\x12";
	s[3]=index*16;
	s[5]=((index*16) ^ (0x12)) & 127;
	link.write(s,6);
	link.setTimeout(std::numeric_limits<uint32_t>::max(),100, 0, 100, 0);
    if(link.read((uint8_t*)result,16)!=16) {
//		if (debug) ROS_INFO("ERROR READING");
		return false; //todo
	}
	if (debug){//debug){
		ROS_INFO("%d => col:%d, pix:%d, minx:%d, miny:%d",result->father,result->color,result->numPix,result->minX,result->minY);
	}
	return true;
}

bool HaViMo::getBiggestObj(int color, Cat* result){
	Cat tmp;
	uint16_t maxCnt=0;
	int maxIndex=0;
	for (int i = 1; i < 16; i++) {
		if (!getObject(i, &tmp)) continue;
		if  (tmp.father > 0 && tmp.color == color && tmp.numPix > maxCnt){
			maxCnt=tmp.numPix;
			maxIndex = i;
			*result = tmp;
		}
	}
	if (maxIndex != 0) return true;
	return false;
} 

bool HaViMo::waitForAck(){
  int tries=5;
  uint8_t tmp[] ="";
  do{
	ROS_INFO("P");
    uint8_t s[] = "\xFF\xFE\x01\x00\x00\x01";
    link.write(s,6);
    link.setTimeout(std::numeric_limits<uint32_t>::max(),10, 0, 10, 0);
    //link.setTimeout(10);
  }while (link.read((uint8_t*)tmp,2)!=2 && tries-- > 0);
  
  if (debug==1 && tries == 0) ROS_INFO("ERROR IN PING");

  return (tries > 0);
}

bool HaViMo::readAllRegions(Cat* results){
	for (int i = 1; i < 16; i++) {
		if (!getObject(i, &results[i-1])) return false;
	}
	return true;
}

bool HaViMo::readGrid(Grid* results){
	for (int i=0; i<24; i++){
		uint8_t s[] = "\xFF\xFE\x16\x00\x20\x36";
		s[3]=i*2;
		s[5]=(i*2) ^ (0x36);
		link.write(s,7);
		link.setTimeout(std::numeric_limits<uint32_t>::max(),10, 0, 10, 0);
		//link.setTimeout(10);
		if(link.read((uint8_t*)(results+i*32),32)!=32) return false;
	}
	return true;
}

Grid HaViMo::readGridCell(int x, int y){
	int i = y;
	uint8_t s[32] = "\xFF\xFE\x16\x00\x20\x36";
	s[3]=i*2;
	s[5]=(i*2) ^ (0x36);
	link.write(s,7);
	link.setTimeout(std::numeric_limits<uint32_t>::max(),10, 0, 10, 0);
	//link.setTimeout(10);
	Grid errcode = {15,15};
	if(link.read((s),32)!=32) return errcode;
	return ((Grid*)s)[x];
}

	
Grid HaViMo::getGridCellFromBuffer(Grid* buffer, int x, int y){
	return buffer[x+y*32];
}


