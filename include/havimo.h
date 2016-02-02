/*
HaViMo.h

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

#ifndef HaViMo_h
#define HaViMo_h
#include <ros/ros.h>
#include "serial/serial.h"
#include <sstream>

typedef	struct{
	uint8_t father;
	uint8_t color;
	int16_t numPix;
	uint32_t sumX;
	uint32_t sumY;
	uint8_t maxX;
	uint8_t minX;
	uint8_t maxY;
	uint8_t minY;
} Cat;

typedef struct{
	uint8_t color:4;
	uint8_t count:4;
} Grid;


class HaViMo
{
public:

	ros::NodeHandle priv_nh_;
    std::string serial_port_;
    int baud_rate_;
    std::string result;
    size_t bytes_wrote;
    int debug;
    
    serial::Serial link;

	HaViMo();
	~HaViMo();

	void invokeRegion();
	void invokeGridding();
	bool getBiggestObj(int color, Cat* result); 
	bool getObject(int index, Cat* result);
	bool readAllRegions(Cat* results);
	bool waitForAck();
	bool readGrid(Grid* results);
	Grid readGridCell(int x, int y);
	Grid getGridCellFromBuffer(Grid* buffer, int x, int y);
};


#endif