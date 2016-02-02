#include <havimo.h>

  
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "havimo_node");
  ros::NodeHandle nh;
  HaViMo module;
  ros::spinOnce();  
  while(ros::ok())
  {
    module.invokeRegion();
    ros::Duration(0.5).sleep();
    if (!module.waitForAck()) 
      ROS_INFO("ERROR BY PING");
    else {
      Cat biggest;
      // Find the biggest bounding box of the color num. 2
      if (module.getBiggestObj(2,&biggest))
      {
      // Print the size and position of the object  
        ROS_INFO("%d pixels in (%d,%d)",biggest.numPix,biggest.sumX / biggest.numPix,biggest.sumY / biggest.numPix);
      }else
      {
        ROS_INFO("NO OBJ FOUND\n");
      }  
    }
    ros::spinOnce();  
  }
  return 0;
}
