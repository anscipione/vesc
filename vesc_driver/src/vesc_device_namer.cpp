#include "vesc_driver/vesc_device_uuid_lookup.h"

#include <stdlib.h> 
#include <iostream>

int main(int argc, char** argv)
{
   std::string devicePort=   argv[1];
   std::string VESC_UUID_ENV="VESC_UUID_ENV=";

   vesc_driver::VescDeviceLookup lookup(devicePort);

   for(int i=0;i<50;i++){
      usleep(20);
      if (lookup.isReady()) break;

    }
    
    if(lookup.isReady())
   {
      VESC_UUID_ENV += lookup.deviceUUID(); 
      std::cout << VESC_UUID_ENV <<std::endl;
      putenv( const_cast<char*>(VESC_UUID_ENV.c_str()) );
      return 0;
   } else return -1;
}