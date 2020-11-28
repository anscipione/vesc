#ifndef VESC_DRIVER_VESC_DEVICE_NAMER_H_
#define VESC_DRIVER_VESC_DEVICE_NAMER_H_


#include <string>
#include <boost/optional.hpp>

#include "vesc_driver/datatypes.h"
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

  class VescDeviceLookup
  {
    public:

      VescDeviceLookup(std::string device);
      
      const char* deviceUUID() const;  
      const char* version() const;
      const char* hwname() const;
      
      bool        isReady();  
    private:
      std::string device_;
      std::string uuid_;
      std::string version_;
      std::string hwname_;
      std::string error_;
      bool ready_;

    private:
      // interface to the VESC
      VescInterface vesc_;
      void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
      void vescErrorCallback(const std::string& error);      
  };
} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DEVICE_NAMER_H_