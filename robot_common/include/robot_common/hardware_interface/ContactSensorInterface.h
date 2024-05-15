// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_CONTACT_SENSOR_INTERFACE_H_
#define _LIMX_CONTACT_SENSOR_INTERFACE_H_

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace robot_common {

  // Class representing a handle for a contact sensor.
  class ContactSensorHandle {
  public:
    // Default constructor.
    ContactSensorHandle() = default;

    // Constructor with parameters.
    ContactSensorHandle(const std::string& name, const bool* is_contact)
      : name_(name), is_contact_(is_contact) {
      // Check if the pointer is null.
      if (is_contact == nullptr) {
        throw hardware_interface::HardwareInterfaceException(
          "Cannot create handle '" + name + "'. is_contact pointer is null.");
      }
    }

    // Getter function for the name of the contact sensor.
    std::string getName() const {
      return name_;
    }

    // Function to check if the contact sensor is in contact.
    bool isContact() const {
      // Ensure that the pointer is not null and return its value.
      assert(is_contact_);
      return *is_contact_;
    }

  private:
    std::string name_;            // Name of the contact sensor.
    const bool* is_contact_{nullptr};  // Pointer to a boolean indicating if the sensor is in contact.
  };

  // Interface for managing contact sensors.
  class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle,
                                                          hardware_interface::DontClaimResources> {
  };

}  // namespace robot_common

#endif // _LIMX_CONTACT_SENSOR_INTERFACE_H_