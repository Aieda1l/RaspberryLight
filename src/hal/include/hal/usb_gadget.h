#pragma once

#include <memory>
#include <string>

namespace limelight::hal {

struct GadgetConfig {
    // USB descriptors
    uint16_t id_vendor = 0;
    uint16_t id_product = 0;
    std::string serial_number;
    std::string manufacturer;
    std::string product;

    // Network function type: "ecm", "rndis", "ncm"
    std::string function_type = "ecm";

    // How many USB Ethernet interfaces to create
    int num_interfaces = 2;
};

class UsbGadget {
public:
    virtual ~UsbGadget() = default;

    virtual bool setup(const GadgetConfig& config) = 0;
    virtual void teardown() = 0;
    virtual bool isActive() const = 0;

    // Get the UDC (USB Device Controller) name for this platform
    virtual std::string getUdcName() const = 0;

    static std::unique_ptr<UsbGadget> create();
};

} // namespace limelight::hal
