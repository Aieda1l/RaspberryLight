// Limelight USB Gadget Configurator — 1:1 recreation
//
// The original stripped aarch64 binary at
// `usr/local/bin/gadget/gadget` is a thin C++ driver that assembles a shell
// script (embedded verbatim in the .rodata section) and hands it off to
// system(3). We preserve that exact architecture rather than reimplementing
// the configfs writes in C — the point is to be behaviorally indistinguishable
// from the original, including the "limelight_dnsmasq.service" hardcoded in
// the script template even under --systemcore mode (which is a bug in the
// original but part of its observable behavior).
//
// Script layout in the binary:
//
//   .rodata @ 0x10fe20: TEMPLATE_HEAD
//       "#!/bin/bash\n...lots of configfs setup...\nsudo ifconfig usb0 up "
//   .rodata @ 0x110a78: TEMPLATE_MID
//       ".1 netmask 255.255.255.0\nsudo ifconfig usb1 up "
//   .rodata @ 0x110aa8: TEMPLATE_TAIL
//       ".1 netmask 255.255.255.0\nsleep 5\nsudo systemctl restart
//        limelight_dnsmasq.service\n\nsystemctl start getty@ttyGS0.service\n"
//
// And the assembled command is:
//   TEMPLATE_HEAD + "<base1>.<id>" + TEMPLATE_MID + "<base2>.<id>" + TEMPLATE_TAIL
//
// where base1/base2 are "172.29"/"172.28" normally or "172.27"/"172.26" under
// --systemcore, and <id> is the global.settings `usb_id` clamped to [0, 15].

#include "usb_gadget.h"

#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

namespace limelight {

namespace {

// Verbatim copy of the template at .rodata:0x10fe20 (3159 bytes in the
// original). The script stops exactly at "sudo ifconfig usb0 up " — the IP
// string is appended by C++ at runtime.
constexpr const char kTemplateHead[] =
R"GADGET(#!/bin/bash

#echo -1 > /sys/module/usbcore/parameters/autosuspend
systemctl stop limelight_dnsmasq
configfs="/sys/kernel/config/usb_gadget"
this="${configfs}/libComposite"

serial="$(grep 'Serial' /proc/cpuinfo | head -n 1 | sed -E -e 's/^Serial\s+:\s+0*//')"
model="$(grep 'Model' /proc/cpuinfo | head -n 1 | sed -E -e 's/^Model\s+:\s+(.+)/\1/')"
manufacturer="Limelight"

# Generate MAC address base using the last 6 characters of the serial number
mac_base="$(echo "${serial}" | sed 's/^.*\(......\)$/\1/')"

ecm_mac_address_dev="00:1A:2B:${mac_base:0:2}:${mac_base:2:2}:${mac_base:4:2}"
ecm_mac_address_host="00:1A:2C:${mac_base:0:2}:${mac_base:2:2}:${mac_base:4:2}"
rndis_mac_address_dev="00:1A:2D:${mac_base:0:2}:${mac_base:2:2}:${mac_base:4:2}"
rndis_mac_address_host="00:1A:2E:${mac_base:0:2}:${mac_base:2:2}:${mac_base:4:2}"

echo "Serial: ${serial}"
echo "Model: ${model}"
echo "Manufacturer: ${manufacturer}"
echo "ECM MAC Address (Device): ${ecm_mac_address_dev}"
echo "ECM MAC Address (Host): ${ecm_mac_address_host}"
echo "RNDIS MAC Address (Device): ${rndis_mac_address_dev}"
echo "RNDIS MAC Address (Host): ${rndis_mac_address_host}"

libcomposite_loaded="$(lsmod | grep -e '^libcomposite' 2>/dev/null)"
[ -z "${libcomposite_loaded}" ] && modprobe libcomposite
while [ ! -d "${configfs}" ]
do
  sleep 0.1
done

mkdir -p "${this}"

echo "0x0200" > "${this}/bcdUSB"
echo "0x1d6b" > "${this}/idVendor"
echo "0x0104" > "${this}/idProduct"
echo "0x02" > "${this}/bDeviceClass"
echo "0x4000" > "${this}/bcdDevice"

mkdir -p "${this}/os_desc"
echo "1" > "${this}/os_desc/use"
echo "0xcd" > "${this}/os_desc/b_vendor_code"
echo "MSFT100" > "${this}/os_desc/qw_sign"

mkdir -p "${this}/strings/0x409"
echo "${manufacturer}" > "${this}/strings/0x409/manufacturer"
echo "${model}" > "${this}/strings/0x409/product"
echo "${serial}" > "${this}/strings/0x409/serialnumber"

for i in 1 2
do
  mkdir -p "${this}/configs/c.${i}/strings/0x409"
  echo "0xC0" > "${this}/configs/c.${i}/bmAttributes"
  echo "250" > "${this}/configs/c.${i}/MaxPower"
done

#breaks chromebooks
#mkdir -p "${this}/functions/acm.usb0"
#ln -s "${this}/functions/acm.usb0" "${this}/configs/c.1/"

mkdir -p "${this}/functions/ecm.usb0"
echo "${ecm_mac_address_host}" > "${this}/functions/ecm.usb0/host_addr"
echo "${ecm_mac_address_dev}" > "${this}/functions/ecm.usb0/dev_addr"
echo "CDC" > "${this}/configs/c.1/strings/0x409/configuration"
ln -s "${this}/functions/ecm.usb0" "${this}/configs/c.1/"

mkdir -p "${this}/functions/rndis.usb0"
mkdir -p "${this}/functions/rndis.usb0/os_desc/interface.rndis"
echo "RNDIS" > "${this}/functions/rndis.usb0/os_desc/interface.rndis/compatible_id"
echo "5162001" > "${this}/functions/rndis.usb0/os_desc/interface.rndis/sub_compatible_id"
echo "${rndis_mac_address_host}" > "${this}/functions/rndis.usb0/host_addr"
echo "${rndis_mac_address_dev}" > "${this}/functions/rndis.usb0/dev_addr"
echo "RNDIS" > "${this}/configs/c.2/strings/0x409/configuration"
ln -s "${this}/configs/c.2" "${this}/os_desc"
ln -s "${this}/functions/rndis.usb0" "${this}/configs/c.2/"

udevadm settle -t 5 || true
ls /sys/class/udc > "${this}/UDC"

sudo ifconfig usb0 up )GADGET";

// Verbatim copy of the literal at .rodata:0x110a78.
constexpr const char kTemplateMid[] =
    ".1 netmask 255.255.255.0\n"
    "sudo ifconfig usb1 up ";

// Verbatim copy of the literal at .rodata:0x110aa8. Note the literal embeds
// "limelight_dnsmasq.service" even though the C++ driver computes (and
// discards) a separate "systemcore_dnsmasq" / "limelight_dnsmasq" string
// based on --systemcore. The original binary has the same inconsistency.
constexpr const char kTemplateTail[] =
    ".1 netmask 255.255.255.0\n"
    "sleep 5\n"
    "sudo systemctl restart limelight_dnsmasq.service\n"
    "\n"
    "systemctl start getty@ttyGS0.service\n";

// Path the original opens via std::ifstream at 0x10fdd8.
constexpr const char kGlobalSettingsPath[] =
    "/usr/local/bin/visionserver/global.settings";

// Read the `usb_id` integer from global.settings. Returns the clamped id in
// [0, 15]. On any open or parse failure, prints the error to cerr (matching
// the "Error reading global settings: " catch handler in the original) and
// returns 0.
int readClampedUsbId() {
    std::ifstream ifs(kGlobalSettingsPath);
    if (!ifs.is_open()) {
        return 0;
    }
    try {
        nlohmann::json j;
        ifs >> j;
        if (j.is_object() && j.contains("usb_id")) {
            const auto& v = j["usb_id"];
            if (v.is_number_integer() || v.is_number_unsigned()) {
                long long raw = v.get<long long>();
                if (raw < 0) return 0;
                if (raw > 15) return 15;
                return static_cast<int>(raw);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading global settings: " << e.what() << std::endl;
    }
    return 0;
}

}  // namespace

int runGadgetSetup(int argc, char** argv) {
    // --- argv scan for --systemcore ---------------------------------------
    // Matches the std::string::compare loop at 0x00102cd0 in the original.
    // Any argv[i] equal to "--systemcore" (exact match) selects the
    // systemcore-mode IP bases. The original also constructs a
    // "systemcore_dnsmasq" / "limelight_dnsmasq" string here but never
    // references it again — we preserve that same harmless dead branch by
    // simply ignoring the service name.
    bool systemcore_mode = false;
    const char* ip_base1 = "172.29";
    const char* ip_base2 = "172.28";
    for (int i = 1; i < argc; ++i) {
        if (argv[i] != nullptr && std::strcmp(argv[i], "--systemcore") == 0) {
            systemcore_mode = true;
            ip_base1 = "172.27";
            ip_base2 = "172.26";
            break;
        }
    }

    // --- banner -----------------------------------------------------------
    std::cout << "Parsing... " << std::endl;
    if (systemcore_mode) {
        std::cout << "Running in SYSTEMCORE mode" << std::endl;
    }

    // --- read usb_id from global.settings ---------------------------------
    const int id = readClampedUsbId();
    std::cout << "ID " << id << std::endl;

    // --- assemble script --------------------------------------------------
    // The original uses std::string::append in sequence:
    //     s  = TEMPLATE_HEAD
    //     s += base1 + "." + to_string(id)
    //     s += TEMPLATE_MID
    //     s += base2 + "." + to_string(id)
    //     s += TEMPLATE_TAIL
    const std::string id_str = std::to_string(id);
    std::string ip1;
    ip1.reserve(std::strlen(ip_base1) + 1 + id_str.size());
    ip1.append(ip_base1).append(".").append(id_str);

    std::string ip2;
    ip2.reserve(std::strlen(ip_base2) + 1 + id_str.size());
    ip2.append(ip_base2).append(".").append(id_str);

    std::string script;
    script.reserve(sizeof(kTemplateHead) + ip1.size() + sizeof(kTemplateMid) +
                   ip2.size() + sizeof(kTemplateTail));
    script.append(kTemplateHead);
    script.append(ip1);
    script.append(kTemplateMid);
    script.append(ip2);
    script.append(kTemplateTail);

    // --- execute ----------------------------------------------------------
    std::cout << "Executing USB gadget configuration..." << std::endl;
    const int rc = ::system(script.c_str());
    if (rc == 0) {
        std::cout << "USB gadget configuration completed successfully." << std::endl;
    } else {
        std::cerr << "Error executing USB gadget configuration script. Return code: "
                  << rc << std::endl;
    }

    return 0;
}

}  // namespace limelight
