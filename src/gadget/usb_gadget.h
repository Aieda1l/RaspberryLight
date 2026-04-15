#pragma once

// Limelight USB Gadget Configurator
//
// Reverse-engineered from the stripped aarch64 ELF
// `usr/local/bin/gadget/gadget` in the Limelight 4 firmware (~86 KB). The
// original binary does almost all of its real work by assembling an embedded
// /bin/bash script and invoking it via system(3). We replicate that exactly —
// the embedded script lives in usb_gadget.cpp as a raw string literal, and
// this header exposes the small driver that mirrors main()'s behavior.
//
// Observed responsibilities (from FUN_00102be0 decompilation):
//   1. Parse argv for `--systemcore`. If present, switch the usb0/usb1 IP
//      bases from 172.29/172.28 to 172.27/172.26. (The dnsmasq service name
//      string is constructed but never actually used — a dead branch in the
//      original — so we preserve that 1:1 behavior.)
//   2. Print "Parsing... " + newline, and "Running in SYSTEMCORE mode" + newline
//      when --systemcore is set.
//   3. Open /usr/local/bin/visionserver/global.settings as an ifstream, parse
//      the JSON, and read the top-level `usb_id` integer. Clamp it to [0, 15].
//      On open/parse failure, id defaults to 0.
//   4. Print "ID <n>" + newline.
//   5. Assemble the shell script template with the IP prefix, middle, and
//      trailing chunks, interleaving "<base1>.<id>" and "<base2>.<id>".
//   6. Print "Executing USB gadget configuration..." + newline.
//   7. Invoke ::system() on the assembled script.
//   8. On return == 0: print "USB gadget configuration completed successfully."
//      to cout. Otherwise: print "Error executing USB gadget configuration
//      script. Return code: <rc>" to cerr.

namespace limelight {

// Entry point implementation — the corresponding main() in main.cpp just
// forwards argv/argc to this. Returns the value main() should return (0).
int runGadgetSetup(int argc, char** argv);

}  // namespace limelight
