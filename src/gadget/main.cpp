// Limelight gadget entry point.
//
// Equivalent of FUN_00102be0 in the original stripped binary: all main()
// does is forward to runGadgetSetup(), which mirrors the observed argv
// parsing, global.settings reading, script assembly, and system() call.

#include "usb_gadget.h"

int main(int argc, char** argv) {
    return limelight::runGadgetSetup(argc, argv);
}
