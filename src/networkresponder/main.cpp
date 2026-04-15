// Limelight networkresponder entry point.
//
// Equivalent of `main` (FUN_00101dc0) in the original binary:
//   1. Construct NetworkResponder on the stack.
//   2. Call init() — on failure, log "Failed to initialize server" and exit(1).
//   3. Otherwise enter the blocking run() loop until signaled.

#include "responder.h"

#include <csignal>
#include <cstdlib>
#include <iostream>

namespace {

void handleSignal(int /*sig*/) {
    limelight::NetworkResponder::requestStop();
}

}  // namespace

int main(int /*argc*/, char** /*argv*/) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    // Ignore SIGPIPE so a broken sendto() doesn't tear down the daemon.
    std::signal(SIGPIPE, SIG_IGN);

    limelight::NetworkResponder responder;

    if (!responder.init()) {
        std::cerr << "Failed to initialize server" << std::endl;
        return 1;
    }

    responder.run();
    return 0;
}
