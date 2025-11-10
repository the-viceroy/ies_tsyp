#include "modules.h"

// Default implementation of poll - does nothing
// Derived classes should override this to implement their specific behavior
void Module::poll(Controller &controller) {
    // Default: no action
}

// Default implementation of condition - always returns true
// Derived classes can override this to add conditions for when poll() should run
bool Module::condition(Controller &controller) {
    return true;
}
