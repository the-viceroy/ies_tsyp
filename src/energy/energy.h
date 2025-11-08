#pragma once

/**
 * Energy Management Compatibility Header
 * 
 * This file provides backward compatibility for code that includes
 * "energy/energy.h" when the actual implementation is in "manager.h"
 * 
 * Purpose: Maintains clean separation between interface naming and
 * implementation details without breaking existing includes
 * 
 * Simply redirects to the actual Manager implementation
 */
#include "manager.h"
