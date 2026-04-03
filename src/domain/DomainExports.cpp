#include "DomainExport.h"

namespace domain {

// Force the shared library to emit at least one symbol on Windows.
DOMAIN_EXPORT void DomainLibraryAnchor() {}

} // namespace domain
