#pragma once

namespace scand3d::bridge {

/// Runs once when the shared library is loaded, so logcat shows the native core is alive
/// before any capture or reconstruction starts.
void onLibraryLoad();

}  // namespace scand3d::bridge
