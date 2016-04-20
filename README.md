# OSVR FSM-9 Plugin
> Maintained at <https://github.com/OSVR/OSVR-FSM9>
>
> For details, see <http://osvr.github.io>
>
> For support, see <http://support.osvr.com>

This is a plugin for OSVR that provides access to Hillcrest Labs' FSM-9 tracker from OSVR applications.

It requires a patched build of VRPN with the following files: https://www.dropbox.com/s/66i9mmu8oop0c9l/vrpn_Freespace.zip?dl=0

Here is a build with those files: <a href="https://www.dropbox.com/s/wl8cjcvrdr5u4p9/vrpn.7z?dl=0">vrpn-freespace-patch</a>.

Also requires <a href="https://github.com/hcrest/libfreespace">libfreespace 0.7</a>.

In CMake gui, set CMAKE_PREFIX_PATH to the OSVR-Core Snapshot directory.
You may also need to manually set libfreespace and jsoncpp root directories.

## Licenses

This project: Licensed under the Apache License, Version 2.0.
