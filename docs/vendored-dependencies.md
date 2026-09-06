# Vendored dependencies

The project builds from the contents of `lib/`; it does not restore dependencies
from a package manager. This inventory records the version evidence and notice
material that are actually present in the repository. It is not a substitute for
checking the upstream distribution terms when a dependency is updated or a binary
package is released.

| Component | Checked-in version evidence | Notice evidence in this repository |
| --- | --- | --- |
| OpenVR headers and Windows import/runtime binaries | SDK 2.15.6 | `lib/openvr/LICENSE`; Valve copyright markers remain in the headers. |
| Eigen | 5.0.1 (`Eigen/Version`) | `lib/Eigen/COPYING.*`, including MPL 2.0 and third-party notices. |
| Dear ImGui | 1.92.9 | `lib/imgui/LICENSE.txt`; bundled `imstb_*` headers retain their notices. |
| GLFW | 3.5.1, built from source | `lib/glfw/COPYING.txt` (upstream `LICENSE.md`). |
| MinHook, including its HDE sources | No release number is encoded in the checked-in copy | `lib/MinHook/LICENSE`, which also includes the HDE notices. |
| gl3w and generated Khronos headers | No gl3w release number is encoded | The generated gl3w files contain a public-domain dedication; `glcorearb.h` and `khrplatform.h` contain Khronos permission notices. |
| picojson | No release number is encoded in the single header | `lib/picojson.h` contains its copyright and redistribution notice. |

## Pinned upstream sources

The four updated distributions were imported from these release commits. Source
files are unmodified except the explicitly namespaced OpenVR compatibility
extract described below.

| Component | Release | Commit |
| --- | --- | --- |
| OpenVR | [v2.15.6](https://github.com/ValveSoftware/openvr/tree/v2.15.6) | `0924064316de3effbcd1acf1e309182a2deb1c05` |
| Eigen | [5.0.1](https://gitlab.com/libeigen/eigen/-/tree/5.0.1) | `bc3b39870ecb690a623a3f49149a358b95c5781d` |
| Dear ImGui | [v1.92.9](https://github.com/ocornut/imgui/tree/v1.92.9) | `01380c579715e62fb9a8d6ec0502c4ea83bfde6e` |
| GLFW | [3.5.1](https://github.com/glfw/glfw/tree/3.5.1) | `d9d6f0f1f967807ffade6598ea9a631ebaf37a56` |

- OpenVR: copy `headers/openvr.h`, `headers/openvr_driver.h`, Windows import
  libraries, and `bin/win64/openvr_api.dll`. The runtime DLL resides under
  `lib/openvr/lib/win64/` for the existing build/package paths. Headers and
  binaries must come from the same release. The retained
  `compat/ivrserverdriverhost_005.h` contains Valve's pose and host declarations
  from the previous 1.10.30 header (repository commit `c607fa3`) in a separate
  namespace. It supplies test evidence for older device drivers, not a second SDK.
- Eigen: copy the complete upstream `Eigen/` subtree and root `COPYING.*`
  notices. Remove obsolete files from the previous subtree.
- ImGui: copy the root C++ sources/headers and the GLFW/OpenGL3 backend files,
  including `imgui_impl_opengl3_loader.h`. VR text and input ownership live in
  `Overlay/ImGuiVRInput.h`; do not patch ImGui or its backends. Application GL
  calls still use the existing gl3w loader.
- GLFW: copy `include/`, `src/` and the license. The overlay project compiles
  the common, null and Win32 sources listed by upstream `src/CMakeLists.txt`,
  with `_GLFW_WIN32`. They inherit the application's C runtime selection in
  Debug and Release; no prebuilt GLFW library or C-runtime shim is needed.

OpenVR x64 binary SHA-256 values:

| File | SHA-256 |
| --- | --- |
| `openvr_api.dll` | `bab8ac6ef64e68a9ca53315b0014d131088584b2efdfa6db511d67ec03cfcb4a` |
| `openvr_api.lib` | `a0bf57c5920f569e8d21ab3e5bc95bac4b73e2016217f8b5b93495a2a7197bbb` |

## Upgrade validation

Run `tools/validate-cpp.ps1 -Mode Build` for the Release build and regression
suite. `Tests/OpenVRLayoutTests.cpp` checks every pose member's type and offset
against 1.10.30 and dispatches through the actual MSVC x64 vtables for host
interfaces `_005`, `_006`, and the driver context. This checks the layout used
by the hooks; it cannot prove that all live SteamVR device drivers are hooked.

`Tests/ImGuiInputTests.cpp` covers VR text replacement, UTF-8 capacity boundaries,
empty and numeric input, and desktop/VR focus transfer. Desktop verification
should also exercise the device rename controls, keyboard navigation, settings
scrolling, guide animation, and result/error modals. Preview flags run without
SteamVR and skip persistence.

Before shipping, also test the installed package with current SteamVR and a
headset: both tracking systems, pose forwarding and calibration, dashboard
mouse/scroll/keyboard, minimize/restore, and driver restart. Building with this
SDK requests newer runtime interfaces; SteamVR 1.10.30 is not a supported runtime
baseline for this upgrade.

## Maintenance and packaging rules

- Record the upstream version or immutable source revision whenever a vendored
  component changes. Do not rely on the date of the importing commit as a version.
- Preserve every checked-in license header and standalone notice verbatim.
- Before distributing binaries, verify the upstream terms for every component and
  include the corresponding standalone license and notice material.
- Include the repository `LICENSE`, `lib/MinHook/LICENSE`,
  `lib/openvr/LICENSE`, `lib/Eigen/COPYING.*`, `lib/imgui/LICENSE.txt`, and
  `lib/glfw/COPYING.txt` in the release notice bundle,
  together with any verified notices added for the remaining components.
- Compare the packaged DLLs and libraries with this inventory. A binary that is not
  explained here should block release until its source, version, and notice are
  recorded.
