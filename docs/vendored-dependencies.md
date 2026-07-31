# Vendored dependencies

The project builds from the contents of `lib/`; it does not restore dependencies
from a package manager. This inventory records the version evidence and notice
material that are actually present in the repository. It is not a substitute for
checking the upstream distribution terms when a dependency is updated or a binary
package is released.

| Component | Checked-in version evidence | Notice evidence in this repository |
| --- | --- | --- |
| OpenVR headers and Windows import/runtime binaries | `openvr.h` and `openvr_driver.h` identify SteamVR 1.10.30 | The generated headers carry Valve copyright markers, but no standalone OpenVR license or notice file is tracked under `lib/openvr`. |
| Eigen | Macros identify 3.3.4 | Eigen headers contain Mozilla Public License 2.0 notices; no standalone Eigen license file is tracked. |
| Dear ImGui | Header identifies 1.62 | `lib/imgui/LICENSE.txt`; bundled stb headers also carry their own notices in the headers. |
| GLFW | Header identifies 3.2.1 | `lib/glfw/COPYING.txt`. |
| MinHook, including its HDE sources | No release number is encoded in the checked-in copy | `lib/MinHook/LICENSE`, which also includes the HDE notices. |
| gl3w and generated Khronos headers | No gl3w release number is encoded | The generated gl3w files contain a public-domain dedication; `glcorearb.h` and `khrplatform.h` contain Khronos permission notices. |
| picojson | No release number is encoded in the single header | `lib/picojson.h` contains its copyright and redistribution notice. |

## Maintenance and packaging rules

- Record the upstream version or immutable source revision whenever a vendored
  component changes. Do not rely on the date of the importing commit as a version.
- Preserve every checked-in license header and standalone notice verbatim.
- Before distributing binaries, verify the upstream terms for every component and
  add the appropriate standalone license or notice material. In particular, the
  current checkout does not contain a standalone notice for the OpenVR binaries or
  Eigen; treat that as a release checklist item rather than inferring terms.
- Include the repository `LICENSE`, `lib/MinHook/LICENSE`,
  `lib/imgui/LICENSE.txt`, and `lib/glfw/COPYING.txt` in the release notice bundle,
  together with any verified notices added for the remaining components.
- Compare the packaged DLLs and libraries with this inventory. A binary that is not
  explained here should block release until its source, version, and notice are
  recorded.
