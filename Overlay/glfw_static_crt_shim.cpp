// The vendored lib-vc2015 glfw3.lib was built against the DLL CRT (/MD), so its
// objects reach strdup through the import slot __imp__strdup. This binary links
// the static CRT (/MT, no msvcrt.lib); alias that slot to the static _strdup.
// Delete this file if GLFW is ever re-vendored as an /MT build or as source.
extern "C" char *_strdup(const char *);
extern "C" char *(*__imp__strdup)(const char *) = _strdup;
