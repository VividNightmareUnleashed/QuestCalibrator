================================================================================
QuestCalibrator - Installation
================================================================================

IMPORTANT: Close Steam completely before installing (not just SteamVR).
Steam.exe itself locks the driver files. Check the system tray and exit Steam
fully.

QuestCalibrator is free to use under the end user license agreement in the
LICENSE file in this folder. Read it before installing: installing or using
QuestCalibrator, by either method below, means you accept it.

There are two ways to install: the script (easiest) or manually.


--------------------------------------------------------------------------------
OPTION 1: Script install (recommended)
--------------------------------------------------------------------------------

1. Extract this entire zip to a folder, e.g. Downloads\QuestCalibrator.
   Keep the folder structure intact - Install.ps1 reads the "app" and "driver"
   folders that sit next to it.

2. Unblock the extracted files (Windows blocks scripts downloaded from the
   internet):
     - Select ALL files in the extracted folder, right-click -> Properties,
       tick "Unblock" at the bottom, OK.
     - Or open PowerShell in the folder and run:
         Get-ChildItem -Recurse | Unblock-File

3. Right-click Install.ps1 -> "Run with PowerShell".

4. Click "Yes" when Windows asks for administrator permission.

5. Accept the license agreement: type R to read it, Y to agree, then Y again
   to approve its sections 2, 6 and 8 specifically. An update with unchanged
   terms doesn't ask again.

6. Answer the optional module question. The Lighthouse module adds base
   station tools, starting with a tab showing each base station and which
   lighthouse-tracked devices can see it; later versions add more. To add
   or remove it later, run Install.ps1 again.

7. Done. Start SteamVR - QuestCalibrator appears as a dashboard overlay and
   starts automatically with SteamVR.

To uninstall: run Uninstall.ps1 the same way, or find "QuestCalibrator" in
Windows Settings -> Apps.


--------------------------------------------------------------------------------
OPTION 2: Manual install (no scripts)
--------------------------------------------------------------------------------

You need to know where SteamVR is installed. The default is:

    C:\Program Files (x86)\Steam\steamapps\common\SteamVR

If Steam is on another drive, look for
    <drive>\SteamLibrary\steamapps\common\SteamVR
If you use a custom Steam library folder, SteamVR is under
    <that folder>\steamapps\common\SteamVR

Steps:

0. Read LICENSE in this folder. A manual install accepts it just as the
   script install does, including the separate approval of its sections 2
   (Restrictions), 6 (Termination) and 8 (Limitation of liability). If you
   don't accept it, stop here.

1. Close Steam completely (not just SteamVR).

2. Create a permanent home for the app, e.g.
       C:\Program Files\QuestCalibrator
   and copy the contents of this package's "app" folder into it
   (QuestCalibrator.exe, openvr_api.dll, manifest.vrmanifest, icon.png).
   Don't run it straight out of Downloads - the app registers its own path
   with SteamVR, so it needs a stable location.

3. Remove conflicting drivers, if present. Inside the SteamVR folder, delete
   these folders if they exist:
       drivers\01spacecalibrator
       drivers\000spacecalibrator
   (OpenVR-SpaceCalibrator conflicts with QuestCalibrator; both installed at
   once will double-apply offsets.)

4. Copy this package's "driver\01questcalibrator" folder into the SteamVR
   "drivers" folder, so you end up with:
       <SteamVR>\drivers\01questcalibrator\driver.vrdrivermanifest
       <SteamVR>\drivers\01questcalibrator\bin\win64\driver_01questcalibrator.dll
       <SteamVR>\drivers\01questcalibrator\resources\...

5. Register the overlay with SteamVR (required for auto-start and the
   dashboard listing). Open a Command Prompt in the folder from step 2 and
   run:
       QuestCalibrator.exe -installmanifest

   A message box reports the result: either the manifest path that was
   registered, or the reason it failed. QuestCalibrator is a windowed
   application, so the Command Prompt returns immediately and any text it
   prints there is unreliable - the dialog is the authoritative result.

   If you skip this step the driver still works, but you must launch
   QuestCalibrator.exe by hand every session.

6. Optional: enable the Lighthouse module. In an administrator Command
   Prompt, run:
       reg add HKLM\Software\QuestCalibrator\Modules /v Lighthouse /t REG_DWORD /d 1 /f

7. Start SteamVR.

Manual uninstall:

1. Close SteamVR.
2. In a Command Prompt in the app folder, run:
       QuestCalibrator.exe -removemanifest
   Do this BEFORE deleting anything: the command finds the registration to
   remove using the manifest file sitting next to the executable.
3. Delete <SteamVR>\drivers\01questcalibrator
4. Delete the app folder from step 2.
5. If you enabled the Lighthouse module, run (as administrator):
       reg delete HKLM\Software\QuestCalibrator /f


--------------------------------------------------------------------------------
Notes
--------------------------------------------------------------------------------

- Windows SmartScreen may warn about QuestCalibrator.exe because it isn't
  code-signed. Click "More info" -> "Run anyway". This is expected.
- QuestCalibrator requires SteamVR's "multiple drivers" support. The app turns
  that setting on every time it starts, so both install methods are covered -
  the script install simply does it up front. Uninstalling deliberately leaves
  the setting enabled, because other OpenVR tools depend on it too.
- If SteamVR has never been run on this PC, start it once and close it before
  installing. Both the installer and -installmanifest ask the OpenVR runtime
  where SteamVR lives, and that information doesn't exist until SteamVR has
  run at least once.
================================================================================
