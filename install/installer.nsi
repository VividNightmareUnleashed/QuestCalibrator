; Legacy executable distribution: package the same guarded script installer.
; Installation, upgrades and removal have one implementation in Install.ps1.
!include "MUI2.nsh"
!include "x64.nsh"
!define DRIVER_RESDIR "..\Driver\01questcalibrator"
!define QUESTCAL_VERSION "1.1.0"
!define QUESTCAL_PUBLISHER "VividNightmare"
Name "QuestCalibrator"
OutFile "out\QuestCalibrator (Automatic Setup).exe"
RequestExecutionLevel admin
ShowInstDetails show
VIProductVersion "${QUESTCAL_VERSION}.0"
VIAddVersionKey "ProductName" "QuestCalibrator"
VIAddVersionKey "ProductVersion" "${QUESTCAL_VERSION}"
VIAddVersionKey "FileVersion" "${QUESTCAL_VERSION}"
VIAddVersionKey "FileDescription" "QuestCalibrator installer"
VIAddVersionKey "CompanyName" "${QUESTCAL_PUBLISHER}"
VIAddVersionKey "LegalCopyright" "Copyright (c) 2026 ${QUESTCAL_PUBLISHER}"
!define MUI_ABORTWARNING
!insertmacro MUI_PAGE_LICENSE "..\LICENSE"
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"
Function .onInit
    ${IfNot} ${RunningX64}
        MessageBox MB_OK|MB_ICONSTOP "QuestCalibrator requires 64-bit Windows."
        SetErrorLevel 1
        Abort
    ${EndIf}
FunctionEnd
Section "Install"
    InitPluginsDir
    SetOutPath "$PLUGINSDIR\package"
    File "Install.ps1"
    File "Uninstall.ps1"
    File "FilesystemPolicy.ps1"
    File "README-INSTALL.txt"
    File "..\LICENSE"
    File "..\THIRD-PARTY-NOTICES.txt"
    SetOutPath "$PLUGINSDIR\package\app"
    File "..\x64\Release\QuestCalibrator.exe"
    File "..\lib\openvr\lib\win64\openvr_api.dll"
    File "..\Overlay\manifest.vrmanifest"
    File "..\Overlay\icon.png"
    SetOutPath "$PLUGINSDIR\package\driver\01questcalibrator"
    File /r "${DRIVER_RESDIR}\*"
    SetOutPath "$PLUGINSDIR\package\driver\01questcalibrator\bin\win64"
    File "..\x64\Release\driver_01questcalibrator.dll"
    ; Sysnative selects the 64-bit shell from this 32-bit NSIS process. The
    ; script declines conflicts in unattended mode and preserves failed work
    ; for retry; no independent NSIS deletion or uninstaller is generated.
    DetailPrint "Close Steam completely before installing. Conflicting software must be removed first."
    nsExec::ExecToLog '"$WINDIR\Sysnative\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -ExecutionPolicy Bypass -File "$PLUGINSDIR\package\Install.ps1" -Unattended -UserLocalAppData "$LOCALAPPDATA"'
    Pop $0
    StrCmp $0 "0" done
        DetailPrint "Installation failed ($0). See the script output above; repair the reported prerequisite and retry."
        SetErrorLevel 1
        Abort
    done:
SectionEnd
