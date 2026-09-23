;--------------------------------
;Include Modern UI

	!include "MUI2.nsh"

;--------------------------------
;General

	!define DRIVER_RESDIR "..\Driver\01questcalibrator"

	; Keep in sync with common\Version.h
	!define QUESTCAL_VERSION "1.1.0"
	!define QUESTCAL_PUBLISHER "VividNightmare"

	Name "QuestCalibrator"
	; Lands in the out directory so both release artifacts sit together
	OutFile "out\QuestCalibrator (Automatic Setup).exe"
	InstallDir "$PROGRAMFILES64\QuestCalibrator"
	InstallDirRegKey HKLM "Software\QuestCalibrator\Main" ""
	RequestExecutionLevel admin
	ShowInstDetails show

	VIProductVersion "${QUESTCAL_VERSION}.0"
	VIAddVersionKey "ProductName"     "QuestCalibrator"
	VIAddVersionKey "ProductVersion"  "${QUESTCAL_VERSION}"
	VIAddVersionKey "FileVersion"     "${QUESTCAL_VERSION}"
	VIAddVersionKey "FileDescription" "QuestCalibrator installer"
	VIAddVersionKey "CompanyName"     "${QUESTCAL_PUBLISHER}"
	VIAddVersionKey "LegalCopyright"  "Copyright (c) 2026 ${QUESTCAL_PUBLISHER}"

;--------------------------------
;Variables

VAR upgradeInstallation
VAR removeSpaceCal

;--------------------------------
;Interface Settings

	!define MUI_ABORTWARNING

;--------------------------------
;Pages

	!insertmacro MUI_PAGE_LICENSE "..\LICENSE"
	!define MUI_PAGE_CUSTOMFUNCTION_PRE dirPre
	!insertmacro MUI_PAGE_DIRECTORY
	!insertmacro MUI_PAGE_INSTFILES
  
	!insertmacro MUI_UNPAGE_CONFIRM
	!insertmacro MUI_UNPAGE_INSTFILES
  
;--------------------------------
;Languages
 
	!insertmacro MUI_LANGUAGE "English"

;--------------------------------
;Macros

;--------------------------------
;Functions

Function dirPre
	StrCmp $upgradeInstallation "true" 0 +2 
		Abort
FunctionEnd

Function .onInit
	StrCpy $upgradeInstallation "false"
	StrCpy $removeSpaceCal "false"

	; If SteamVR is already running, display a warning message and exit
	FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
	StrCmp $0 0 +3
		MessageBox MB_OK|MB_ICONEXCLAMATION \
			"SteamVR is still running. Cannot install this software.$\nPlease close SteamVR and try again."
		Abort

	; OpenVR-SpaceCalibrator's driver hooks the same vrserver internals; both
	; installed at once double-apply offsets, so it has to go before we install.
	ReadRegStr $R1 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "UninstallString"
	StrCmp $R1 "" nospacecal
	MessageBox MB_OKCANCEL|MB_ICONEXCLAMATION \
		"OpenVR-SpaceCalibrator is installed and conflicts with QuestCalibrator. \
		$\n$\nClick `OK` to uninstall it and continue, or `Cancel` to abort." \
		IDOK removespacecal
	Abort

	removespacecal:
		StrCpy $removeSpaceCal "true"
	nospacecal:

	ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator" "UninstallString"
	StrCmp $R0 "" done

	MessageBox MB_OKCANCEL|MB_ICONEXCLAMATION \
		"QuestCalibrator is already installed. $\n$\nClick `OK` to upgrade the \
		existing installation or `Cancel` to cancel this upgrade." \
		IDOK upgrade
	Abort

	upgrade:
		StrCpy $upgradeInstallation "true"
	done:
FunctionEnd

;--------------------------------
;Installer Sections

Section "Install" SecInstall
	
	StrCmp $upgradeInstallation "true" 0 noupgrade 
		DetailPrint "Uninstall previous version..."
		ExecWait '"$INSTDIR\Uninstall.exe" /S _?=$INSTDIR'
		Delete $INSTDIR\Uninstall.exe
		Goto afterupgrade
		
	noupgrade:

	afterupgrade:

	SetOutPath "$INSTDIR"

	File "..\LICENSE"
	File "..\THIRD-PARTY-NOTICES.txt"
	File "..\x64\Release\QuestCalibrator.exe"
	File "..\lib\openvr\lib\win64\openvr_api.dll"
	File "..\Overlay\manifest.vrmanifest"
	File "..\Overlay\icon.png"

	Var /GLOBAL vrRuntimePath
	; -noui suppresses the app's result dialog, which would block the installer
	nsExec::ExecToStack '"$INSTDIR\QuestCalibrator.exe" -openvrpath -noui'
	Pop $0
	Pop $vrRuntimePath
	DetailPrint "VR runtime path: $vrRuntimePath"

	StrCmp $removeSpaceCal "true" 0 nospacecal
		DetailPrint "Uninstalling OpenVR-SpaceCalibrator..."
		ReadRegStr $R2 HKLM "Software\OpenVR-SpaceCalibrator\Main" ""
		StrCmp $R2 "" 0 spacecaldirknown
			StrCpy $R2 "$PROGRAMFILES64\OpenVR-SpaceCalibrator"
		spacecaldirknown:
		IfFileExists "$R2\Uninstall.exe" 0 spacecalnouninst
			ExecWait '"$R2\Uninstall.exe" /S _?=$R2'
			Delete "$R2\Uninstall.exe"
			RMDir "$R2"
			Goto nospacecal
		spacecalnouninst:
			; Half-removed install: uninstaller is gone, clean up its traces directly
			DeleteRegKey HKLM "Software\OpenVR-SpaceCalibrator"
			DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator"
			Delete "$SMPROGRAMS\OpenVR-SpaceCalibrator.lnk"
	nospacecal:

	; The driver folder is the actual conflict; sweep it even when the
	; uninstaller ran (or was never registered), covering forks and manual installs.
	RMDir /r "$vrRuntimePath\drivers\01spacecalibrator"
	RMDir /r "$vrRuntimePath\drivers\000spacecalibrator"

	SetOutPath "$vrRuntimePath\drivers\01questcalibrator"
	File "${DRIVER_RESDIR}\driver.vrdrivermanifest"
	SetOutPath "$vrRuntimePath\drivers\01questcalibrator\resources"
	File "${DRIVER_RESDIR}\resources\driver.vrresources"
	SetOutPath "$vrRuntimePath\drivers\01questcalibrator\resources\settings"
	File "${DRIVER_RESDIR}\resources\settings\default.vrsettings"
	SetOutPath "$vrRuntimePath\drivers\01questcalibrator\bin\win64"
	File "..\x64\Release\driver_01questcalibrator.dll"
	
	WriteRegStr HKLM "Software\QuestCalibrator\Main" "" $INSTDIR
	WriteRegStr HKLM "Software\QuestCalibrator\Driver" "" $vrRuntimePath
  
	WriteUninstaller "$INSTDIR\Uninstall.exe"
	!define ARP "Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator"
	WriteRegStr HKLM "${ARP}" "DisplayName" "QuestCalibrator"
	WriteRegStr HKLM "${ARP}" "DisplayVersion" "${QUESTCAL_VERSION}"
	WriteRegStr HKLM "${ARP}" "Publisher" "${QUESTCAL_PUBLISHER}"
	WriteRegStr HKLM "${ARP}" "DisplayIcon" "$INSTDIR\QuestCalibrator.exe"
	WriteRegStr HKLM "${ARP}" "InstallLocation" "$INSTDIR"
	WriteRegStr HKLM "${ARP}" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""
	WriteRegStr HKLM "${ARP}" "QuietUninstallString" "$\"$INSTDIR\Uninstall.exe$\" /S"
	WriteRegDWORD HKLM "${ARP}" "NoModify" 1
	WriteRegDWORD HKLM "${ARP}" "NoRepair" 1

	CreateShortCut "$SMPROGRAMS\QuestCalibrator.lnk" "$INSTDIR\QuestCalibrator.exe"
	
	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\QuestCalibrator.exe" -installmanifest -noui'
	nsExec::ExecToLog '"$INSTDIR\QuestCalibrator.exe" -activatemultipledrivers -noui'

SectionEnd

;--------------------------------
;Uninstaller Section

Section "Uninstall"
	; If SteamVR is already running, display a warning message and exit
	FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
	StrCmp $0 0 +3
		MessageBox MB_OK|MB_ICONEXCLAMATION \
			"SteamVR is still running. Cannot uninstall this software.$\nPlease close SteamVR and try again."
		Abort
	
	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\QuestCalibrator.exe" -removemanifest -noui'

	Var /GLOBAL vrRuntimePath2
	ReadRegStr $vrRuntimePath2 HKLM "Software\QuestCalibrator\Driver" ""
	DetailPrint "VR runtime path: $vrRuntimePath2"
	Delete "$vrRuntimePath2\drivers\01questcalibrator\driver.vrdrivermanifest"
	Delete "$vrRuntimePath2\drivers\01questcalibrator\resources\driver.vrresources"
	Delete "$vrRuntimePath2\drivers\01questcalibrator\resources\settings\default.vrsettings"
	Delete "$vrRuntimePath2\drivers\01questcalibrator\bin\win64\driver_01questcalibrator.dll"
	Delete "$vrRuntimePath2\drivers\01questcalibrator\bin\win64\quest_calibrator_driver.log"
	RMdir "$vrRuntimePath2\drivers\01questcalibrator\resources\settings"
	RMdir "$vrRuntimePath2\drivers\01questcalibrator\resources\"
	RMdir "$vrRuntimePath2\drivers\01questcalibrator\bin\win64\"
	RMdir "$vrRuntimePath2\drivers\01questcalibrator\bin\"
	RMdir "$vrRuntimePath2\drivers\01questcalibrator\"

	Delete "$INSTDIR\LICENSE"
	Delete "$INSTDIR\THIRD-PARTY-NOTICES.txt"
	Delete "$INSTDIR\QuestCalibrator.exe"
	Delete "$INSTDIR\openvr_api.dll"
	Delete "$INSTDIR\manifest.vrmanifest"
	Delete "$INSTDIR\icon.png"
	
	DeleteRegKey HKLM "Software\QuestCalibrator\Main"
	DeleteRegKey HKLM "Software\QuestCalibrator\Driver"
	DeleteRegKey HKLM "Software\QuestCalibrator"
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator"

	Delete "$SMPROGRAMS\QuestCalibrator.lnk"
SectionEnd
