;  This file is part of OpenHRI.
;  Copyright (C) 2010  AIST-OpenHRI Project
;

!addplugindir "."

;--------------------------------
;Include Modern UI

!include "MUI.nsh"

;--------------------------------
;General

!define PACKAGE_NAME "OpenHRIWorlds"
!define PACKAGE_VERSION "1.00"
!define OUTFILE "${PACKAGE_NAME}-${PACKAGE_VERSION}-installer.exe"
!define TOP_SRCDIR "..\.."
!define TOP_BUILDDIR "..\.."
!define INSTDIR_REG_ROOT "HKLM"
!define INSTDIR_REG_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PACKAGE_NAME}"
!define SCDIR "$SMPROGRAMS\OpenHRI\worlds"

;Name and file
Name "${PACKAGE_NAME} ${PACKAGE_VERSION}"
OutFile "${OUTFILE}"
ShowInstDetails show
ShowUninstDetails show
InstallDir "$PROGRAMFILES\${PACKAGE_NAME}"
InstallDirRegKey ${INSTDIR_REG_ROOT} ${INSTDIR_REG_KEY} "InstallDir"

!include "AdvUninstLog.nsh"
!insertmacro UNATTENDED_UNINSTALL
;!insertmacro INTERACTIVE_UNINSTALL

;--------------------------------
;Interface Settings

;  !define MUI_ICON "${TOP_SRCDIR}\icons\openhriworlds.ico"
;  !define MUI_UNICON "${TOP_SRCDIR}\icons\openhriworlds.uninstall.ico"

;--------------------------------
;Pages

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_LICENSE $(MUILicense)
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_WELCOME
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH

;--------------------------------
;Languages

!insertmacro MUI_LANGUAGE "English"
!insertmacro MUI_LANGUAGE "Japanese"

;--------------------------------
;License Language String

LicenseLangString MUILicense ${LANG_ENGLISH} "${TOP_SRCDIR}\COPYING"
LicenseLangString MUILicense ${LANG_JAPANESE} "${TOP_SRCDIR}\COPYING"

;--------------------------------
;Reserve Files

;These files should be inserted before other files in the data block
;Keep these lines before any File command
;Only for solid compression (by default, solid compression is enabled for BZIP2 and LZMA)

!insertmacro MUI_RESERVEFILE_LANGDLL


;--------------------------------
;Installer Sections

Section $(TEXT_SecBase) SecBase

  SetOutPath "$INSTDIR"

  !insertmacro UNINSTALL.LOG_OPEN_INSTALL

  ; Main executables
  File "${TOP_BUILDDIR}\dist\BlocksWorld.exe"
  File "rtc.conf"

  ; Model data
  File /r "${TOP_BUILDDIR}\openhriworlds\data"

  ; Required Libralies
  File /r "${TOP_BUILDDIR}\dist\*.pyd"
  File /r "${TOP_BUILDDIR}\dist\*.dll"
  File "${TOP_BUILDDIR}\dist\library.zip"

  ; Information/documentation files
;  File "/oname=ChangeLog.txt" "${TOP_SRCDIR}\ChangeLog"
  File "/oname=Authors.txt" "${TOP_SRCDIR}\AUTHORS"
  File "/oname=License.txt" "${TOP_SRCDIR}\COPYING"

  !insertmacro UNINSTALL.LOG_CLOSE_INSTALL

  ;Store installation folder
  WriteRegStr HKLM "Software\${PACKAGE_NAME}" "" $INSTDIR

  ; Write the Windows-uninstall keys
  WriteRegStr ${INSTDIR_REG_ROOT} "${INSTDIR_REG_KEY}" "DisplayName" "${PACKAGE_NAME}"
  WriteRegStr ${INSTDIR_REG_ROOT} "${INSTDIR_REG_KEY}" "DisplayVersion" "${PACKAGE_VERSION}"
  WriteRegStr ${INSTDIR_REG_ROOT} "${INSTDIR_REG_KEY}" "Publisher" "AIST-OpenHRI Project"
  WriteRegStr ${INSTDIR_REG_ROOT} "${INSTDIR_REG_KEY}" "InstallDir" "$INSTDIR"
  WriteRegStr ${INSTDIR_REG_ROOT} "${INSTDIR_REG_KEY}" "UninstallString" "$INSTDIR\uninstall.exe"
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PACKAGE_NAME}" "NoModify" 1
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PACKAGE_NAME}" "NoRepair" 1

  ;Create uninstaller
  WriteUninstaller "$INSTDIR\uninstall.exe"

  ;Create shortcuts
  CreateDirectory "${SCDIR}"
  CreateShortCut "${SCDIR}\Uninstall Virtual Worlds.lnk" "$INSTDIR\uninstall.exe"
  CreateShortCut "${SCDIR}\BlocksWorld.lnk" "$INSTDIR\BlocksWorld.exe"

SectionEnd

;--------------------------------
;Installer Functions

Function .onInit
  !insertmacro MUI_LANGDLL_DISPLAY
  !insertmacro UNINSTALL.LOG_PREPARE_INSTALL
FunctionEnd

Function .onInstSuccess
  !insertmacro UNINSTALL.LOG_UPDATE_INSTALL
FunctionEnd

;--------------------------------
;Descriptions

  ;Language strings
  LangString TEXT_SecBase ${LANG_ENGLISH} "Standard installation."
  LangString DESC_SecBase ${LANG_ENGLISH} "Standard installation."
 
  LangString TEXT_SecBase ${LANG_JAPANESE} "Standard installation"
  LangString DESC_SecBase ${LANG_JAPANESE} "Standard installation"
 
  ;Assign language strings to sections
  !insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
    !insertmacro MUI_DESCRIPTION_TEXT ${SecBase} $(DESC_SecBase)
  !insertmacro MUI_FUNCTION_DESCRIPTION_END


;--------------------------------
;Uninstaller Section

Section "Uninstall"

  ;!insertmacro UNINSTALL.LOG_BEGIN_UNINSTALL
  !insertmacro UNINSTALL.LOG_UNINSTALL "$INSTDIR"
  !insertmacro UNINSTALL.LOG_END_UNINSTALL

  RMDir /r "$INSTDIR\tcl"

  Delete "$INSTDIR\uninstall.exe"

  Delete "${SCDIR}\Uninstall Virtual Worlds.lnk"
  Delete "${SCDIR}\BlocksWorld.lnk"
  RMDir "${SCDIR}"

  DeleteRegKey /ifempty ${INSTDIR_REG_ROOT} "${INSTDIR_REG_KEY}"

  ; Unregister with Windows' uninstall system
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PACKAGE_NAME}"

SectionEnd

;--------------------------------
;Uninstaller Functions

Function un.onInit
  !insertmacro MUI_UNGETLANGUAGE
  !insertmacro UNINSTALL.LOG_BEGIN_UNINSTALL
FunctionEnd
