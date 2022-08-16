//
// Include the necessary resources
//
#include <winver.h>
#include <ntdef.h>

#ifdef RC_INVOKED

//
// Set up debug information
//
#if DBG
#define VER_DBG VS_FF_DEBUG
#else
#define VER_DBG 0
#endif

// ------- version info -------------------------------------------------------

VS_VERSION_INFO VERSIONINFO
FILEVERSION             1,0,0,0
PRODUCTVERSION          1,0,0,0
FILEFLAGSMASK           VS_FFI_FILEFLAGSMASK
FILEFLAGS               VER_DBG
FILEOS                  VOS_NT
FILETYPE                VFT_DRV
FILESUBTYPE             VFT2_DRV_SYSTEM
BEGIN
	BLOCK "StringFileInfo"
	BEGIN
		BLOCK "040904b0"
        BEGIN
		VALUE "Comments",         ""
		VALUE "CompanyName",      "Solar Stik, Inc."
		VALUE "FileDescription",  "24VDC HyPR 6000 Client"
		VALUE "FileVersion",      "V1.2"
		VALUE "InternalName",     "93-0000020"
		VALUE "LegalCopyright",   "(C)2022 Solar Stik, Inc."
		VALUE "OriginalFilename", "93-0000020.exe"
		VALUE "ProductName",      "24VDC HyPR 6000 Client"
		VALUE "ProductVersion",	  "V1.2"
        END
	END
	BLOCK "VarFileInfo"
	BEGIN
		VALUE "Translation", 0x0409,1200
	END
END
#endif