#cs ----------------------------------------------------------------------------

 AutoIt Version: 3.3.14.2
 Author:         myName

 Script Function:
	Template AutoIt script.

#ce ----------------------------------------------------------------------------

; Script Start - Add your code below here

Opt("WinTitleMatchMode", 2)

WinActivate("DAVID 3D")
WinWaitActive("DAVID 3D")

$davidWindowPos = WinGetPos("[ACTIVE]")

$addToListX = $davidWindowPos[0] + 177
$addToListMonoY = $davidWindowPos[1] + 528;
$addToListStereoY = $davidWindowPos[1] + 740
MsgBox(0,"hello", "DavidVive4")

; Scan
Send("{F5}")

Sleep(500)

; Wait until scan complete
$disabledColorMono = PixelGetColor($addToListX, $addToListMonoY)
$disabledColorStereo = PixelGetColor($addToListX, $addToListStereoY)

;If $disabledColorMono = 0x090909 Then
;   $disabledColor = $disabledColorMono
;   $addToListY = $addToListMonoY
;Else
   $disabledColor = $disabledColorStereo
   $addToListY = $addToListStereoY
;EndIf

Do
   $currentColor = PixelGetColor($addToListX, $addToListY)
Until $currentColor <> $disabledColor

; Add to list
MouseClick("left", $addToListX, $addToListY)

Sleep(200)

; Select scan
MouseClick("left", $davidWindowPos[0] + $davidWindowPos[2] - 58, $davidWindowPos[1] + 268)

; Save to file
MouseClick("left", $davidWindowPos[0] + $davidWindowPos[2] - 58, $davidWindowPos[1] + 292)

Send("D:\Scans\currentScan{TAB}{DOWN}{DOWN}{DOWN}{ENTER}!s!y")

WinWaitActive("[TITLE:Save;REGEXPCLASS:DAVID4]")
$savePos = WinGetPos("[ACTIVE]")
MouseClick("left", $savePos[0] + 162, $savePos[1] + 182)

; Remove scan
MouseClick("left", $davidWindowPos[0] + $davidWindowPos[2] - 114, $davidWindowPos[1] + 293)

WinWaitActive("Confirmation")
$confirmPos = WinGetPos("[ACTIVE]")
MouseClick("left", $confirmPos[0] + 182, $confirmPos[1] + 164)