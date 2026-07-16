Set WshShell = CreateObject("WScript.Shell")
Dim strPath
strPath = CreateObject("Scripting.FileSystemObject").GetParentFolderName(WScript.ScriptFullName)
WshShell.CurrentDirectory = strPath
WshShell.Run "cmd.exe /c run_forever.bat", 0, False
