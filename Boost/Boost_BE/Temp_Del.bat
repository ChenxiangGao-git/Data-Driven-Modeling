cd %~dp0

del *.slxc
for /f %%i in ('dir /ad /b *slprj') do (rd /s /q %%i)