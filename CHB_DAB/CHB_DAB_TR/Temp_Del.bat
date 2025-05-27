cd %~dp0

del *.bakx
del *.psmx
del *.slxc
for /f %%i in ('dir /ad /b *if12') do (rd /s /q %%i)
for /f %%i in ('dir /ad /b *gf46') do (rd /s /q %%i)
for /f %%i in ('dir /ad /b *slprj') do (rd /s /q %%i)