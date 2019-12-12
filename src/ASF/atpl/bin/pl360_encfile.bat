@echo off
rem Parameters:
rem Param1 = Input file
rem Param2 = Output file containing CBC cyphered input.
rem Param3 = Cypher key [default = 230FE79BC18EF87CD51096E450C8C27F]
rem Param4 = Initialization Vector [default = 10456FD785DF2AF08397A2EB69647888]
rem Param5 = Signature key [default = F7962AD60E8C8EC26481358DB76F7F6A]

@set OPENSSL_CONF=C:\OpenSSL-Win32\bin\openssl.cfg

set input=%1
set output=%2
set cbc-key=%3
set iv-cbc=%4
set sign-key=%5

IF [%input%]   == []  set input="ATPL360B_G3_CENA.bin"
IF [%output%]   == []  set output="ATPL360B_G3_CENA_SEC.bin"
IF [%cbc-key%]  == []  set cbc-key=230FE79BC18EF87CD51096E450C8C27F 
IF [%iv-cbc%]   == []  set iv-cbc=10456FD785DF2AF08397A2EB69647888
IF [%sign-key%] == []  set sign-key=F7962AD60E8C8EC26481358DB76F7F6A

c:\python27\python.exe pl360_encfile.py -in %input% -out %output% -cbc-key %cbc-key% -cbc-iv %iv-cbc% -cmac-key %sign-key% -force_ivnbinc
pause